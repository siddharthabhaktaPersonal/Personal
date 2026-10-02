/* =====================================================================
 * lockfree_list.c -- Harris/Michael lock-free sorted linked list.
 * See lockfree_list.h for the API contract and algorithm summary.
 * ===================================================================== */
#include <stdlib.h>
#include <limits.h>
#include "lockfree_list.h"
#include "atomic_hashdefs.h"
#include "hazard_ptr.h"

#define LF_KEY_MIN LONG_MIN
#define LF_KEY_MAX LONG_MAX

volatile unsigned long long cons_cycles=0, cons_count=0;
volatile unsigned long long prod_cycles=0, prod_count=0;

/* Per-operation timing and shared statistics are useful in the bundled
 * benchmark, but the two global atomic increments serialize every writer.
 * Keep them opt-in for applications that care about throughput. */
#ifndef LF_ENABLE_STATS
#define LF_ENABLE_STATS 0
#endif

#if LF_ENABLE_STATS
#define LF_STATS_DECLARE(var) unsigned long long var
#define LF_STATS_START(var) ((var) = rdtsc())
#define LF_STATS_ADD(cycles, count, start) do { \
    unsigned long long lf_end__ = rdtsc(); \
    __atomic_fetch_add(&(cycles), (lf_end__ - (start)), __ATOMIC_RELAXED); \
    __atomic_fetch_add(&(count), 1, __ATOMIC_RELAXED); \
} while (0)
#else
#define LF_STATS_DECLARE(var)
#define LF_STATS_START(var) ((void)0)
#define LF_STATS_ADD(cycles, count, start) ((void)0)
#endif

/* Hazard-pointer slot roles used by lf_search() below (must match
 * LF_HP_SLOTS_PER_THREAD == 2 in hazard_ptr.h):
 *   HP_PRED - protects the current predecessor ("pred"): the last
 *             node confirmed live (not logically deleted) that we
 *             are walking forward from. Only reassigned when pred
 *             itself advances to a newly-validated live node.
 *   HP_CAND - protects the "candidate" node under validation on the
 *             current hop, before it is safe to dereference. */
#define HP_PRED 0
#define HP_CAND 1

struct lf_node {
    lf_atomic_uptr_t next; /* (struct lf_node *) | mark_bit, see below */
    long   key;
    void  *value;
};

/* ---- mark-bit helpers: steal bit 0 of the next pointer ------------- */
static inline int is_marked(uintptr_t v) {
    return (int)(v & (uintptr_t)1);
}
static inline lf_node_t *unmarked(uintptr_t v) {
    return (lf_node_t *)(v & ~(uintptr_t)1);
}
static inline uintptr_t marked_of(lf_node_t *p) {
    return ((uintptr_t)p) | (uintptr_t)1;
}

static void reclaim_node(void *p) {
    free(p);
}

void lf_list_init(lf_list_t *list) {
    hp_init(reclaim_node);

    list->head = (lf_node_t *)malloc(sizeof(lf_node_t));
    list->tail = (lf_node_t *)malloc(sizeof(lf_node_t));

    list->head->key = LF_KEY_MIN;
    list->head->value = NULL;
    list->tail->key = LF_KEY_MAX;
    list->tail->value = NULL;

    lf_store_uptr(&list->tail->next, 0);
    lf_store_uptr(&list->head->next, (uintptr_t)list->tail);
    lf_fence();
}

void lf_list_destroy(lf_list_t *list) {
    int t;
    lf_node_t *cur = list->head;
    while (cur != NULL) {
        lf_node_t *next = unmarked(lf_load_uptr(&cur->next));
        reclaim_node(cur);
        cur = next;
    }
    list->head = list->tail = NULL;

    /* A node a prior delete() physically unlinked is not reachable
     * from the chain walk above -- it sits in the retiring thread's
     * per-thread limbo list until a hazard-pointer scan clears it,
     * which may not have happened yet if that thread's retire count
     * never hit LF_RETIRE_CAPACITY. Force a final reclamation pass
     * for every thread so destroy() doesn't leak those. Safe here: by
     * the time a caller destroys the list, no thread should still be
     * inside an insert/delete/find call publishing a hazard pointer
     * into it. */
    for (t = 0; t < LF_MAX_THREADS; t++) {
        hp_flush(t);
    }
}

/*
 * search(): locate the window (pred, curr) such that
 *   pred->key < key, pred is live, and pred->next == curr (no marked
 * nodes in between -- any marked node is physically unlinked the
 * moment it is encountered, "helping" other threads as we go).
 * Returns 1 iff curr != tail and curr->key == key.
 *
 * Every pointer is validated (protected via a hazard pointer, THEN
 * re-checked against the field it was read from) before it is ever
 * dereferenced. This publish-then-validate discipline is what makes
 * it safe against a concurrent thread retiring and freeing a node in
 * the window between our reading its address and protecting it --
 * see hazard_ptr.h and the design notes in README.md.
 *
 * A second, subtler hazard has to be handled alongside that one: what
 * if "pred" itself -- not curr -- is concurrently marked (logically
 * deleted) by another thread, after we have already promoted it to be
 * our predecessor? A marked node's next field is frozen forever from
 * that point on (nobody physically relinks a node's own next pointer
 * once it has been marked, they only unlink it from ITS predecessor),
 * so re-validating a read against a "zombie" pred's next field always
 * trivially succeeds -- it never changes again. If we kept walking
 * forward from a zombie pred anyway, this thread could derive a
 * "curr" from an arbitrarily stale snapshot: one that, by the time we
 * get around to publishing a hazard pointer on it, may already have
 * been legitimately retired *and reclaimed* via the real, live
 * predecessor chain elsewhere in the list. This thread would have had
 * no window in which its hazard pointer could have been published in
 * time to prevent that reclamation -- a genuine use-after-free, not
 * merely a lost race. So every re-validation below must check not
 * just "did pred->next change" but "did pred->next change *because
 * pred itself got marked*" -- and if so, abandon pred and restart the
 * search from head, exactly as on a failed CAS.
 */
static int lf_search(const lf_list_t *list, long key,
                      lf_node_t **out_left, lf_node_t **out_right, int tid) {
    lf_node_t *pred, *curr;
    uintptr_t pred_next_raw, curr_next_raw;

try_again:
    pred = list->head;              /* permanent sentinel, never freed */
    hp_set(tid, HP_PRED, pred);
    pred_next_raw = lf_load_uptr(&pred->next);

    for (;;) {
        curr = unmarked(pred_next_raw);

        if (curr == list->tail) {
            *out_left = pred;
            *out_right = curr;
            return 0;
        }

        hp_set(tid, HP_CAND, curr);
        /* Validate: re-read the exact field curr was derived from.
         * If it changed, curr may already be retired/freed -- do not
         * dereference it; just re-derive from (still-protected) pred.
         * Unless the change is pred itself having been marked (see
         * the big comment above lf_search): that pred is a zombie and
         * must be abandoned, not silently "re-derived" from. */
        {
            uintptr_t recheck = lf_load_uptr(&pred->next);
            if (recheck != pred_next_raw) {
                if (is_marked(recheck)) {
                    goto try_again;
                }
                pred_next_raw = recheck;
                continue;
            }
        }

        /* curr is now guaranteed safe to dereference. */
        curr_next_raw = lf_load_uptr(&curr->next);

        if (is_marked(curr_next_raw)) {
            /* curr is logically deleted: physically unlink it now. */
            if (!lf_cas_uptr(&pred->next, pred_next_raw, (uintptr_t)unmarked(curr_next_raw))) {
                goto try_again; /* lost a race; simplest correct recovery */
            }
            hp_retire(tid, curr);
            pred_next_raw = (uintptr_t)unmarked(curr_next_raw);
            continue; /* pred unchanged (still protected in HP_PRED) */
        }

        if (curr->key < key) {
            /* curr is live and precedes the key: advance pred to curr. */
            hp_set(tid, HP_PRED, curr);
            pred = curr;
            pred_next_raw = curr_next_raw; /* known unmarked here */
            continue;
        }

        /* curr is live and curr->key >= key: this is our window. */
        *out_left = pred;
        *out_right = curr;
        return (curr->key == key);
    }
}

int lf_list_insert(lf_list_t *list, long key, void *value, int tid) {
    LF_STATS_DECLARE(start);
    lf_node_t *left, *right;
    lf_node_t *new_node = (lf_node_t *)malloc(sizeof(lf_node_t));
    new_node->key = key;
    new_node->value = value;

    LF_STATS_START(start);

    for (;;) {
        if (lf_search(list, key, &left, &right, tid)) {
            free(new_node);
            hp_clear_all(tid);
            /* __atomic_fetch_add, not '+=' -- these globals are shared
             * across every thread calling lf_list_insert()/delete()
             * (see test_lockfree_list.c's N-writer stress test), and a
             * plain '+=' on a shared variable is a classic lost-update
             * race: two threads' read-modify-write can interleave and
             * one increment silently vanishes. TSan catches this
             * exact bug (data race on 'prod_cycles'/'prod_count') the
             * first time it's built with more than one writer thread.
             * __ATOMIC_RELAXED is enough here -- these counters have
             * no ordering relationship with anything else, they're
             * pure statistics -- and deliberately not seq_cst, so this
             * instrumentation doesn't itself add extra full-barrier
             * cost to the very latency it's trying to measure. */
            LF_STATS_ADD(prod_cycles, prod_count, start);
            return 0; /* already present */
        }
        lf_store_uptr(&new_node->next, (uintptr_t)right);
        if (lf_cas_uptr(&left->next, (uintptr_t)right, (uintptr_t)new_node)) {
            hp_clear_all(tid);
            /* __atomic_fetch_add, not '+=' -- these globals are shared
             * across every thread calling lf_list_insert()/delete()
             * (see test_lockfree_list.c's N-writer stress test), and a
             * plain '+=' on a shared variable is a classic lost-update
             * race: two threads' read-modify-write can interleave and
             * one increment silently vanishes. TSan catches this
             * exact bug (data race on 'prod_cycles'/'prod_count') the
             * first time it's built with more than one writer thread.
             * __ATOMIC_RELAXED is enough here -- these counters have
             * no ordering relationship with anything else, they're
             * pure statistics -- and deliberately not seq_cst, so this
             * instrumentation doesn't itself add extra full-barrier
             * cost to the very latency it's trying to measure. */
            LF_STATS_ADD(prod_cycles, prod_count, start);
            return 1;
        }
        lf_cpu_relax(); /* lost the race against a concurrent mutator; retry */
    }

    return 0;
}

int lf_list_delete(lf_list_t *list, long key, int tid) {
    LF_STATS_DECLARE(start);
    lf_node_t *left, *right;
    uintptr_t right_next_val;

    LF_STATS_START(start);

    for (;;) {
        if (!lf_search(list, key, &left, &right, tid)) {
            hp_clear_all(tid);
            LF_STATS_ADD(cons_cycles, cons_count, start);
            return 0; /* not found */
        }
        right_next_val = lf_load_uptr(&right->next);
        if (!is_marked(right_next_val)) {
            if (lf_cas_uptr(&right->next, right_next_val,
                             marked_of(unmarked(right_next_val)))) {
                break; /* logically deleted */
            }
        }
        lf_cpu_relax(); /* lost the race; re-search and try again */
    }

    /* Best-effort physical unlink right away; if it loses the race,
     * the very next search() through this region will finish the job
     * (that's the "helping" property of the algorithm). */
    if (lf_cas_uptr(&left->next, (uintptr_t)right, (uintptr_t)unmarked(right_next_val))) {
        hp_retire(tid, right);
    }
    hp_clear_all(tid);
    LF_STATS_ADD(cons_cycles, cons_count, start);
    return 1;
}

int lf_list_find(const lf_list_t *list, long key, void **out_value, int tid) {
    lf_node_t *left, *right;
    int found = lf_search(list, key, &left, &right, tid);
    if (found && out_value) {
        *out_value = right->value;
    }
    hp_clear_all(tid);
    return found;
}

void lf_list_dump(const lf_list_t *list, void (*visit)(long key, void *value)) {
    lf_node_t *n = unmarked(lf_load_uptr(&list->head->next));
    while (n != list->tail) {
        uintptr_t nv = lf_load_uptr(&n->next);
        if (!is_marked(nv)) {
            visit(n->key, n->value);
        }
        n = unmarked(nv);
    }
}
