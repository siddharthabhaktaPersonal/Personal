/* =====================================================================
 * lockfree_queue.c -- Michael & Scott lock-free MPMC FIFO queue.
 * See lockfree_queue.h for the API contract and algorithm summary.
 * ===================================================================== */
#include <stdlib.h>
#include "lockfree_queue.h"
#include "atomic_hashdefs.h"
#include "hazard_ptr.h"

volatile unsigned long long qins_cycles = 0, qins_count = 0;
volatile unsigned long long qdeq_cycles = 0, qdeq_count = 0;

/* Hazard-pointer slot roles (must fit within LF_HP_SLOTS_PER_THREAD ==
 * 2, same as lockfree_list.c -- see hazard_ptr.h). insert() only ever
 * needs one slot at a time (TAIL); fetch_first() needs two (HEAD and
 * NEXT) simultaneously. Reusing slot 0/1 across the two operations is
 * safe because a single tid never has an insert() and a fetch_first()
 * both in flight at once (hp_clear_all() at the end of each call
 * leaves both slots free for whichever call comes next). */
#define HP_Q_TAIL 0
#define HP_Q_HEAD 0
#define HP_Q_NEXT 1

struct lf_qnode {
    lf_atomic_uptr_t next; /* -> lf_qnode_t, plain pointer, no mark bit needed here */
    void *value;
};

static void reclaim_qnode(void *p) {
    free(p);
}

void lf_queue_init(lf_queue_t *q) {
    lf_qnode_t *dummy;

    hp_init(reclaim_qnode);

    dummy = (lf_qnode_t *)malloc(sizeof(lf_qnode_t));
    dummy->value = NULL;
    lf_store_uptr(&dummy->next, 0);

    lf_store_uptr(&q->head, (uintptr_t)dummy);
    lf_store_uptr(&q->tail, (uintptr_t)dummy);
    lf_fence();
}

void lf_queue_destroy(lf_queue_t *q) {
    int t;
    lf_qnode_t *cur = (lf_qnode_t *)lf_load_uptr(&q->head);

    while (cur != NULL) {
        lf_qnode_t *next = (lf_qnode_t *)lf_load_uptr(&cur->next);
        reclaim_qnode(cur);
        cur = next;
    }
    lf_store_uptr(&q->head, 0);
    lf_store_uptr(&q->tail, 0);

    /* Same rationale as lf_list_destroy(): drain every thread's
     * retire list so a node this destroy() didn't see via the chain
     * walk above (because a concurrent-with-shutdown dequeue had
     * already physically unlinked it but its hazard-pointer scan
     * hadn't run yet) doesn't leak. */
    for (t = 0; t < LF_MAX_THREADS; t++) {
        hp_flush(t);
    }
}

/* enqueue -- Michael & Scott (1996) Figure 1, "enqueue". CAS a new
 * node onto the current tail's next field, then (best-effort) swing
 * Q->Tail to it. If this thread stalls right after the first CAS and
 * before the second, any OTHER thread's insert()/fetch_first() that
 * notices Tail->next != NULL will "help" by advancing Q->Tail itself
 * -- that's what the `else` branch below does -- so the queue can
 * never get wedged behind a slow/descheduled producer. */
int lf_queue_insert(lf_queue_t *q, void *value, int tid) {
    unsigned long long start, end;
    lf_qnode_t *node = (lf_qnode_t *)malloc(sizeof(lf_qnode_t));
    uintptr_t tail_raw, next_raw;

    if (node == NULL) {
        return 0;
    }
    node->value = value;
    lf_store_uptr(&node->next, 0);

    start = lfq_rdtsc();

    for (;;) {
        lf_qnode_t *tail;

        tail_raw = lf_load_uptr(&q->tail);
        tail = (lf_qnode_t *)tail_raw;
        hp_set(tid, HP_Q_TAIL, tail);
        /* Publish-then-validate (same discipline as lf_search() in
         * lockfree_list.c): if Q->Tail already moved on from the
         * snapshot we just protected, `tail` may already be retired
         * elsewhere -- don't dereference it, just retry. */
        if (lf_load_uptr(&q->tail) != tail_raw) {
            continue;
        }

        next_raw = lf_load_uptr(&tail->next);
        /* Re-check Tail is still consistent with what we read next
         * from (M&S's own "if (tail == Q->Tail)" double-check) before
         * acting on `next_raw`. */
        if (lf_load_uptr(&q->tail) != tail_raw) {
            continue;
        }

        if (next_raw == 0) {
            /* tail really is the last node: try to link our new node
             * onto it. */
            if (lf_cas_uptr(&tail->next, 0, (uintptr_t)node)) {
                /* Success -- try to swing Tail to the new node too.
                 * Whether or not this CAS wins doesn't matter for
                 * correctness: if it loses, some other thread already
                 * did it for us (the `else` branch below, run from
                 * inside their own insert() or fetch_first()). */
                lf_cas_uptr(&q->tail, tail_raw, (uintptr_t)node);
                break;
            }
        } else {
            /* Tail is lagging one node behind the real end of the
             * list (another producer finished its first CAS but not
             * its second) -- help it catch up before retrying. */
            lf_cas_uptr(&q->tail, tail_raw, next_raw);
        }
        lf_cpu_relax();
    }

    hp_clear_all(tid);
    end = lfq_rdtsc();
    /* __atomic_fetch_add, not '+=' -- see the identical comment in
     * lockfree_list.c's lf_list_insert(): these counters are written
     * from every producer thread, so a plain '+=' would be a lost-
     * update data race under concurrent callers. __ATOMIC_RELAXED is
     * enough since they carry no ordering relationship with anything
     * else and are pure statistics. */
    __atomic_fetch_add(&qins_cycles, (end - start), __ATOMIC_RELAXED);
    __atomic_fetch_add(&qins_count, 1, __ATOMIC_RELAXED);
    return 1;
}

/* dequeue / "fetch first" -- Michael & Scott (1996) Figure 1,
 * "dequeue", using the hazard-pointer variant from Michael (2004)
 * (protect Head, then Head->next, before touching either). Head
 * always points at a dummy node; the real oldest value lives in
 * Head->next. A successful dequeue CASes Q->Head from the old dummy
 * to that node (which becomes the new dummy) and hands back the value
 * read out of it; the old dummy is now unreachable and is retired. */
int lf_queue_fetch_first(lf_queue_t *q, void **out_value, int tid) {
    unsigned long long start, end;
    lf_qnode_t *head, *tail, *next;
    uintptr_t head_raw, tail_raw, next_raw;
    void *value = NULL;

    start = lfq_rdtsc();

    for (;;) {
        head_raw = lf_load_uptr(&q->head);
        head = (lf_qnode_t *)head_raw;
        hp_set(tid, HP_Q_HEAD, head);
        if (lf_load_uptr(&q->head) != head_raw) {
            continue; /* head already moved/retired; restart, don't deref */
        }

        tail_raw = lf_load_uptr(&q->tail);
        tail = (lf_qnode_t *)tail_raw;

        next_raw = lf_load_uptr(&head->next);
        next = (lf_qnode_t *)next_raw;
        hp_set(tid, HP_Q_NEXT, next);
        /* Re-validate Head is still what we protected before we rely
         * on `next` being a safe-to-dereference snapshot of it (M&S's
         * "if (Q->Head == head)" check) -- see lf_queue_insert()'s
         * matching comment for why this order matters. */
        if (lf_load_uptr(&q->head) != head_raw) {
            continue;
        }

        if (head == tail) {
            if (next == NULL) {
                /* Head == Tail and Head->next == NULL: genuinely
                 * empty. */
                hp_clear_all(tid);
                end = lfq_rdtsc();
                __atomic_fetch_add(&qdeq_cycles, (end - start), __ATOMIC_RELAXED);
                __atomic_fetch_add(&qdeq_count, 1, __ATOMIC_RELAXED);
                return 0;
            }
            /* Tail is lagging behind a completed-but-not-yet-swung
             * insert() (see lf_queue_insert()'s `else` branch) --
             * help it catch up, then retry. */
            lf_cas_uptr(&q->tail, tail_raw, next_raw);
        } else {
            /* head != tail implies head->next cannot be NULL (head is
             * not the last node), so it's always safe to read the
             * value out of it here. */
            value = next->value;
            if (lf_cas_uptr(&q->head, head_raw, next_raw)) {
                break; /* dequeued; `head` (the old dummy) is now unlinked */
            }
        }
        lf_cpu_relax();
    }

    hp_retire(tid, head);
    hp_clear_all(tid);
    end = lfq_rdtsc();
    __atomic_fetch_add(&qdeq_cycles, (end - start), __ATOMIC_RELAXED);
    __atomic_fetch_add(&qdeq_count, 1, __ATOMIC_RELAXED);
    if (out_value != NULL) {
        *out_value = value;
    }
    return 1;
}
