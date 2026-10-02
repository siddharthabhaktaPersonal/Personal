/* =====================================================================
 * lockfree_list.h
 *
 * A lock-free, sorted, singly linked set of (key, value) pairs.
 * Implements the Harris (2001) / Michael (2002) algorithm:
 *   - logical deletion via a "mark bit" stolen from the LSB of each
 *     node's `next` pointer (nodes are pointer-aligned, so bit 0 is
 *     always free for real pointers)
 *   - physical unlinking is opportunistic, performed by whichever
 *     thread's single-word CAS happens to win
 *   - safe reclamation of physically-unlinked nodes via hazard
 *     pointers (hazard_ptr.h), so freed memory is never touched by a
 *     concurrent reader -- this is what makes the design correct
 *     without needing a double-word/DWCAS or a GC.
 *
 * Progress guarantee: lock-free (some thread always makes progress;
 * no thread can block another by dying/being descheduled while
 * holding a lock, because there is no lock).
 *
 * Concurrency model: every API call takes an explicit `tid` in
 * [0, LF_MAX_THREADS) identifying the calling thread/core, used only
 * to index its hazard-pointer slots (see hazard_ptr.h for why this
 * replaces compiler TLS). Two different logical threads must never
 * reuse the same tid concurrently.
 * ===================================================================== */
#ifndef LF_LOCKFREE_LIST_H
#define LF_LOCKFREE_LIST_H

#include <stddef.h>

__inline__ unsigned long long rdtsc(void)
{
#define RDTSC_MAX ULLONG_MAX
  unsigned hi, lo;
  __asm__ __volatile__ ("rdtsc" : "=a"(lo), "=d"(hi));
  return ( (unsigned long long)lo)|( ((unsigned long long)hi)<<32 );
}

typedef struct lf_node lf_node_t;

typedef struct {
    lf_node_t *head; /* sentinel, key == LF_KEY_MIN, never removed */
    lf_node_t *tail; /* sentinel, key == LF_KEY_MAX, never removed */
} lf_list_t;

/* Call once, before any thread touches the list. Also brings up the
 * global hazard-pointer subsystem (do not call hp_init separately). */
void lf_list_init(lf_list_t *list);

/* Single-threaded teardown only -- caller must guarantee no other
 * thread is concurrently accessing `list` when this runs. */
void lf_list_destroy(lf_list_t *list);

/* Returns 1 if `key` was absent and is now inserted with `value`.
 * Returns 0 if `key` was already present (list unchanged). */
int lf_list_insert(lf_list_t *list, long key, void *value, int tid);

/* Returns 1 if `key` was present and is now removed.
 * Returns 0 if `key` was not found. */
int lf_list_delete(lf_list_t *list, long key, int tid);

/* Returns 1 and (if out_value != NULL) writes the associated value if
 * `key` is present; returns 0 if not found. */
int lf_list_find(const lf_list_t *list, long key, void **out_value, int tid);

/* Debug helper: single-threaded-only in-order dump; NOT lock-free
 * safe to call while other threads are mutating the list. */
void lf_list_dump(const lf_list_t *list, void (*visit)(long key, void *value));

extern volatile unsigned long long cons_cycles, cons_count;
extern volatile unsigned long long prod_cycles, prod_count;

#endif /* LF_LOCKFREE_LIST_H */
