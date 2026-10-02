/* =====================================================================
 * lockfree_queue.h
 *
 * A lock-free, unordered-by-key MPMC (Multi-Producer/Multi-Consumer)
 * FIFO queue: just two operations, "insert" (enqueue) and "fetch
 * first" (dequeue-the-oldest-item) -- no key, no search.
 *
 * This is added ALONGSIDE lockfree_list.[ch] (that sorted-set code is
 * untouched) as a separate, independent data structure for callers
 * who don't need key-ordered storage and want the fastest possible
 * lock-free hand-off between producers and consumers. It exists
 * because of a very direct finding from profiling the sorted list:
 * that structure's insert/delete cost is dominated by lf_search()'s
 * O(n) walk from head to the right key position, which is *inherent*
 * to keeping the list sorted -- it is not a fixable inefficiency in
 * lf_list_insert()/lf_list_delete() themselves. This queue sidesteps
 * that entirely: it never searches for anything. Every insert() links
 * exactly one new node at the tail; every fetch_first() unlinks
 * exactly one node at the head. Both are O(1), independent of how
 * many items are currently in the queue -- so latency here does not
 * grow as the queue fills up the way it does for the sorted list.
 *
 * ALGORITHM: Michael & Scott (1996), "Simple, Fast, and Practical
 * Non-Blocking and Blocking Concurrent Queue Algorithms" -- the
 * standard lock-free MPMC queue design, combined with hazard pointers
 * (Michael, 2004 -- the same paper that introduces hazard pointers
 * uses exactly this queue as its running example) for safe
 * reclamation of dequeued nodes. Concretely:
 *   - A permanent dummy/sentinel node sits at Head at all times: the
 *     real, current head-of-queue value lives in Head->next, not in
 *     Head itself. This is what lets both enqueue and dequeue proceed
 *     with a single CAS each, with no special-casing of "queue has
 *     exactly one element" -- that classic lock-free queue trick.
 *   - enqueue(value): CAS a new node onto Tail->next, then (best
 *     effort, may be "helped" by another thread if this thread stalls
 *     or dies) CAS Q->Tail to the new node.
 *   - dequeue(): CAS Q->Head from the dummy to Head->next, reading the
 *     value out of the node that is about to become the new dummy;
 *     the old dummy is now unreachable and is hp_retire()'d.
 *   - No mark bit is needed here (unlike lockfree_list.c): removal is
 *     a single CAS that atomically both "logically" and "physically"
 *     unlinks the node, so there is no separate logical-delete step
 *     to race on.
 *
 * PORTABILITY / REUSE: built entirely on the same primitives as
 * lockfree_list.c -- atomic_hashdefs.h's lf_atomic_uptr_t / lf_cas_uptr
 * / lf_load_uptr / lf_store_uptr / lf_fence / lf_cpu_relax (so it
 * inherits the identical x86 CMPXCHG vs ARM/PowerPC LL/SC portability
 * story), and hazard_ptr.h's hp_init/hp_set/hp_clear_all/hp_retire/
 * hp_flush/hp_set_active_threads, which are already fully generic
 * (void* payloads, explicit tid, pluggable reclaim function) and were
 * not written with the list specifically in mind.
 *
 * IMPORTANT -- hazard-pointer subsystem is a SINGLE GLOBAL INSTANCE:
 * hazard_ptr.c has one process-wide set of hazard-pointer tables, not
 * one per data structure. lf_list_init() and lf_queue_init() each call
 * hp_init() to bring that subsystem up. That is exactly right when a
 * program uses ONLY the list or ONLY the queue (each of this project's
 * test binaries links exactly one of the two, by design -- see the
 * Makefile's `test`/`pc-test` targets vs the new `qtest` target).
 * If you want to use lf_list_t and lf_queue_t at the same time in one
 * process, call hp_init() yourself exactly once (e.g. via whichever of
 * lf_list_init()/lf_queue_init() runs first) and do not call the
 * other one's *_init() a second time -- reusing the shared tid space
 * across both structures is fine (a thread only ever has one call to
 * a list/queue function in flight at a time, and hp_clear_all() at the
 * end of every call leaves both hazard slots clean for the next call,
 * whichever structure it targets), but a second hp_init() call would
 * reset the table out from under the other structure.
 * ===================================================================== */
#ifndef LF_LOCKFREE_QUEUE_H
#define LF_LOCKFREE_QUEUE_H

#include <stddef.h>
#include "atomic_hashdefs.h"

/* Timing is opt-in because shared profiling counters contend between
 * producers and consumers. Non-x86 builds can supply LFQ_READ_CYCLES(). */
#ifndef LF_QUEUE_ENABLE_STATS
#define LF_QUEUE_ENABLE_STATS 0
#endif

#if LF_QUEUE_ENABLE_STATS
/* The bundled timing harness enables stats on x86. Portable library builds
 * leave them off; targets can provide LFQ_READ_CYCLES() for a native timer. */
#ifndef LFQ_READ_CYCLES
#if defined(_MSC_VER) && (defined(_M_X64) || defined(_M_IX86))
#include <intrin.h>
#define LFQ_READ_CYCLES() __rdtsc()
#elif defined(__i386__) || defined(__x86_64__)
static inline unsigned long long lfq_rdtsc(void) {
    unsigned hi, lo;
    __asm__ __volatile__("rdtsc" : "=a"(lo), "=d"(hi));
    return ((unsigned long long)lo) | (((unsigned long long)hi) << 32);
}
#define LFQ_READ_CYCLES() lfq_rdtsc()
#else
#error "Define LFQ_READ_CYCLES() when LF_QUEUE_ENABLE_STATS is enabled on this target"
#endif
#endif
#endif

typedef struct lf_qnode lf_qnode_t; /* opaque; defined in lockfree_queue.c */

typedef struct {
    lf_atomic_uptr_t value;
    unsigned char padding[LF_CACHELINE_SIZE - sizeof(lf_atomic_uptr_t)];
} lf_queue_atomic_line_t;

typedef struct {
    lf_queue_atomic_line_t head; /* permanent dummy/sentinel */
    lf_queue_atomic_line_t tail; /* may transiently lag the real last node */
} lf_queue_t;

/* Queue layout changed to isolate head and tail on separate cache lines;
 * rebuild callers when upgrading this header. */

/* Call once, before any thread touches the queue. Also brings up the
 * global hazard-pointer subsystem -- see the big comment above about
 * combining this with lf_list_t in one process. */
void lf_queue_init(lf_queue_t *q);

/* Single-threaded teardown only -- caller must guarantee no other
 * thread is concurrently accessing `q` when this runs. */
void lf_queue_destroy(lf_queue_t *q);

/* Enqueue `value` at the tail. O(1): no search, unlike lf_list_insert().
 * Always succeeds and returns 1, unless the internal node allocation
 * fails, in which case it returns 0 and the queue is unchanged. */
int lf_queue_insert(lf_queue_t *q, void *value, int tid);

/* Dequeue the oldest (longest-queued) value, i.e. "fetch first": if
 * the queue is non-empty, writes it to *out_value (when out_value !=
 * NULL) and returns 1; if empty, returns 0 and *out_value is
 * untouched. O(1): no search, unlike lf_list_delete(). */
int lf_queue_fetch_first(lf_queue_t *q, void **out_value, int tid);

/* Optional cycle accumulators. Compile lockfree_queue.c with
 * -DLF_QUEUE_ENABLE_STATS=1 to collect them. Stats are off by default.
 * When enabled outside x86, define LFQ_READ_CYCLES() to a target timer. */
extern volatile unsigned long long qins_cycles, qins_count;
extern volatile unsigned long long qdeq_cycles, qdeq_count;

#endif /* LF_LOCKFREE_QUEUE_H */
