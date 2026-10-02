/* =====================================================================
 * hazard_ptr.c  -- see hazard_ptr.h for the API contract.
 * ===================================================================== */
#include "hazard_ptr.h"

#ifndef LF_RETIRE_CAPACITY
/* Must comfortably exceed the total number of hazard slots in the
 * system (LF_MAX_THREADS * LF_HP_SLOTS_PER_THREAD) so a scan always
 * has room to make progress. Tune down on tiny MCUs along with
 * LF_MAX_THREADS to control static RAM use
 * (capacity * sizeof(void*) bytes, per thread). */
#define LF_RETIRE_CAPACITY 128
#endif

/* One hazard-pointer row, and one retired-count, per thread -- each
 * padded/aligned to a full cache line (LF_ALIGN, atomic_hashdefs.h) so
 * two different threads' rows can never land on the same line. Without
 * this, hp_set()/hp_clear_all() (called on EVERY hop of every
 * lf_search()) or a retire-count increment on thread A invalidates the
 * cache line backing thread B's row too -- false sharing between
 * threads that have no logical relationship to each other, on what is
 * one of the hottest shared cache lines in the whole program under
 * real multi-thread contention. Slot/count value 0 means "unused". */
typedef struct {
    lf_atomic_uptr_t slot[LF_HP_SLOTS_PER_THREAD];
} LF_ALIGN(LF_CACHELINE_SIZE) lf_hp_row_t;

typedef struct {
    size_t count;
} LF_ALIGN(LF_CACHELINE_SIZE) lf_retired_count_row_t;

static lf_hp_row_t g_hp[LF_MAX_THREADS];

/* Per-thread retire (limbo) list -- only ever touched by its owning
 * thread id, so it needs no atomics of its own. */
static void   *g_retired[LF_MAX_THREADS][LF_RETIRE_CAPACITY];
static lf_retired_count_row_t g_retired_count[LF_MAX_THREADS];

static lf_reclaim_fn g_reclaim = NULL;

/* See hp_set_active_threads() in hazard_ptr.h for the usage contract
 * (set once, before any thread that might race on this starts). Not
 * atomic on purpose: the intended call pattern (main thread sets this,
 * *then* spawns workers) is already ordered by pthread_create()'s own
 * happens-before guarantee, so adding atomics here would only cost
 * cycles on the scan's hot path for no additional guarantee. Defaults
 * to LF_MAX_THREADS, i.e. identical to the scan range before this
 * option existed -- fully backward compatible unless an app opts in. */
static int g_active_threads = LF_MAX_THREADS;

void hp_set_active_threads(int n) {
    if (n < 1) {
        n = 1;
    } else if (n > LF_MAX_THREADS) {
        n = LF_MAX_THREADS;
    }
    g_active_threads = n;
}

void hp_init(lf_reclaim_fn reclaim) {
    int t, s;
    g_reclaim = reclaim;
    for (t = 0; t < LF_MAX_THREADS; t++) {
        for (s = 0; s < LF_HP_SLOTS_PER_THREAD; s++) {
            lf_store_uptr(&g_hp[t].slot[s], 0);
        }
        g_retired_count[t].count = 0;
    }
}

void hp_set(int tid, int slot, void *ptr) {
    /* lf_store_uptr() is already memory_order_seq_cst (see
     * atomic_hashdefs.h): on x86-64/GCC this compiles to `xchg`, which
     * IS a full hardware barrier by itself (verified by inspecting the
     * generated assembly -- `xchgq` has an implicit LOCK, exactly the
     * same full-fence property `lf_fence()` provides via `lock orq $0,
     * (%rsp)`). A prior version of this function followed the store
     * with an additional explicit lf_fence(), which -- on this target
     * at least -- compiled to a second, entirely redundant full
     * barrier: two `lock`-prefixed instructions doing the same job,
     * on a function called on every single hop of every lf_search().
     * Removed. If you port this to a toolchain/architecture where a
     * seq_cst store does NOT already imply a full fence (none of the
     * backends in atomic_hashdefs.h currently behave that way, but a
     * future one might), that backend's lf_store_uptr() should be
     * fixed to make seq_cst mean seq_cst -- not patched over by
     * re-adding a fence at every call site like this one. */
    lf_store_uptr(&g_hp[tid].slot[slot], (uintptr_t)ptr);
}

void hp_clear_all(int tid) {
    int s;
    for (s = 0; s < LF_HP_SLOTS_PER_THREAD; s++) {
        lf_store_uptr(&g_hp[tid].slot[s], 0);
    }
}

static int is_hazardous(void *ptr) {
    int t, s;
    uintptr_t p = (uintptr_t)ptr;
    /* Bounded by g_active_threads, not always LF_MAX_THREADS: an app
     * using far fewer threads than LF_MAX_THREADS's default of 32 (see
     * hp_set_active_threads()) would otherwise pay for scanning rows
     * that are provably still all-zero for the whole run. Default
     * (g_active_threads == LF_MAX_THREADS unless overridden) is
     * identical to the original unconditional scan. */
    for (t = 0; t < g_active_threads; t++) {
        for (s = 0; s < LF_HP_SLOTS_PER_THREAD; s++) {
            if (lf_load_uptr(&g_hp[t].slot[s]) == p) {
                return 1;
            }
        }
    }
    return 0;
}

static void scan_and_reclaim(int tid) {
    size_t i, kept = 0;
    /* Belt-and-suspenders full fence before scanning: lf_cas_uptr/
     * lf_load_uptr/lf_store_uptr are already seq_cst on every backend
     * (see atomic_hashdefs.h), which is sufficient on its own, but an
     * explicit fence here costs nothing on the (infrequent, batched)
     * reclamation path and makes the "scanner side is also fenced"
     * requirement visible at the call site rather than only implicit
     * in the primitive's chosen memory order. (Unlike the hp_set() fix
     * above, this one is NOT redundant: is_hazardous()'s loop is pure
     * seq_cst loads, and a stray fence before a batch of loads that
     * were going to happen anyway is free relative to how rarely this
     * whole function runs -- it is not on the hot per-hop path.) */
    lf_fence();
    for (i = 0; i < g_retired_count[tid].count; i++) {
        void *p = g_retired[tid][i];
        if (is_hazardous(p)) {
            g_retired[tid][kept++] = p;
        } else if (g_reclaim) {
            g_reclaim(p);
        }
    }
    g_retired_count[tid].count = kept;
}

void hp_flush(int tid) {
    scan_and_reclaim(tid);
}

void hp_retire(int tid, void *ptr) {
    if (g_retired_count[tid].count >= LF_RETIRE_CAPACITY) {
        scan_and_reclaim(tid);
    }
    if (g_retired_count[tid].count >= LF_RETIRE_CAPACITY) {
        /* Pathological case: more nodes are simultaneously
         * hazard-protected than LF_RETIRE_CAPACITY allows. Retry the
         * scan a few times (other threads may be mid-traversal and
         * about to clear their slots) before giving up on the oldest
         * entry. Reaching this path means LF_RETIRE_CAPACITY should
         * be raised relative to LF_MAX_THREADS for this workload. */
        int attempts = 8;
        while (attempts-- > 0 && g_retired_count[tid].count >= LF_RETIRE_CAPACITY) {
            lf_cpu_relax();
            scan_and_reclaim(tid);
        }
    }
    if (g_retired_count[tid].count < LF_RETIRE_CAPACITY) {
        g_retired[tid][g_retired_count[tid].count++] = ptr;
    }
    /* else: deliberately leaked with a documented cause above, rather
     * than freeing a potentially still-referenced node. */
}
