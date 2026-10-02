/* =====================================================================
 * hazard_ptr.h
 *
 * Minimal Hazard Pointer (Michael, 2004) safe-memory-reclamation
 * module used by lockfree_list.c so that CAS-unlinked nodes are never
 * freed while another thread still holds a raw pointer to them (the
 * classic lock-free "use after free instead of ABA" hazard).
 *
 * Deliberately does NOT use compiler thread-local storage (__thread /
 * _Thread_local / __declspec(thread)) because TLS support is
 * inconsistent across the bare-metal NXP toolchains this project
 * targets (older ARMCC, some IAR/PowerPC EABI configs). Instead every
 * API call takes an explicit small integer thread id in
 * [0, LF_MAX_THREADS), which the caller picks once per worker
 * (thread, RTOS task, or CPU core) and reuses for every call -- this
 * works identically on pthreads, an RTOS, or bare-metal multicore.
 * ===================================================================== */
#ifndef LF_HAZARD_PTR_H
#define LF_HAZARD_PTR_H

#include <stddef.h>
#include "atomic_hashdefs.h"

#ifndef LF_MAX_THREADS
#define LF_MAX_THREADS 32
#endif

/* Two hazard slots per thread is sufficient for the Harris-Michael
 * list's search() (needs to protect 'cur' and 'next' simultaneously). */
#define LF_HP_SLOTS_PER_THREAD 2

typedef void (*lf_reclaim_fn)(void *node);

/* Must be called once before any other hp_* call, with the function
 * that actually frees a node (e.g. free(), or a slab-pool release). */
void hp_init(lf_reclaim_fn reclaim);

/* Publish that thread `tid` currently holds a live reference to `ptr`
 * in hazard slot `slot` (0 or 1). Pass NULL to clear the slot. */
void hp_set(int tid, int slot, void *ptr);

/* Clear both of this thread's hazard slots (call after leaving a
 * critical traversal, e.g. at the end of insert/delete/find). */
void hp_clear_all(int tid);

/* Retire a node this thread just physically unlinked via CAS. It is
 * queued and freed later, once no thread's hazard slot references it. */
void hp_retire(int tid, void *ptr);

/* Force a scan/reclaim pass for thread `tid`'s retire list right now
 * (hp_retire() already does this periodically; exposed for tests /
 * for explicit "drain before shutdown" use). */
void hp_flush(int tid);

/* Optional performance hint: tell the reclamation scan (is_hazardous(),
 * called from scan_and_reclaim() on every hp_retire()/hp_flush()) that
 * only tids [0, n) are ever actually used, so it doesn't have to walk
 * all LF_MAX_THREADS rows -- most of which sit at their initial 0/unused
 * value for the whole run in a typical app that uses far fewer threads
 * than LF_MAX_THREADS's default of 32. Never required for correctness
 * (the default, unless this is called, is the original LF_MAX_THREADS
 * scan) -- purely narrows a hot loop. Call this ONCE, with the true
 * count of tids your app will use, before creating any thread that
 * might call an hp_ or lf_list_ function -- there is no synchronization
 * between this call and the scan it affects beyond ordinary thread-
 * creation happens-before, so calling it after threads are already
 * running is a race. n is clamped to [1, LF_MAX_THREADS]. */
void hp_set_active_threads(int n);

#endif /* LF_HAZARD_PTR_H */
