/* =====================================================================
 * test_lockfree_list.c
 *
 * 1) A single-threaded correctness self-test (insert/find/delete
 *    behave like an ordinary sorted set).
 * 2) A multi-threaded stress test: N worker threads each own a
 *    private key range (so final membership is deterministic and
 *    checkable) while ALSO hammering one small SHARED key range
 *    concurrently (so insert/delete/find genuinely race on the same
 *    nodes -- this is what actually exercises the hazard-pointer /
 *    mark-bit reclamation logic, not just the API surface).
 *
 * Build for the native host (pthreads) with the provided Makefile:
 *   make test && ./test_lockfree_list
 * A bare-metal NXP target has no pthreads; port the worker body onto
 * your RTOS tasks / cores instead -- see README.md.
 * ===================================================================== */
#if defined(__linux__)
/* Must be defined before any system header is pulled in: exposes the
 * glibc extension pthread_attr_setaffinity_np() used below for the
 * optional --core-pinning command-line feature. */
#define _GNU_SOURCE
#endif
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <assert.h>
#include <stdint.h>
#include <unistd.h>
#include <pthread.h>
#include "lockfree_list.h"
#include "hazard_ptr.h" /* for hp_set_active_threads() -- see main() below */

/* Optional core pinning (see parse_core_args() / pin_attr_for() below):
 * a glibc/Linux-only pthread extension, so it's compiled in only where
 * it actually exists. On any other host (macOS, other libcs) the
 * command-line core arguments are still accepted and validated, but
 * threads run unpinned with a one-time warning -- this is a test-
 * harness convenience only and has no bearing on the list/hazard-
 * pointer code itself, which doesn't know or care which core it runs
 * on. There is no equivalent concept to wire up on the bare-metal NXP
 * targets: there, "which core" is exactly the RTOS task <-> core
 * assignment the integrator already makes when picking each task's
 * fixed `tid` (see hazard_ptr.h) -- see README.md. */
#if defined(__linux__) && defined(__GLIBC__)
#define LF_HAVE_AFFINITY 1
#include <sched.h>
#endif

#define NUM_THREADS        8
#define PER_THREAD_KEYS    2000
#define SHARED_KEY_RANGE   64     /* deliberately small: forces heavy contention */
#define SHARED_OPS_PER_THREAD 20000

static lf_list_t g_list;
static int g_fail_count = 0;

/* Core numbers given on the command line, one per thread; reused
 * round-robin if fewer were given than NUM_THREADS. Empty (g_num_core_ids
 * == 0) means "don't pin" -- the default, unchanged behavior. */
static int g_core_ids[NUM_THREADS];
static int g_num_core_ids = 0;

static void print_usage(const char *prog) {
    fprintf(stderr,
        "Usage: %s [core0 core1 ...]\n"
        "\n"
        "  Optional CPU core numbers to pin each of the %d worker threads\n"
        "  to, one argument per thread (thread i -> the i'th core given).\n"
        "  If fewer core numbers are given than threads, the list is\n"
        "  reused round-robin, e.g. on a 2-core box:\n"
        "      %s 0 1\n"
        "  pins threads 0,2,4,6 to core 0 and threads 1,3,5,7 to core 1.\n"
        "  With no arguments, threads are left unpinned (default OS\n"
        "  scheduling) -- the original behavior.\n"
        "\n"
        "  Core pinning is a Linux/glibc-only feature (pthread_attr_\n"
        "  setaffinity_np); on other platforms the arguments are still\n"
        "  parsed and range-checked but have no effect, and a warning is\n"
        "  printed once.\n",
        prog, NUM_THREADS, prog);
}

/* Parses argv[1..argc-1] as non-negative core numbers into g_core_ids.
 * Returns 0 on success, -1 on a bad argument (usage already printed). */
static int parse_core_args(int argc, char **argv) {
    long nproc = -1;
    int i;
#if defined(_SC_NPROCESSORS_ONLN)
    nproc = sysconf(_SC_NPROCESSORS_ONLN); /* -1 if the query isn't supported; just skip the check */
#endif
    if (argc - 1 > NUM_THREADS) {
        fprintf(stderr, "%s: got %d core numbers but there are only %d threads "
                        "(extra ones would never be used)\n", argv[0], argc - 1, NUM_THREADS);
        return -1;
    }
    for (i = 1; i < argc; i++) {
        char *end = NULL;
        long v = strtol(argv[i], &end, 10);
        if (end == argv[i] || *end != '\0' || v < 0) {
            fprintf(stderr, "%s: '%s' is not a valid core number (expected a "
                            "non-negative integer)\n", argv[0], argv[i]);
            return -1;
        }
        if (nproc > 0 && v >= nproc) {
            fprintf(stderr, "%s: core %ld is out of range -- this machine reports "
                            "%ld core(s) online (0..%ld)\n", argv[0], v, nproc, nproc - 1);
            return -1;
        }
        g_core_ids[g_num_core_ids++] = (int)v;
    }
    return 0;
}

#ifdef LF_HAVE_AFFINITY
/* Builds a pthread attr that pins to core g_core_ids[thread_idx %
 * g_num_core_ids]; caller must pthread_attr_destroy() it. Returns 1 on
 * success (attr filled in and safe to pass to pthread_create), 0 if
 * pinning isn't in effect (no core args given) -- caller should then
 * pass NULL to pthread_create instead of the untouched attr. */
static int pin_attr_for(int thread_idx, pthread_attr_t *attr, int *out_core) {
    cpu_set_t cpuset;
    int core;
    if (g_num_core_ids == 0) {
        return 0;
    }
    core = g_core_ids[thread_idx % g_num_core_ids];
    pthread_attr_init(attr);
    CPU_ZERO(&cpuset);
    CPU_SET(core, &cpuset);
    if (pthread_attr_setaffinity_np(attr, sizeof(cpuset), &cpuset) != 0) {
        fprintf(stderr, "[thread %d] pthread_attr_setaffinity_np(core %d) failed; "
                        "leaving unpinned\n", thread_idx, core);
        pthread_attr_destroy(attr);
        return 0;
    }
    *out_core = core;
    return 1;
}
#endif

/* ---------------------------------------------------------------- */
static void single_threaded_self_test(void) {
    lf_list_t list;
    void *val;
    int i;

    printf("[self-test] single-threaded correctness ... ");
    fflush(stdout);

    lf_list_init(&list);

    for (i = 0; i < 100; i++) {
        assert(lf_list_insert(&list, i, (void *)(intptr_t)(i * 10), 0) == 1);
    }
    /* duplicate insert must fail */
    assert(lf_list_insert(&list, 42, (void *)(intptr_t)9999, 0) == 0);

    for (i = 0; i < 100; i++) {
        assert(lf_list_find(&list, i, &val, 0) == 1);
        assert((intptr_t)val == i * 10);
    }
    assert(lf_list_find(&list, 12345, &val, 0) == 0);

    for (i = 0; i < 100; i += 2) {
        assert(lf_list_delete(&list, i, 0) == 1);
    }
    /* deleting again must fail */
    assert(lf_list_delete(&list, 0, 0) == 0);

    for (i = 0; i < 100; i++) {
        int found = lf_list_find(&list, i, &val, 0);
        if (i % 2 == 0) {
            assert(found == 0);
        } else {
            assert(found == 1);
        }
    }

    lf_list_destroy(&list);
    printf("OK\n");
}

/* ---------------------------------------------------------------- */
typedef struct {
    int tid;
} worker_arg_t;

static unsigned int xorshift32(unsigned int *s) {
    unsigned int x = *s;
    x ^= x << 13; x ^= x >> 17; x ^= x << 5;
    *s = x;
    return x;
}

static void *worker(void *arg_) {
    worker_arg_t *arg = (worker_arg_t *)arg_;
    int tid = arg->tid;
    unsigned int seed = (unsigned int)(tid * 2654435761u + 1);
    int i;

    /* Phase 1: insert this thread's private key range, verify each
     * is immediately visible. Private keys never collide with any
     * other thread, so every one of these assertions is deterministic
     * even under full concurrency. */
    long base = (long)tid * PER_THREAD_KEYS * 10;
    for (i = 0; i < PER_THREAD_KEYS; i++) {
        long key = base + i;
        if (!lf_list_insert(&g_list, key, (void *)(intptr_t)key, tid)) {
            fprintf(stderr, "[thread %d] unexpected duplicate on private key %ld\n", tid, key);
            __atomic_fetch_add(&g_fail_count, 1, __ATOMIC_RELAXED);
        }
    }
    for (i = 0; i < PER_THREAD_KEYS; i++) {
        long key = base + i;
        void *val = NULL;
        if (!lf_list_find(&g_list, key, &val, tid) || (intptr_t)val != key) {
            fprintf(stderr, "[thread %d] private key %ld missing/wrong after insert\n", tid, key);
            __atomic_fetch_add(&g_fail_count, 1, __ATOMIC_RELAXED);
        }
    }

    /* Phase 2: hammer a small SHARED key range with random
     * insert/delete/find from every thread at once. Outcomes on any
     * single call are not individually predictable (another thread
     * may win the race), but the operations themselves must never
     * crash, deadlock, or corrupt the structure -- checked globally
     * after all threads join. */
    for (i = 0; i < SHARED_OPS_PER_THREAD; i++) {
        /* Negative key space: guaranteed disjoint from every thread's
         * non-negative private range (tid * PER_THREAD_KEYS * 10 ..),
         * so a "shared" collision can only ever be with another
         * thread's shared-range op, never with someone's private key. */
        long key = -1 - (long)(xorshift32(&seed) % SHARED_KEY_RANGE);
        unsigned int op = xorshift32(&seed) % 3;
        if (op == 0) {
            lf_list_insert(&g_list, key, (void *)(intptr_t)key, tid);
        } else if (op == 1) {
            lf_list_delete(&g_list, key, tid);
        } else {
            void *val;
            lf_list_find(&g_list, key, &val, tid);
        }
    }

    /* Phase 3: delete this thread's own private keys back out, then
     * confirm they are gone -- deterministic again. */
    for (i = 0; i < PER_THREAD_KEYS; i++) {
        long key = base + i;
        if (!lf_list_delete(&g_list, key, tid)) {
            fprintf(stderr, "[thread %d] failed to delete own private key %ld\n", tid, key);
            __atomic_fetch_add(&g_fail_count, 1, __ATOMIC_RELAXED);
        }
    }
    for (i = 0; i < PER_THREAD_KEYS; i++) {
        long key = base + i;
        void *val;
        if (lf_list_find(&g_list, key, &val, tid)) {
            fprintf(stderr, "[thread %d] private key %ld still present after delete\n", tid, key);
            __atomic_fetch_add(&g_fail_count, 1, __ATOMIC_RELAXED);
        }
    }

    return NULL;
}

static long g_dump_last_key;
static int  g_dump_first;
static int  g_dump_sorted_ok;
static long g_dump_count;

static void dump_check_visitor(long key, void *value) {
    (void)value;
    g_dump_count++;
    if (!g_dump_first) {
        if (key <= g_dump_last_key) {
            g_dump_sorted_ok = 0;
        }
    }
    g_dump_first = 0;
    g_dump_last_key = key;
}

#include <time.h>
#include <stdint.h>

/* Measure how many rdtsc ticks occur in a known wall-clock interval,
 * using CLOCK_MONOTONIC as the reference clock. Call once at process
 * startup (after pinning to a core, ideally -- see caveats below). */
static double calibrate_ns_per_cycle(void) {
    struct timespec t0, t1;
    unsigned long long c0, c1;
    double wall_ns;

    /* Warm up: get both clocks off any cold-cache/branch-predictor
     * penalty before the real measurement starts. */
    (void)rdtsc();
    clock_gettime(CLOCK_MONOTONIC, &t0);

    clock_gettime(CLOCK_MONOTONIC, &t0);
    c0 = rdtsc();

    /* Busy-spin rather than sleep()/nanosleep(): a sleep can wake up
     * late by tens of microseconds of scheduler jitter, which is a
     * huge relative error over a short window. A tight spin for
     * ~200ms keeps both clocks "hot" and the window accurate. */
    do {
        clock_gettime(CLOCK_MONOTONIC, &t1);
    } while ((t1.tv_sec - t0.tv_sec) * 1000000000LL +
             (t1.tv_nsec - t0.tv_nsec) < 200000000LL /* 200ms */);
    c1 = rdtsc();

    wall_ns = (double)((t1.tv_sec - t0.tv_sec) * 1000000000LL +
                        (t1.tv_nsec - t0.tv_nsec));
    return wall_ns / (double)(c1 - c0);
}

static double g_ns_per_cycle;

static inline double cycles_to_ns(unsigned long long cycles) {
    return (double)cycles * g_ns_per_cycle;
}


int main(int argc, char **argv) {
    pthread_t threads[NUM_THREADS];
    worker_arg_t args[NUM_THREADS];
    int i;

    if (argc > 1 && (strcmp(argv[1], "-h") == 0 || strcmp(argv[1], "--help") == 0)) {
        print_usage(argv[0]);
        return 0;
    }
    if (parse_core_args(argc, argv) != 0) {
        print_usage(argv[0]);
        return 1;
    }
#ifndef LF_HAVE_AFFINITY
    if (g_num_core_ids > 0) {
        fprintf(stderr, "note: core pinning isn't supported on this platform "
                        "(needs Linux/glibc); running unpinned.\n");
    }
#endif
    /* at startup: */
    g_ns_per_cycle = calibrate_ns_per_cycle();

    single_threaded_self_test();

    printf("[stress] %d threads, %d private keys/thread, %d shared ops/thread over a "
           "%d-key shared range ...\n",
           NUM_THREADS, PER_THREAD_KEYS, SHARED_OPS_PER_THREAD, SHARED_KEY_RANGE);

    lf_list_init(&g_list);
    /* This harness only ever uses tids [0, NUM_THREADS) -- tell the
     * reclamation scan so it doesn't walk LF_MAX_THREADS (32 by
     * default) rows on every hp_retire()/hp_flush() when only
     * NUM_THREADS (8) of them are ever non-zero. Must be called before
     * any worker thread starts -- see hazard_ptr.h. */
    hp_set_active_threads(NUM_THREADS);

    for (i = 0; i < NUM_THREADS; i++) {
        args[i].tid = i;
#ifdef LF_HAVE_AFFINITY
        {
            pthread_attr_t attr;
            int core = -1;
            int pinned = pin_attr_for(i, &attr, &core);
            int rc = pthread_create(&threads[i], pinned ? &attr : NULL, worker, &args[i]);
            if (pinned) {
                pthread_attr_destroy(&attr);
            }
            if (rc != 0) {
                fprintf(stderr, "pthread_create failed\n");
                return 1;
            }
            if (pinned) {
                printf("[thread %d] pinned to core %d\n", i, core);
            }
        }
#else
        if (pthread_create(&threads[i], NULL, worker, &args[i]) != 0) {
            fprintf(stderr, "pthread_create failed\n");
            return 1;
        }
#endif
    }
    for (i = 0; i < NUM_THREADS; i++) {
        pthread_join(threads[i], NULL);
    }

    /* Global structural invariant check: what remains must be exactly
     * the residue of the shared-range races (private keys were all
     * deleted by their owning thread), strictly sorted, no duplicate
     * keys (lf_list_dump only visits live/unmarked nodes). */
    g_dump_first = 1;
    g_dump_sorted_ok = 1;
    g_dump_count = 0;
    lf_list_dump(&g_list, dump_check_visitor);

    printf("[stress] remaining nodes after run: %ld (must be <= %d, the shared range)\n",
           g_dump_count, SHARED_KEY_RANGE);
    printf("[stress] list order strictly ascending: %s\n", g_dump_sorted_ok ? "yes" : "NO -- BUG");

    if (!g_dump_sorted_ok || g_dump_count > SHARED_KEY_RANGE || g_fail_count != 0) {
        printf("RESULT: FAIL (%d assertion failures logged above)\n", g_fail_count);
        lf_list_destroy(&g_list);
        return 1;
    }

    lf_list_destroy(&g_list);
    printf("RESULT: PASS\n");

    printf("Consumer Cycles => %llu, %lf\n", cons_cycles, cycles_to_ns(cons_cycles));
    printf("Producer Cycles => %llu, %lf\n", prod_cycles, cycles_to_ns(prod_cycles));
    printf("Consumer Count => %llu\n", cons_count);
    printf("Producer Count => %llu\n", prod_count);
    if (cons_count > 0) {
        printf("Consumer avg latency (lf_list_delete) => %.1f ns/op\n",
               cycles_to_ns(cons_cycles) / (double)cons_count);
    }
    if (prod_count > 0) {
        printf("Producer avg latency (lf_list_insert) => %.1f ns/op\n",
               cycles_to_ns(prod_cycles) / (double)prod_count);
    }

    return 0;
}
