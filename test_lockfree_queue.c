/* =====================================================================
 * test_lockfree_queue.c
 *
 * 1) A single-threaded correctness self-test (insert/fetch_first
 *    behave like an ordinary FIFO queue: strict insertion order,
 *    fetch_first on empty returns 0).
 * 2) A multi-producer/multi-consumer (MPMC) stress test: NUM_PRODUCERS
 *    threads concurrently insert() unique values while NUM_CONSUMERS
 *    threads concurrently fetch_first() them, verified afterwards for
 *    exactly-once delivery (no lost values, no duplicates) via a
 *    "seen" bitmap -- FIFO order across different producers is NOT
 *    checked (the queue makes no such promise when multiple threads
 *    are racing to enqueue; only *a single producer's own* items stay
 *    in relative order, which the single-threaded self-test above
 *    already covers), only that every value that went in comes out
 *    exactly once.
 *
 * Build for the native host (pthreads) with the provided Makefile:
 *   make qtest && ./test_lockfree_queue
 * A bare-metal NXP target has no pthreads; port the producer/consumer
 * worker bodies onto your RTOS tasks / cores instead, exactly as
 * described for the list in README.md.
 * ===================================================================== */
#if defined(__linux__)
#define _GNU_SOURCE
#endif
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <assert.h>
#include <stdint.h>
#include <unistd.h>
#include <time.h>
#include <pthread.h>
#include "lockfree_queue.h"
#include "hazard_ptr.h" /* for hp_set_active_threads() -- see main() below */

#if defined(__linux__) && defined(__GLIBC__)
#define LF_HAVE_AFFINITY 1
#include <sched.h>
#endif

/* Overridable at build time, e.g. `make qtest CFLAGS_EXTRA="-DNUM_PRODUCERS=1 -DNUM_CONSUMERS=1"`,
 * to compare an SPSC-shaped run against test_producer_consumer.c's
 * sorted-list SPSC numbers, or to scale contention up/down. */
#ifndef NUM_PRODUCERS
#define NUM_PRODUCERS       4
#endif
#ifndef NUM_CONSUMERS
#define NUM_CONSUMERS       4
#endif
#define NUM_THREADS         (NUM_PRODUCERS + NUM_CONSUMERS)
#ifndef ITEMS_PER_PRODUCER
#define ITEMS_PER_PRODUCER  50000
#endif
#define TOTAL_ITEMS         (NUM_PRODUCERS * ITEMS_PER_PRODUCER)

static lf_queue_t g_queue;
static int g_fail_count = 0;

/* Marks which of the [0, TOTAL_ITEMS) unique item ids have been
 * dequeued so far; written from every consumer thread via an atomic
 * exchange (see consumer_worker() below) so a duplicate delivery is
 * caught even under full concurrency, not just via a post-hoc count. */
static unsigned char g_seen[TOTAL_ITEMS];
static unsigned long long g_dequeued_total = 0;

/* Same core-pinning convenience as test_lockfree_list.c / test_
 * producer_consumer.c -- see the comment there for the full rationale
 * (Linux/glibc-only, purely a test-harness convenience, no bearing on
 * lockfree_queue.c itself). One core number per thread; thread i is
 * producer i for i < NUM_PRODUCERS, else consumer (i - NUM_PRODUCERS). */
static int g_core_ids[NUM_THREADS];
static int g_num_core_ids = 0;

static void print_usage(const char *prog) {
    fprintf(stderr,
        "Usage: %s [core0 core1 ...]\n"
        "\n"
        "  Optional CPU core numbers to pin each of the %d worker threads\n"
        "  to (%d producers followed by %d consumers), one argument per\n"
        "  thread, reused round-robin if fewer are given. With no\n"
        "  arguments, threads are left unpinned (default OS scheduling).\n"
        "  Linux/glibc-only feature; parsed but ignored elsewhere.\n",
        prog, NUM_THREADS, NUM_PRODUCERS, NUM_CONSUMERS);
}

static int parse_core_args(int argc, char **argv) {
    long nproc = -1;
    int i;
#if defined(_SC_NPROCESSORS_ONLN)
    nproc = sysconf(_SC_NPROCESSORS_ONLN);
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
    lf_queue_t q;
    void *val;
    int i;

    printf("[self-test] single-threaded correctness ... ");
    fflush(stdout);

    lf_queue_init(&q);

    /* Empty queue: fetch_first must report empty, not crash. */
    assert(lf_queue_fetch_first(&q, &val, 0) == 0);

    for (i = 0; i < 1000; i++) {
        assert(lf_queue_insert(&q, (void *)(intptr_t)i, 0) == 1);
    }
    /* Strict FIFO order for a single producer/consumer. */
    for (i = 0; i < 1000; i++) {
        assert(lf_queue_fetch_first(&q, &val, 0) == 1);
        assert((intptr_t)val == i);
    }
    /* Drained again: empty. */
    assert(lf_queue_fetch_first(&q, &val, 0) == 0);

    /* Interleaved insert/fetch_first: for a single producer/consumer
     * the queue is still strictly FIFO, so whatever comes out (here,
     * or when drained below) must be strictly ascending. */
    {
        intptr_t last = -1;
        for (i = 0; i < 500; i++) {
            assert(lf_queue_insert(&q, (void *)(intptr_t)i, 0) == 1);
            if (i % 3 == 0) {
                assert(lf_queue_fetch_first(&q, &val, 0) == 1);
                assert((intptr_t)val > last);
                last = (intptr_t)val;
            }
        }
        /* Drain the rest and confirm it continues strictly ascending. */
        while (lf_queue_fetch_first(&q, &val, 0)) {
            assert((intptr_t)val > last);
            last = (intptr_t)val;
        }
    }

    lf_queue_destroy(&q);
    printf("OK\n");
}

/* ---------------------------------------------------------------- */
typedef struct {
    int tid;
    int producer_idx; /* valid for producer threads only */
} worker_arg_t;

static void *producer_worker(void *arg_) {
    worker_arg_t *arg = (worker_arg_t *)arg_;
    int tid = arg->tid;
    long base = (long)arg->producer_idx * ITEMS_PER_PRODUCER;
    int i;

    for (i = 0; i < ITEMS_PER_PRODUCER; i++) {
        long id = base + i;
        if (!lf_queue_insert(&g_queue, (void *)(intptr_t)id, tid)) {
            fprintf(stderr, "[producer %d] insert failed (allocation failure) on id %ld\n",
                    arg->producer_idx, id);
            __atomic_fetch_add(&g_fail_count, 1, __ATOMIC_RELAXED);
        }
    }
    return NULL;
}

static void *consumer_worker(void *arg_) {
    worker_arg_t *arg = (worker_arg_t *)arg_;
    int tid = arg->tid;

    while (__atomic_load_n(&g_dequeued_total, __ATOMIC_RELAXED) < TOTAL_ITEMS) {
        void *val;
        if (lf_queue_fetch_first(&g_queue, &val, tid)) {
            intptr_t id = (intptr_t)val;
            unsigned char was_seen;
            if (id < 0 || id >= TOTAL_ITEMS) {
                fprintf(stderr, "[consumer] got out-of-range id %ld\n", (long)id);
                __atomic_fetch_add(&g_fail_count, 1, __ATOMIC_RELAXED);
                continue;
            }
            was_seen = __atomic_exchange_n(&g_seen[id], (unsigned char)1, __ATOMIC_RELAXED);
            if (was_seen) {
                fprintf(stderr, "[consumer] duplicate delivery of id %ld\n", (long)id);
                __atomic_fetch_add(&g_fail_count, 1, __ATOMIC_RELAXED);
            }
            __atomic_fetch_add(&g_dequeued_total, 1, __ATOMIC_RELAXED);
        } else {
            lf_cpu_relax();
        }
    }
    return NULL;
}

/* Measure how many rdtsc ticks occur in a known wall-clock interval --
 * identical approach to test_lockfree_list.c's calibrate_ns_per_cycle()
 * / cycles_to_ns(), duplicated here (rather than shared) so this file
 * builds standalone against only lockfree_queue.[ch] + hazard_ptr.[ch],
 * with no dependency on lockfree_list.[ch] at all. */
static double calibrate_ns_per_cycle(void) {
    struct timespec t0, t1;
    unsigned long long c0, c1;
    double wall_ns;

    (void)lfq_rdtsc();
    clock_gettime(CLOCK_MONOTONIC, &t0);

    clock_gettime(CLOCK_MONOTONIC, &t0);
    c0 = lfq_rdtsc();

    do {
        clock_gettime(CLOCK_MONOTONIC, &t1);
    } while ((t1.tv_sec - t0.tv_sec) * 1000000000LL +
             (t1.tv_nsec - t0.tv_nsec) < 200000000LL /* 200ms */);
    c1 = lfq_rdtsc();

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
    unsigned long long missing;

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
    g_ns_per_cycle = calibrate_ns_per_cycle();

    single_threaded_self_test();

    printf("[stress] %d producers x %d items, %d consumers, %d items total (MPMC) ...\n",
           NUM_PRODUCERS, ITEMS_PER_PRODUCER, NUM_CONSUMERS, TOTAL_ITEMS);

    lf_queue_init(&g_queue);
    /* This harness only ever uses tids [0, NUM_THREADS) -- see the
     * identical call and rationale in test_lockfree_list.c's main(). */
    hp_set_active_threads(NUM_THREADS);
    memset(g_seen, 0, sizeof(g_seen));

    for (i = 0; i < NUM_PRODUCERS; i++) {
        args[i].tid = i;
        args[i].producer_idx = i;
    }
    for (i = 0; i < NUM_CONSUMERS; i++) {
        args[NUM_PRODUCERS + i].tid = NUM_PRODUCERS + i;
        args[NUM_PRODUCERS + i].producer_idx = -1;
    }

    for (i = 0; i < NUM_THREADS; i++) {
        void *(*fn)(void *) = (i < NUM_PRODUCERS) ? producer_worker : consumer_worker;
#ifdef LF_HAVE_AFFINITY
        {
            pthread_attr_t attr;
            int core = -1;
            int pinned = pin_attr_for(i, &attr, &core);
            int rc = pthread_create(&threads[i], pinned ? &attr : NULL, fn, &args[i]);
            if (pinned) {
                pthread_attr_destroy(&attr);
            }
            if (rc != 0) {
                fprintf(stderr, "pthread_create failed\n");
                return 1;
            }
            if (pinned) {
                printf("[thread %d] (%s) pinned to core %d\n", i,
                       (i < NUM_PRODUCERS) ? "producer" : "consumer", core);
            }
        }
#else
        if (pthread_create(&threads[i], NULL, fn, &args[i]) != 0) {
            fprintf(stderr, "pthread_create failed\n");
            return 1;
        }
#endif
    }
    for (i = 0; i < NUM_THREADS; i++) {
        pthread_join(threads[i], NULL);
    }

    /* Global correctness check: every one of the TOTAL_ITEMS unique
     * ids produced must have been consumed exactly once. Duplicates
     * were already caught (and counted into g_fail_count) inline in
     * consumer_worker(); this pass additionally catches any id that
     * was silently lost (never delivered at all). */
    missing = 0;
    for (i = 0; i < TOTAL_ITEMS; i++) {
        if (!g_seen[i]) {
            missing++;
        }
    }
    if (missing > 0) {
        fprintf(stderr, "[stress] %llu of %d items were never delivered\n", missing, TOTAL_ITEMS);
        __atomic_fetch_add(&g_fail_count, 1, __ATOMIC_RELAXED);
    }
    printf("[stress] delivered %llu/%d items, 0 lost, 0 duplicates: %s\n",
           g_dequeued_total, TOTAL_ITEMS, (missing == 0 && g_fail_count == 0) ? "yes" : "NO -- BUG");

    if (g_fail_count != 0) {
        printf("RESULT: FAIL (%d assertion/consistency failures logged above)\n", g_fail_count);
        lf_queue_destroy(&g_queue);
        return 1;
    }

    lf_queue_destroy(&g_queue);
    printf("RESULT: PASS\n");

    printf("Enqueue (insert)     Cycles => %llu, %.1f ns total\n", qins_cycles, cycles_to_ns(qins_cycles));
    printf("Dequeue (fetch_first) Cycles => %llu, %.1f ns total\n", qdeq_cycles, cycles_to_ns(qdeq_cycles));
    printf("Enqueue Count => %llu\n", qins_count);
    printf("Dequeue Count => %llu\n", qdeq_count);
    if (qins_count > 0) {
        printf("Enqueue avg latency (lf_queue_insert)      => %.1f ns/op\n",
               cycles_to_ns(qins_cycles) / (double)qins_count);
    }
    if (qdeq_count > 0) {
        printf("Dequeue avg latency (lf_queue_fetch_first) => %.1f ns/op\n",
               cycles_to_ns(qdeq_cycles) / (double)qdeq_count);
    }

    return 0;
}
