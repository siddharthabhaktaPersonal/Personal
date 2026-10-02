/* =====================================================================
 * test_producer_consumer.c
 *
 * A single-producer / single-consumer (SPSC) test of the lock-free
 * sorted list: one thread (tid 0, "producer") inserts a strictly
 * increasing stream of keys; a second thread (tid 1, "consumer")
 * drains them in the same order, busy-waiting on lf_list_find() for
 * each key to appear, then deleting it and checking its value.
 *
 * This exercises a different shape of concurrency than
 * test_lockfree_list.c's N-writers-on-one-hot-range stress test:
 *   - Only ONE thread ever inserts and ONE thread ever deletes, so
 *     lf_list_insert()'s and lf_list_delete()'s own internal CAS-retry
 *     paths should rarely if ever actually race against each other on
 *     the *same* node -- but lf_search() (shared by both) still walks
 *     the same live nodes concurrently from both threads on every
 *     call, so the hazard-pointer publish/validate discipline is
 *     still very much in play: the consumer is forever dereferencing
 *     nodes the producer is still busy inserting after, and every
 *     delete the consumer performs frees a node that was live only
 *     microseconds earlier while the producer was walking past it.
 *   - It's a genuine handoff: a value produced on one core/thread must
 *     become visible, byte-for-byte, to a different core/thread with
 *     no lock, no dedicated queue structure, and no shared cache line
 *     the two threads spin on -- this is precisely the memory-
 *     visibility property the "why seq_cst" comment in
 *     atomic_hashdefs.h and README.md talks about, exercised in its
 *     simplest possible form (one writer, one reader).
 *   - Because the sorted set has no capacity limit, this is an
 *     unbounded, non-blocking handoff: the producer never waits on the
 *     consumer (there is no backpressure, unlike a fixed-size ring
 *     buffer) -- it can run arbitrarily far ahead. Only the consumer
 *     ever waits, spinning on lf_list_find() until its next expected
 *     key shows up.
 *
 * Correctness checks:
 *   - every key 0..N-1 is eventually observed by the consumer with
 *     exactly the value the producer stored for it (value = key*7+3,
 *     deliberately different from the key itself so a bug that
 *     accidentally hands back the key instead of the real value
 *     pointer would be caught);
 *   - every key the consumer looked up is then successfully deleted
 *     (a failed delete right after a successful find would mean two
 *     threads both grabbed it, which can't happen with a single
 *     consumer -- or that the list got corrupted);
 *   - the list is empty once both threads finish, since every key
 *     that was ever inserted was also drained.
 *
 * Build: make pc-test / pc-tsan / pc-asan (see Makefile), or by hand:
 *   cc -std=c11 -pthread -o test_producer_consumer \
 *       lockfree_list.c hazard_ptr.c test_producer_consumer.c
 *
 * Usage: ./test_producer_consumer [item_count] [producer_core consumer_core]
 *   item_count             defaults to 200000 if omitted.
 *   producer_core/consumer_core  optional CPU core numbers to pin each
 *                          thread to (Linux/glibc only -- see
 *                          test_lockfree_list.c's core-pinning support,
 *                          which this reuses); omit both for the
 *                          default, unpinned OS scheduling.
 * ===================================================================== */
#if defined(__linux__)
/* Must precede any system header -- see test_lockfree_list.c. */
#define _GNU_SOURCE
#endif
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <unistd.h>
#include <pthread.h>
#include <time.h>
#include "lockfree_list.h"
#include "atomic_hashdefs.h" /* only for lf_cpu_relax() in the consumer's spin */
#include "hazard_ptr.h" /* for hp_set_active_threads() -- see main() below */

#if defined(__linux__) && defined(__GLIBC__)
#define LF_HAVE_AFFINITY 1
#include <sched.h>
#endif

#define PRODUCER_TID 0
#define CONSUMER_TID 1

#define DEFAULT_ITEM_COUNT 200000L

/* value = key*7+3: deliberately not equal to key, so that a bug which
 * accidentally propagates the key itself as the value (instead of the
 * real value pointer stored by the producer) shows up as a mismatch
 * rather than silently "working". */
static inline void *encode_value(long key) {
    return (void *)(intptr_t)(key * 7 + 3);
}

static lf_list_t g_list;
static long g_item_count = DEFAULT_ITEM_COUNT;

typedef struct {
    long produced;
} producer_stats_t;

typedef struct {
    long consumed;
    long mismatches;
    long failed_deletes;
} consumer_stats_t;

static void *producer_thread(void *arg_) {
    producer_stats_t *st = (producer_stats_t *)arg_;
    long key;
    for (key = 0; key < g_item_count; key++) {
        /* A strictly increasing key stream from a single inserter is
         * the best case for this data structure: each new key is
         * greater than everything still live, so lf_search() only
         * ever has to walk past whatever the consumer hasn't drained
         * yet (typically a short window right behind the producer),
         * not the whole history of keys ever inserted. */
        if (!lf_list_insert(&g_list, key, encode_value(key), PRODUCER_TID)) {
            fprintf(stderr, "[producer] BUG: lf_list_insert rejected key %ld as a "
                            "duplicate -- it should be impossible for this key to "
                            "already be present (single producer, strictly "
                            "increasing keys)\n", key);
            abort();
        }
        st->produced++;
    }
    return NULL;
}

static void *consumer_thread(void *arg_) {
    consumer_stats_t *st = (consumer_stats_t *)arg_;
    long key;
    for (key = 0; key < g_item_count; key++) {
        void *val = NULL;
        unsigned spins = 0;

        /* Busy-wait for the producer to publish this key. This is the
         * actual cross-thread handoff: once lf_list_find() returns 1,
         * `val` must be exactly what the producer stored -- there is
         * no other synchronization between these two threads (no
         * mutex, no condvar, no memory fence written by hand in this
         * test) other than what lf_list_insert()/lf_list_find()
         * themselves provide. */
        while (!lf_list_find(&g_list, key, &val, CONSUMER_TID)) {
            spins++;
            if ((spins & 0xFFFu) == 0) {
                /* Long wait (producer is behind or descheduled): give
                 * the core back to the scheduler instead of pegging it
                 * at 100% spinning on a key that isn't coming for a
                 * while -- matters most on a small machine (e.g. this
                 * 2-core sandbox) where the producer needs a core too. */
                sched_yield();
            } else {
                lf_cpu_relax();
            }
        }

        if ((intptr_t)val != (intptr_t)encode_value(key)) {
            fprintf(stderr, "[consumer] BUG: key %ld carried value %ld, expected %ld\n",
                    key, (long)(intptr_t)val, (long)(intptr_t)encode_value(key));
            st->mismatches++;
        }

        /* Only this thread ever deletes, and each key is drained
         * exactly once, so this must always succeed. */
        if (!lf_list_delete(&g_list, key, CONSUMER_TID)) {
            fprintf(stderr, "[consumer] BUG: lf_list_delete failed for key %ld "
                            "immediately after a successful find -- something else "
                            "deleted it, which shouldn't be possible with a single "
                            "consumer\n", key);
            st->failed_deletes++;
        }
        st->consumed++;
    }
    return NULL;
}

static long g_final_count = 0;
static void count_visitor(long key, void *value) {
    (void)key; (void)value;
    g_final_count++;
}

#ifdef LF_HAVE_AFFINITY
static int pin_attr_for_core(int core, pthread_attr_t *attr) {
    cpu_set_t cpuset;
    pthread_attr_init(attr);
    CPU_ZERO(&cpuset);
    CPU_SET(core, &cpuset);
    if (pthread_attr_setaffinity_np(attr, sizeof(cpuset), &cpuset) != 0) {
        fprintf(stderr, "warning: pthread_attr_setaffinity_np(core %d) failed; "
                        "leaving that thread unpinned\n", core);
        pthread_attr_destroy(attr);
        return 0;
    }
    return 1;
}
#endif

/* Measure rdtsc ticks per nanosecond via CLOCK_MONOTONIC, same
 * approach as test_lockfree_list.c -- see the comment there. */
static double calibrate_ns_per_cycle(void) {
    struct timespec t0, t1;
    unsigned long long c0, c1;
    (void)rdtsc();
    clock_gettime(CLOCK_MONOTONIC, &t0);
    c0 = rdtsc();
    do {
        clock_gettime(CLOCK_MONOTONIC, &t1);
    } while ((t1.tv_sec - t0.tv_sec) * 1000000000LL +
             (t1.tv_nsec - t0.tv_nsec) < 200000000LL /* 200ms */);
    c1 = rdtsc();
    return (double)((t1.tv_sec - t0.tv_sec) * 1000000000LL +
                     (t1.tv_nsec - t0.tv_nsec)) / (double)(c1 - c0);
}
static double g_ns_per_cycle;
static inline double cycles_to_ns(unsigned long long cycles) {
    return (double)cycles * g_ns_per_cycle;
}

static void print_usage(const char *prog) {
    fprintf(stderr,
        "Usage: %s [item_count] [producer_core consumer_core]\n"
        "\n"
        "  item_count       number of keys to hand off producer -> consumer\n"
        "                   (default %ld).\n"
        "  producer_core,\n"
        "  consumer_core    optional CPU core numbers to pin each thread to\n"
        "                   (Linux/glibc only; give both or neither).\n",
        prog, DEFAULT_ITEM_COUNT);
}

int main(int argc, char **argv) {
    pthread_t prod_t, cons_t;
    producer_stats_t pstat = {0};
    consumer_stats_t cstat = {0};
    int have_cores = 0;
    int producer_core = -1, consumer_core = -1;

    if (argc > 1 && (strcmp(argv[1], "-h") == 0 || strcmp(argv[1], "--help") == 0)) {
        print_usage(argv[0]);
        return 0;
    }
    if (argc >= 2) {
        char *end = NULL;
        long v = strtol(argv[1], &end, 10);
        if (end == argv[1] || *end != '\0' || v <= 0) {
            fprintf(stderr, "%s: '%s' is not a valid item count\n", argv[0], argv[1]);
            print_usage(argv[0]);
            return 1;
        }
        g_item_count = v;
    }
    if (argc == 4) {
        char *end1 = NULL, *end2 = NULL;
        long c1 = strtol(argv[2], &end1, 10);
        long c2 = strtol(argv[3], &end2, 10);
        if (end1 == argv[2] || *end1 != '\0' || c1 < 0 ||
            end2 == argv[3] || *end2 != '\0' || c2 < 0) {
            fprintf(stderr, "%s: producer_core/consumer_core must be non-negative "
                            "integers\n", argv[0]);
            print_usage(argv[0]);
            return 1;
        }
        producer_core = (int)c1;
        consumer_core = (int)c2;
        have_cores = 1;
    } else if (argc == 3) {
        fprintf(stderr, "%s: give both producer_core and consumer_core, or neither\n", argv[0]);
        print_usage(argv[0]);
        return 1;
    } else if (argc > 4) {
        print_usage(argv[0]);
        return 1;
    }
#ifndef LF_HAVE_AFFINITY
    if (have_cores) {
        fprintf(stderr, "note: core pinning isn't supported on this platform "
                        "(needs Linux/glibc); running unpinned.\n");
        have_cores = 0;
    }
#endif

    printf("[pc-test] producer/consumer handoff: %ld items, "
           "producer=tid%d consumer=tid%d%s\n",
           g_item_count, PRODUCER_TID, CONSUMER_TID,
           have_cores ? " (pinned)" : " (unpinned)");

    g_ns_per_cycle = calibrate_ns_per_cycle();

    lf_list_init(&g_list);
    /* Only tids 0 (producer) and 1 (consumer) are ever used here --
     * see hazard_ptr.h. Must precede thread creation. */
    hp_set_active_threads(2);

#ifdef LF_HAVE_AFFINITY
    if (have_cores) {
        pthread_attr_t pattr, cattr;
        int p_pinned = pin_attr_for_core(producer_core, &pattr);
        int c_pinned = pin_attr_for_core(consumer_core, &cattr);
        if (pthread_create(&prod_t, p_pinned ? &pattr : NULL, producer_thread, &pstat) != 0 ||
            pthread_create(&cons_t, c_pinned ? &cattr : NULL, consumer_thread, &cstat) != 0) {
            fprintf(stderr, "pthread_create failed\n");
            return 1;
        }
        if (p_pinned) { printf("[producer] pinned to core %d\n", producer_core); pthread_attr_destroy(&pattr); }
        if (c_pinned) { printf("[consumer] pinned to core %d\n", consumer_core); pthread_attr_destroy(&cattr); }
    } else
#endif
    {
        if (pthread_create(&prod_t, NULL, producer_thread, &pstat) != 0 ||
            pthread_create(&cons_t, NULL, consumer_thread, &cstat) != 0) {
            fprintf(stderr, "pthread_create failed\n");
            return 1;
        }
    }

    pthread_join(prod_t, NULL);
    pthread_join(cons_t, NULL);

    g_final_count = 0;
    lf_list_dump(&g_list, count_visitor);

    printf("[pc-test] produced=%ld consumed=%ld mismatches=%ld failed_deletes=%ld "
           "remaining_in_list=%ld\n",
           pstat.produced, cstat.consumed, cstat.mismatches, cstat.failed_deletes,
           g_final_count);

    lf_list_destroy(&g_list);

    if (pstat.produced != g_item_count || cstat.consumed != g_item_count ||
        cstat.mismatches != 0 || cstat.failed_deletes != 0 || g_final_count != 0) {
        printf("RESULT: FAIL\n");
        return 1;
    }
    printf("RESULT: PASS\n");
    printf("Consumer Cycles => %llu, %.1f ns total\n", cons_cycles, cycles_to_ns(cons_cycles));
    printf("Producer Cycles => %llu, %.1f ns total\n", prod_cycles, cycles_to_ns(prod_cycles));
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
