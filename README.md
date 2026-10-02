# Lock-free linked list (x86 / ARM / NXP)

A lock-free, sorted, singly linked set implementation portable across
x86, ARM (32/64-bit), and NXP silicon, driven entirely by a
preprocessor "hashdef" platform-abstraction header. It also includes a
second, independent data structure -- a lock-free MPMC queue with no
key or search, for callers who don't need sorted/associative storage
and want the fastest possible producer/consumer hand-off; see "Lock-
free MPMC queue" below.

## Files

| File                    | Purpose                                                            |
|--------------------------|--------------------------------------------------------------------|
| `atomic_hashdefs.h`      | `#ifdef`/`#define` platform layer: CAS, memory barriers, CPU-relax |
| `hazard_ptr.h/.c`        | Hazard-pointer safe memory reclamation (no TLS dependency), shared by both structures below |
| `lockfree_list.h/.c`     | The lock-free sorted linked list itself (Harris/Michael algorithm) |
| `test_lockfree_list.c`   | Single-threaded self-test + multi-writer stress test (pthreads)    |
| `test_producer_consumer.c` | Single-producer/single-consumer handoff test (pthreads)          |
| `lockfree_queue.h/.c`    | Lock-free MPMC FIFO queue, no key: `insert`/`fetch_first` (Michael & Scott algorithm) |
| `test_lockfree_queue.c`  | Single-threaded self-test + multi-producer/multi-consumer stress test (pthreads) |
| `Makefile`                | Native + cross build targets                                       |

## Why "NXP" means dispatching on ARM *or* PowerPC

NXP is a silicon vendor, not an instruction set. Its parts are built on
one of two architectures relevant to lock-free code:

* **ARM** Cortex-M/A cores -- Kinetis, LPC, i.MX, i.MX RT, S32K, ...
* **Power Architecture** e200/e500 cores -- MPC55xx/56xx/57xx, QorIQ

So `atomic_hashdefs.h` doesn't have a literal `#ifdef NXP` branch with
its own instructions -- there's no such thing at the ISA level. Instead
it detects the *actual* architecture (`LF_ARCH_ARM32`/`LF_ARCH_ARM64`/
`LF_ARCH_PPC`) and, on the GCC-based branch that all of NXP's supported
toolchains share (MCUXpresso's `arm-none-eabi-gcc`, S32 Design Studio's
gcc for both its ARM and PowerPC targets), the same `__atomic` builtins
correctly lower to each core's native atomic instructions. A
`-DLF_TARGET_NXP=1` build flag is provided purely for your project's
own build logs/asserts; it does not change codegen.

## x86 CAS vs. ARM/PowerPC LL/SC

This is the key hardware difference the whole abstraction layer exists
to paper over:

* **x86**: `CMPXCHG` (with a `LOCK` prefix for multi-core visibility)
  is a single hardware instruction that atomically compares and
  conditionally stores. `lf_cas_uptr()` compiles straight to one
  `lock cmpxchg`.
* **ARM / Power Architecture**: neither ISA has a CAS instruction.
  Both provide **Load-Linked/Store-Conditional (LL/SC)** instead:
  `LDREX`/`STREX` (or `LDXR`/`STXR`, or `CASAL` on ARMv8.1+) on ARM,
  and `lwarx`/`stwcx.` on Power Architecture (the e200/e500 cores NXP's
  MPC/QorIQ parts use). `LDREX`/`lwarx` place an exclusive-access
  "reservation" on the cache line; the paired `STREX`/`stwcx.` commits
  only if nothing has touched that line since, otherwise it reports
  failure. A CAS on these architectures is therefore *built* out of a
  small retry loop: load, compare in software, attempt the conditional
  store, and if that failed for *any* reason (including a completely
  unrelated interrupt landing on the core), loop back to the load.

`atomic_hashdefs.h` hides this either by delegating to the compiler's
`__atomic_compare_exchange_n` (GCC/Clang emit exactly that LL/SC retry
loop under the hood on ARM/PPC, and a single `lock cmpxchg` on x86), or
by writing the `LDREX`/`STREX` loop out explicitly for older
non-C11-atomics toolchains (legacy `armcc`, IAR). The practical
consequence for anyone extending this code: **every caller of
`lf_cas_uptr()` must already be structured as a retry loop**, since a
spurious/benign failure on ARM/PPC is expected behavior, not an error
condition. Every call site in `lockfree_list.c` follows this rule.

## Algorithm

Harris (2001) / Michael (2002) lock-free sorted linked list:

* Each node's `next` pointer doubles as a deletion flag: bit 0 (the
  "mark bit") is stolen from the pointer (nodes are naturally
  pointer-aligned, so this bit is otherwise always zero) to mean
  "this node is logically deleted."
* **Insert**: find the (`pred`, `curr`) window via `search()`, then
  `CAS(pred->next, curr, new_node)`.
* **Delete**: find the node, first mark it deleted with
  `CAS(node->next, unmarked_next, marked_next)` (this is the true
  linearization point of the delete), then opportunistically physically
  unlink it with `CAS(pred->next, node, unmarked_next)`.
* **search()** *helps*: any marked node it walks past is physically
  unlinked on the spot, whichever thread's CAS wins, before the search
  continues. This is why delete's physical-unlink CAS is allowed to
  fail silently -- some other thread's search already did it.

This design deliberately uses only a **native-word single CAS** (never
a double-word/`cmpxchg16b`-class op), so the exact same C source
compiles correctly on 32-bit Cortex-M, 32-bit e200/e500 PowerPC, and
64-bit x86-64/AArch64 without a code path that only exists on the
64-bit targets.

### Why hazard pointers, not just the mark bit

The mark bit alone tells you a node is *logically* gone, but a
concurrent thread can still be mid-traversal holding a raw pointer to
that node's memory. If you `free()` on physical unlink, that thread's
next dereference is a use-after-free -- and because `malloc` tends to
hand back the same address quickly, this is also exactly the classic
ABA setup for a plain single-word CAS. `hazard_ptr.h`/`.c` implements
Michael's (2004) hazard-pointer scheme: before dereferencing any node
reached via a shared pointer, a thread publishes that pointer in its
own slot; a node is only actually `free()`'d once no thread's slot
references it. `lf_search()` in `lockfree_list.c` follows a strict
*publish-then-validate* discipline -- protect the candidate node, then
re-read the field it was derived from and confirm it hasn't changed --
before ever dereferencing it; see the comments there for the
happens-before argument for why that closes the race.

Unlike most hazard-pointer write-ups, this one does **not** use
compiler thread-local storage (`__thread`/`_Thread_local`/
`__declspec(thread)`), because TLS support is inconsistent across
older bare-metal ARM/PowerPC toolchains. Every `hp_*`/`lf_list_*` call
instead takes an explicit small integer `tid` (see `hazard_ptr.h`),
which you assign once per pthread / RTOS task / CPU core and reuse.

### Prior art

The per-architecture-branches-behind-one-API layering here mirrors
DPDK's EAL atomics split (`lib/eal/x86/include/rte_atomic.h` vs.
`lib/eal/arm/include/rte_atomic_32.h`/`rte_atomic_64.h` --
`rte_atomicN_cmpset()` wraps exactly the "one CAS instruction on x86,
an LDREX/STREX loop on ARM" split described above), and this file's
`hp_retire()`/scan mechanism plays the role DPDK's `rte_rcu_qsbr`
(`lib/rcu`) plays for `rte_ring`/`rte_hash`: both defer freeing a
lock-free-unlinked element until it's provably safe. If you're
integrating this into a DPDK application, `rte_ring`/`rte_stack_lf`
solve the bounded-capacity-queue and LIFO-stack cases with a
128-bit-CAS+generation-counter ABA scheme instead of hazard pointers --
worth using directly if your target is x86-64/AArch64-only DPDK and you
don't need the sorted/associative structure this file provides.

## Lock-free MPMC queue (`insert` / `fetch_first`, no key)

`lockfree_queue.h/.c` is a **separate, independent data structure**,
added alongside everything above (nothing in `lockfree_list.c`/
`hazard_ptr.c`/`atomic_hashdefs.h` was changed to add it). It exists
because of a direct finding from profiling the sorted list (see
"Performance notes" below): `lf_search()`'s O(n) walk from `head` is
*inherent* to keeping the list sorted, not fixable inefficiency. If
your workload doesn't actually need key-ordered storage -- just "hand
off a value from any producer to any consumer, oldest first" -- this
queue sidesteps the search entirely:

* `lf_queue_insert(q, value, tid)` -- enqueue at the tail. O(1): one
  CAS to link the new node, one (best-effort, "helped" if this thread
  stalls) CAS to swing the tail pointer.
* `lf_queue_fetch_first(q, &value, tid)` -- dequeue the oldest value.
  O(1): one CAS to advance the head pointer past the node it reads the
  value out of. Returns 0 if the queue is empty.

Neither operation's cost depends on how many items are currently
queued -- unlike `lf_list_insert()`/`lf_list_delete()`, whose cost
grows with the list's live-node count (see "Performance notes").

### Algorithm

Michael & Scott (1996), "Simple, Fast, and Practical Non-Blocking and
Blocking Concurrent Queue Algorithms" -- the standard lock-free MPMC
queue design, combined with hazard pointers for reclamation (Michael's
2004 hazard-pointer paper uses this exact queue as its running
example, so the two fit together directly). A permanent dummy node
always sits at `Head`; the real head-of-queue value lives in
`Head->next`, which is what lets both `insert` and `fetch_first`
proceed with a single CAS each and no special-casing of "queue has
exactly one element." Unlike the sorted list, **no mark bit is
needed**: removal is a single CAS that atomically both logically and
physically unlinks a node, so there's no separate logical-delete step
to race on. See the algorithm comment block at the top of
`lockfree_queue.c` for the full step-by-step reasoning, including the
publish-then-validate hazard-pointer discipline each operation follows
(same pattern as `lf_search()`, applied to `Head`/`Tail`/`next`
instead of `pred`/`curr`).

It reuses `atomic_hashdefs.h` (`lf_atomic_uptr_t`, `lf_cas_uptr`,
`lf_fence`, `lf_cpu_relax`, ...) and `hazard_ptr.h`/`.c` as-is --
neither file needed a single change, since the hazard-pointer module
was already fully generic (`void*` payloads, explicit `tid`, pluggable
reclaim function) rather than list-specific. The one thing to know if
you ever use `lf_list_t` and `lf_queue_t` in the *same* process: the
hazard-pointer subsystem in `hazard_ptr.c` is a single global
instance, not one per structure, and both `lf_list_init()` and
`lf_queue_init()` call `hp_init()`. Each of this project's test
binaries links exactly one of the two structures, so this doesn't come
up in the provided tests; if you combine them, call `hp_init()`
yourself exactly once (e.g. by calling only one of `lf_list_init()`/
`lf_queue_init()`, and skipping the other's global setup) rather than
letting both run it. See the comment at the top of `lockfree_queue.h`
for the full detail.

### DPDK-inspired queue tuning

The queue keeps the unbounded Michael-Scott linked-node algorithm, but
uses a DPDK `rte_ring` performance lesson: producer and consumer state
should not share a cache line. `head` and `tail` now occupy separate
cache-line-sized slots, reducing cache-line bouncing when consumers
move the head while producers move the tail. Queue timing is opt-in
(`-DLF_QUEUE_ENABLE_STATS=1`); benchmark targets enable it, while normal
library builds skip clock reads and shared counter updates. Non-x86
targets build with stats disabled by default; to enable them, provide a
native `LFQ_READ_CYCLES()` implementation. `lf_queue_t` has a new layout,
so applications must rebuild against the updated header.

DPDK's `rte_ring` is a bounded array-based queue with bulk operations;
this queue remains unbounded and linked, so it does not inherit those
capacity or API semantics. DPDK's core libraries use BSD-3-Clause, not
public-domain terms. This change adopts the cache-line separation idea
without copying DPDK implementation code.

### Building and testing

```sh
make qtest        # self-test + 4-producer/4-consumer stress test (200,000 items)
make qtsan         # + ThreadSanitizer
make qasan         # + AddressSanitizer/UBSan
make qarm-linux    # queue + hazard-pointer cross-compile for 32-bit ARM Linux
make qaarch64-linux # queue + hazard-pointer cross-compile for AArch64 Linux
make qnxp-cortexm  # queue + hazard-pointer objects for Cortex-M4
make qnxp-ppc      # queue + hazard-pointer objects for NXP e200z4
make qtest QARGS="0 1 2 3 4 5 6 7"   # pin the 8 worker threads to cores 0-7
./test_lockfree_queue --help          # full usage
```

The stress test runs `NUM_PRODUCERS` (default 4) producer threads each
inserting `ITEMS_PER_PRODUCER` (default 50,000) uniquely-numbered
values concurrently with `NUM_CONSUMERS` (default 4) consumer threads
draining them, then checks -- via an atomically-marked "seen" table,
not just a count -- that every single value was delivered **exactly
once**: none lost, none duplicated. (FIFO order *within* one
producer's own values is guaranteed and is what the single-threaded
self-test checks; FIFO order *across* different producers racing
concurrently is not a guarantee this queue makes, any more than any
other MPMC queue makes it, so the stress test correctly does not
assert it.) All three thread counts are overridable at build time
without editing source, e.g. to get an SPSC-shaped run comparable to
`make pc-test`'s sorted-list numbers:

```sh
make qtest CFLAGS_EXTRA="-DNUM_PRODUCERS=1 -DNUM_CONSUMERS=1 -DITEMS_PER_PRODUCER=200000"
```

Passes clean under `qtsan`/`qasan` (zero sanitizer reports) and
`-Wall -Wextra -Wpedantic` (zero compiler warnings), same discipline
as the list -- see "Testing performed" below.

### Measured cycles/latency

Using the same `rdtsc`-based approach as the list (`lfq_rdtsc()`,
`calibrate_ns_per_cycle()` against `CLOCK_MONOTONIC`), accumulated with
`__atomic_fetch_add(..., __ATOMIC_RELAXED)` into `qins_cycles`/
`qins_count`/`qdeq_cycles`/`qdeq_count` (declared in
`lockfree_queue.h`, exactly parallel to the list's `prod_cycles`/
`cons_cycles`) -- run on the same host as every other number in this
README:

| Shape | `lf_queue_insert` | `lf_queue_fetch_first` |
|---|---:|---:|
| SPSC (1 producer, 1 consumer, 200,000 items) | ~151 ns/op | ~200 ns/op |
| MPMC (4 producers, 4 consumers, 200,000 items total) | ~500-800 ns/op | ~300-1100 ns/op |

Compare against the sorted list's SPSC numbers from `make pc-test`
(~250-260 ns/op insert, ~170-630 ns/op delete, depending on run): the
queue's insert is consistently faster, which is exactly what the
"insert is O(1), no search" design predicts, and its dequeue is in the
same ballpark as the list's delete under this workload (both are
dominated by fixed per-call overhead -- hazard-pointer publish/
validate/clear plus two `rdtsc` calls -- at these small, mostly-empty
sizes, not by algorithmic cost).

**Caveat worth knowing before trusting these numbers at face value:**
both `qdeq_count` and the list's `cons_count` include every call that
*returned "empty"/"not found," not only successful ones* -- the
consumer threads in both stress tests busy-poll (`lf_cpu_relax()`
between attempts) rather than blocking, so under contention a
meaningful fraction of the calls being averaged are fast "nothing
there" returns, not real dequeues/deletes. This mirrors the existing
convention in `lf_list_delete()` (its "not found" path is timed and
counted exactly the same way), so the two structures' numbers stay
comparable to each other, but it means "avg latency" here measures
"cost per `fetch_first`/`delete` call under this polling workload," not
purely "cost of a successful dequeue." If you need the latter split
out, add a second pair of counters gated on the function's return
value.

The MPMC row's wide range reflects real cross-core CAS contention on
`Q->Head`/`Q->Tail` from 4 threads racing on each end simultaneously --
expected and inherent to any lock-free MPMC structure, not specific to
this implementation; run-to-run variance under contention is normal
for lock-free code and is exactly why the list's own numbers above are
also given as ranges, not single figures.

## Building for each target

```sh
make test        # native x86_64 (or whatever host you're on): build + run
make tsan         # + ThreadSanitizer (recommended before trusting any change)
make asan         # + AddressSanitizer/UBSan
```

### Producer/consumer test

`test_producer_consumer.c` is a second, narrower test alongside the
N-writers stress test above: one thread (tid 0, "producer") inserts a
strictly increasing stream of keys; a second thread (tid 1,
"consumer") drains them in the same order, busy-waiting on
`lf_list_find()` for each key to appear, checking its value, then
deleting it. Where the main stress test throws many writers at one hot
key range to shake out reclamation races under heavy contention, this
test isolates the simplest possible concurrent shape -- exactly two
threads, one inserting and one deleting -- to check the basic
cross-thread handoff property directly: a value written on one
thread/core becomes visible, byte for byte, to a different thread/core
through nothing but `lf_list_insert()`/`lf_list_find()`, with no lock,
condvar, or dedicated queue structure in between. Because the sorted
set has no capacity limit, the producer never blocks on the consumer
(there's no backpressure, unlike a fixed-size ring buffer) -- only the
consumer ever waits, spinning (with `lf_cpu_relax()` and an occasional
`sched_yield()`) until its next expected key shows up.

```sh
make pc-test        # default: 200000 items, unpinned
make pc-tsan         # + ThreadSanitizer
make pc-asan         # + AddressSanitizer/UBSan
make pc-test PC_ARGS="50000 0 1"   # 50000 items, producer on core 0, consumer on core 1
./test_producer_consumer --help    # full usage
```

It checks: every key is observed by the consumer with exactly the
value the producer stored (the value is `key*7+3`, deliberately not
equal to the key, so a bug that accidentally handed back the key
itself as the value would be caught); every key the consumer finds is
then successfully deleted; and the list is empty once both threads
finish. Passes clean under `make pc-tsan`/`pc-asan` at both the
default size and 1,000,000 items, run repeatedly, including with both
threads deliberately pinned to the *same* core to maximize preemption
between them.

### Pinning worker threads to specific cores

By default both test harnesses leave their threads unpinned (plain
`pthread_create(..., NULL, ...)`) and let the OS scheduler place and
migrate them freely -- this is what all the testing above was run
with. To pin them instead, pass CPU core numbers on the command line;
run either binary with `-h`/`--help` for its exact argument order.
`test_lockfree_list` takes one core number per worker thread (fewer
than `NUM_THREADS` are reused round-robin):

```sh
./test_lockfree_list 0 1 2 3      # 8 threads pinned round-robin across cores 0-3
./test_lockfree_list 2            # all 8 threads pinned to core 2
make test ARGS="0 1"              # same, via the Makefile (ARGS is passed through)
```

`test_producer_consumer` instead takes an optional item count followed
by exactly two core numbers (producer, then consumer) -- see "Producer/
consumer test" above.

This is a Linux/glibc-only feature (`pthread_attr_setaffinity_np`,
gated on `#if defined(__linux__) && defined(__GLIBC__)`) in both test
files; on any other host the arguments are still parsed and
range-checked (bad or out-of-range core numbers are rejected either
way) but pinning has no effect, and the program says so once at
startup. It's a test-harness convenience for reproducing
scheduling-sensitive interleavings on purpose -- e.g. forcing several
threads onto one core to squeeze the real-time gap that the zombie-
`pred` bug (see "Testing performed" below) needed to manifest -- not
something the list or hazard-pointer code itself needs, uses, or is
even aware of; `lf_search()`/`hp_*()` never call `sched_getcpu()` or
look at which core they're on. There is also no equivalent to wire up
on the bare-metal NXP targets: there, "which core" is exactly the RTOS
task <-> core assignment the integrator already makes when picking
each task's fixed `tid` (see hazard_ptr.h).

**ARM Linux** (e.g. cross-compiling for a Linux-based NXP i.MX board):
```sh
sudo apt-get install gcc-arm-linux-gnueabihf   # or your distro's equivalent
make arm-linux        # 32-bit
make aarch64-linux     # 64-bit, gcc-aarch64-linux-gnu
```

**Bare-metal NXP Cortex-M** (MCUXpresso / S32 Design Studio, no OS):
```sh
make nxp-cortexm      # uses arm-none-eabi-gcc; set -mcpu for your part
```
There is no pthread on bare metal, so only `lockfree_list.c`/
`hazard_ptr.c` are compiled (not the test harness). Wire `hp_set`/
`lf_list_*`'s `tid` parameter to your RTOS task index or CPU core ID,
and drive `worker()`'s logic from `test_lockfree_list.c` as a template
for your own tasks.

**Bare-metal NXP QorIQ / MPC5xx** (e200/e500 Power Architecture):
```sh
make nxp-ppc          # uses powerpc-eabi-gcc (S32DS PowerPC toolchain); set -mcpu
```

## Notes on Cortex-M0/M0+

Cortex-M0/M0+ has no `LDREX`/`STREX` at all. If you need this list on
M0/M0+, the correct portable substitute is a short critical section
(`__disable_irq()`/`__enable_irq()`, or `PRIMASK` save/restore) around
the compare-and-swap in a new `atomic_hashdefs.h` branch guarded on
`__ARM_ARCH_6M__` -- this is *not* lock-free in the formal sense on
that core (a single interrupt-disabled region briefly serializes
cores/ISRs), but it's the standard practical accommodation for that
specific part family and is called out here rather than silently
assumed.

## Performance notes

If you instrument `lf_list_insert()`/`lf_list_delete()` (e.g. with
`rdtsc`) you'll find per-op latency ranges from a few hundred
nanoseconds up to tens of microseconds depending on the workload --
and the dominant factor is **how many live nodes the list holds**, not
constant-factor inefficiency in the insert/delete code. `lf_search()`
is a linear scan from `head`: an O(n) cost that is a property of
*sorted linked lists in general*, lock-free or not, not a bug specific
to this implementation. A controlled single-threaded benchmark
(insert N keys in random order, time each one) shows this directly:

| list size at time of insert | latency |
|---:|---:|
| 100 | ~280 ns |
| 1,000 | ~7 us |
| 5,000 | ~29 us |
| 10,000 | ~123 us |
| 20,000 | ~205 us |

This is why `test_lockfree_list.c`'s 8-thread stress test (which lets
up to ~16,000 private keys coexist in the list at once, on top of the
contended shared range) measures average insert latency in the tens
of microseconds, while `test_producer_consumer.c`'s SPSC handoff
(where the consumer keeps the live window small by design -- see its
file comment) measures a few hundred nanoseconds for the *same*
`lf_list_insert()`/`lf_list_delete()` code. If you're seeing latency
in this range, check how many nodes are actually live in your list
before assuming the implementation is slow -- and if your workload
genuinely needs thousands of concurrently-live entries with per-op
latency independent of size, the real fix is a different structure
(e.g. a lock-free skip list layered on this same mark-bit + hazard-
pointer scheme for O(log n), or a lock-free hash table for O(1) if
sorted order isn't actually required), not tuning this one.

That said, three real, safe constant-factor optimizations *were* found
and fixed in `hazard_ptr.c`/`atomic_hashdefs.h`, worth roughly 2x on an
uncontended single thread and 3-4x under real multi-thread contention
(measured on the 8-thread stress test above: ~155-193 us/insert before,
~48-53 us/insert after; ~2.0-4.2 us/delete before, ~0.7-1.1 us/delete
after -- run repeatedly, no overlap between before and after):

1. **A redundant fence in `hp_set()`.** `lf_store_uptr()` is already
   `memory_order_seq_cst`; on x86-64/GCC this was confirmed (by reading
   the generated assembly) to already compile to `xchg`, which is
   itself a full hardware barrier. The explicit `lf_fence()` that used
   to follow it compiled to a *second*, entirely redundant full
   barrier -- on a function called on every single hop of every
   `lf_search()`. Removed; see the comment on `hp_set()`.
2. **False sharing between threads' hazard-pointer rows.**
   `g_hp[LF_MAX_THREADS][LF_HP_SLOTS_PER_THREAD]` packed several
   threads' hazard slots into the same 64-byte cache line, so
   `hp_set()`/`hp_clear_all()` from any one thread invalidated a line
   every *other* thread on that line also needed -- real cross-core
   cache-line ping-pong on one of the hottest shared lines in the
   program under contention, not just an extra-instruction cost. Each
   thread's row (and its retired-node counter) is now padded and
   aligned to its own cache line via a new `LF_ALIGN(n)` macro in
   `atomic_hashdefs.h` (falls back to no-op, not an error, on a
   toolchain without a portable alignment spelling -- see the comment
   there).
3. **`is_hazardous()` always scanning all `LF_MAX_THREADS` (32) rows**,
   even when an app uses far fewer. `hp_set_active_threads(n)`
   (`hazard_ptr.h`) is a new, optional, backward-compatible call --
   default behavior (scan everything) is unchanged unless you call it
   -- that bounds the scan to the `n` tids you actually use; both test
   harnesses now call it with their real thread count. Call it once,
   before spawning any thread that might race on it.

None of these three change the algorithm or its correctness proof --
`make tsan`/`asan`/`pc-tsan`/`pc-asan` all still pass clean after them
(see "Testing performed" below), and they were verified against the
*original* code with `git diff`-style A/B testing before landing.

Separately: if you add your own timing instrumentation to
`lf_list_insert()`/`lf_list_delete()` the way this analysis did,
accumulate into shared counters with `__atomic_fetch_add()`
(`__ATOMIC_RELAXED` is fine -- they're pure statistics with no
ordering dependency), not `+=`. A plain `+=` on a counter written by
more than one thread (e.g. `test_lockfree_list.c`'s 8 workers, all
calling `lf_list_insert()`) is a lost-update data race -- ThreadSanitizer
catches it immediately, and until it's fixed the totals you're
dividing by to get "average latency" are themselves wrong.

## Porting to a new toolchain

Add a new `#elif` branch to the "uintptr_t-wide compare-and-swap"
section of `atomic_hashdefs.h` implementing `lf_cas_uptr`,
`lf_load_uptr`, `lf_store_uptr`, and the three fence macros; the rest
of the codebase (hazard pointers, the list, the queue) needs no
changes.

## Testing performed in this repository

`make test`, `make tsan` (ThreadSanitizer) and `make asan`
(AddressSanitizer + UBSan) all pass clean, with `-Wall -Wextra
-Wpedantic` and no warnings, on the included stress test (8 threads,
2000 private keys/thread, 20000 racing shared-key operations/thread)
-- see the Makefile. `make pc-tsan`/`pc-asan` (the single-producer/
single-consumer handoff test) also pass clean, at both the default
200,000-item size and 1,000,000 items, including a run with both
threads pinned to the same core to maximize preemption between them.
Each was additionally run several more times back
to back at that same full size to rule out a lucky pass, since this is
exactly the kind of code where a subtle bug compiles fine and only
shows up intermittently under real contention. Cross-compilation for
ARM/AArch64 Linux and for bare-metal NXP Cortex-M and e200/e500
PowerPC (`make arm-linux` / `aarch64-linux` / `nxp-cortexm` /
`nxp-ppc`) is exercised by command only in this environment, since no
cross toolchain is installed in this sandbox -- install the
appropriate toolchain (see "Building for each target" above) to
validate those.

Run `make tsan` and `make asan` before trusting any change to
`lf_search()`. During development this file's traversal logic had a
real bug that TSan and ASan both caught reliably: after `pred` is
promoted to a newly-validated live node, a *different* thread can
concurrently mark that same node (logically delete it) and physically
unlink it from its own predecessor. The node itself is not freed yet
(the retiring thread queues it for hazard-pointer-gated reclamation),
but its `next` field is now frozen forever -- nothing updates a
retired node's own successor pointer again. The original validation
only checked whether `pred->next` had changed since the last read; a
frozen field trivially "validates" against itself on every future
loop iteration, so the code kept walking forward from this zombie
`pred` indefinitely. That let a thread derive a `curr` candidate from
an arbitrarily stale snapshot -- one that, by the time the thread got
around to publishing a hazard pointer on it, could already have been
legitimately retired *and reclaimed* via the real (live) predecessor
chain elsewhere in the list, with no window in which this thread's
hazard pointer could have prevented that reclamation. That is a
genuine use-after-free, not a lost race, and no amount of stronger
memory ordering fixes it -- the fix is algorithmic: `lf_search()` now
checks whether a changed `pred->next` reading is marked (meaning
`pred` itself was concurrently deleted) and, if so, abandons `pred`
and restarts the whole search from `head`, exactly as on a failed CAS,
instead of continuing to trust it. See the comment above `lf_search()`
in `lockfree_list.c` for the full reasoning. (All atomics here are
still `memory_order_seq_cst` rather than acquire/release, which is a
separate, independently-motivated choice: the hazard-pointer
publish/validate protocol is an IRIW-shaped pattern that plain
acquire/release on two different locations does not fully order; see
the comment in `atomic_hashdefs.h`.)

`lf_list_destroy()` also flushes every thread's hazard-pointer retire
list (`hp_flush`) after freeing the reachable chain, since a node a
`delete()` call physically unlinked but that hadn't yet cleared a
hazard scan sits in a per-thread limbo list, not in the chain --
without this, `make asan`'s LeakSanitizer reported the discrepancy on
every run that exercised deletes.

### Queue testing

`make qtest`, `make qtsan`, and `make qasan` all pass clean (zero
sanitizer reports, zero `-Wall -Wextra -Wpedantic` warnings) on the
default 4-producer/4-consumer, 200,000-item stress test, run
repeatedly, and on the SPSC-shaped 1-producer/1-consumer variant (see
"Building and testing" above). Every run's "delivered N/N items, 0
lost, 0 duplicates" line is a real check, not a log message: the
consumer threads mark each dequeued value's slot in a global table
with `__atomic_exchange_n()` and flag a duplicate immediately if the
slot was already marked, and a final single-threaded pass after all
threads join confirms every one of the `NUM_PRODUCERS *
ITEMS_PER_PRODUCER` values was actually seen. `lf_queue_destroy()`
flushes every thread's hazard-pointer retire list for the same reason
`lf_list_destroy()` does (see above); `make qasan` confirmed no leak.
