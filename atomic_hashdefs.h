/* =====================================================================
 * atomic_hashdefs.h
 *
 * Platform abstraction layer for lock-free primitives, built entirely
 * from preprocessor "hashdefs" (#ifdef / #define chains).  Provides a
 * single portable API:
 *
 *   lf_cas_uptr(addr, expected, desired)   -> single-word CAS, returns 1/0
 *   lf_fence()                             -> full memory barrier
 *   lf_load_fence()                        -> acquire/load barrier
 *   lf_store_fence()                       -> release/store barrier
 *   lf_cpu_relax()                         -> spin-wait CPU hint
 *
 * NOTE ON "NXP": NXP is a silicon *vendor*, not an instruction set.
 * NXP parts implement one of two architectures relevant here:
 *   - ARM (Cortex-M/A: Kinetis, LPC, i.MX, S32K, i.MX RT, ...)
 *   - Power Architecture (e200/e500 core: MPC55xx/56xx/57xx, QorIQ)
 * so the correct, technically honest way to support "NXP" is to
 * dispatch on the *actual* architecture the NXP part is built on
 * (LF_ARCH_ARM / LF_ARCH_ARM64 / LF_ARCH_PPC below), which is exactly
 * what this header does. A convenience LF_TARGET_NXP tag is provided
 * for build logging only; it does not change codegen.
 *
 * Word size: this design intentionally uses only a NATIVE-WORD
 * (uintptr_t sized) single CAS -- not a double-word/DWCAS -- so it
 * runs unmodified on 32-bit Cortex-M, 32-bit e200/e500 PowerPC, and
 * 64-bit x86-64/AArch64 alike. ABA safety is achieved in the list
 * itself via Harris' mark-bit technique + hazard pointers, not via
 * a wide CAS, which is why a single-word primitive suffices here.
 *
 * WHY lf_cas_uptr() IS NOT "THE SAME INSTRUCTION" ON EVERY ARCH:
 *   - x86 (LF_ARCH_X86): the ISA provides CMPXCHG (LOCK-prefixed for
 *     multi-core) as one true hardware Read-Modify-Write instruction:
 *     the compare and the conditional store are a single atomic
 *     micro-op that either commits or doesn't. lf_cas_uptr() lowers
 *     directly to `lock cmpxchg` (32-bit) / `lock cmpxchg` on a
 *     64-bit register (x86-64) -- one instruction, no software loop.
 *   - ARM (LF_ARCH_ARM32 / LF_ARCH_ARM64) and Power Architecture
 *     (LF_ARCH_PPC, the e200/e500 core NXP's MPC/QorIQ parts use):
 *     neither ISA has a native CAS instruction at all. Both instead
 *     expose Load-Linked/Store-Conditional (LL/SC): LDREX/STREX on
 *     ARMv7-A/Cortex-M3+ (or LDXR/STXR, or CASAL on ARMv8.1+, for
 *     AArch64), and lwarx/stwcx. on Power Architecture. LDREX/lwarx
 *     tags a "reservation" on the cache line; STREX/stwcx. commits
 *     only if that reservation is still valid, else reports failure.
 *     A CAS therefore has to be *built* on top of LL/SC as a small
 *     retry loop ("load, compare in software, try the conditional
 *     store, retry from the load if it failed") -- see the GNU
 *     branch below, where __atomic_compare_exchange_n emits exactly
 *     that load/compare/STREX-or-retry sequence, and the ARMCC/IAR
 *     branches further down, where the LDREX/STREX loop is written
 *     out explicitly because those toolchains predate C11 atomics.
 *     Practical consequence: an ARM/PPC CAS can spuriously "fail"
 *     even when the compared value matches (an interrupt, a cache
 *     eviction, or another core's access can invalidate the
 *     reservation), so lf_cas_uptr() callers must always be written
 *     as retry loops -- every caller in lockfree_list.c already is,
 *     which is what makes the same C source correct on both families.
 *
 * PRIOR ART / STYLE: this per-architecture-branch-behind-one-API
 * layering mirrors DPDK's EAL atomics split (lib/eal/x86/include/
 * rte_atomic.h vs lib/eal/arm/include/rte_atomic_32.h /
 * rte_atomic_64.h -- rte_atomicN_cmpset() wraps the identical
 * "one CAS instruction on x86, an LDREX/STREX loop on ARM" split
 * described above), and the reclamation strategy in hazard_ptr.h
 * plays the same role as DPDK's rte_rcu_qsbr in lib/rcu: both defer
 * freeing a lock-free-unlinked node until it is provably no longer
 * being read by any other core.
 * ===================================================================== */
#ifndef LF_ATOMIC_HASHDEFS_H
#define LF_ATOMIC_HASHDEFS_H

#include <stdint.h>

/* --------------------------------------------------------------------
 * 1. Architecture detection hashdefs
 * ------------------------------------------------------------------ */
#if defined(__x86_64__) || defined(_M_X64) || defined(__i386__) || defined(_M_IX86)
#  define LF_ARCH_X86 1
#elif defined(__aarch64__) || defined(_M_ARM64)
#  define LF_ARCH_ARM64 1
#elif defined(__arm__) || defined(_M_ARM)
#  define LF_ARCH_ARM32 1
#elif defined(__powerpc__) || defined(__PPC__) || defined(__ppc__) || defined(__PPC64__)
#  define LF_ARCH_PPC 1
#else
#  define LF_ARCH_UNKNOWN 1
#endif

#if defined(LF_ARCH_ARM32) || defined(LF_ARCH_ARM64)
#  define LF_ARCH_ARM 1
#endif

/* Convenience "vendor" tag. Define -DLF_TARGET_NXP=1 in your build
 * (S32DS / MCUXpresso / CodeWarrior project settings) purely for
 * documentation/build-log purposes; it does not gate any code path
 * below because the ISA (ARM vs PPC) is what actually matters. */
#if !defined(LF_TARGET_NXP)
#  define LF_TARGET_NXP 0
#endif

/* --------------------------------------------------------------------
 * 2. Compiler/toolchain detection hashdefs
 * ------------------------------------------------------------------ */
#if defined(_MSC_VER) && !defined(__clang__)
#  define LF_TOOLCHAIN_MSVC 1
#elif defined(__IAR_SYSTEMS_ICC__)
#  define LF_TOOLCHAIN_IAR 1
#elif defined(__CC_ARM) || defined(__ARMCC_VERSION)
#  define LF_TOOLCHAIN_ARMCC 1   /* legacy ARM Compiler 5/6 (armcc/armclang -Wa) */
#elif defined(__GNUC__) || defined(__clang__)
#  define LF_TOOLCHAIN_GNU 1     /* gcc, clang, and every NXP GCC-based SDK:
                                    arm-none-eabi-gcc (MCUXpresso), S32DS gcc
                                    for both ARM and e200/e500 PowerPC */
#else
#  define LF_TOOLCHAIN_UNKNOWN 1
#endif

/* Prefer C11 <stdatomic.h> whenever it is genuinely available, since it
 * is the most portable and best-optimized path across GCC/Clang and
 * recent MSVC (/std:c11 or /experimental:c11atomics). */
#if !defined(LF_FORCE_NO_STDATOMIC) \
    && defined(__STDC_VERSION__) && (__STDC_VERSION__ >= 201112L) \
    && !defined(__STDC_NO_ATOMICS__)
#  define LF_HAVE_STDATOMIC 1
#endif

/* --------------------------------------------------------------------
 * 3. uintptr_t-wide compare-and-swap
 * ------------------------------------------------------------------ */
#if defined(LF_HAVE_STDATOMIC)

#include <stdatomic.h>
typedef _Atomic(uintptr_t) lf_atomic_uptr_t;

/* NOTE ON memory_order_seq_cst HERE (not acquire/release): the
 * hazard-pointer publish/validate protocol in hazard_ptr.c +
 * lf_search() is an IRIW-shaped pattern -- one thread does
 * {store hazard-slot; load shared-pointer}, another does
 * {store shared-pointer (via CAS); load hazard-slot} -- and plain
 * acquire/release on two DIFFERENT locations does not prevent a
 * StoreLoad reordering on either side (this is precisely what bit an
 * earlier, acquire/release-only version of this file: it passed under
 * a naive read of the code, compiled and ran fine on x86 in casual
 * testing, and then failed intermittently -- exactly the failure mode
 * this class of bug produces -- until ThreadSanitizer's stress run
 * caught the missing happens-before edge and this comment was added
 * with the fix). Only a full seq_cst fence -- or, equivalently and
 * more simply here, seq_cst on the operations themselves -- closes
 * that gap. See README.md "Why every atomic op here is seq_cst".
 */
static inline int lf_cas_uptr(lf_atomic_uptr_t *addr, uintptr_t expected, uintptr_t desired) {
    return atomic_compare_exchange_strong_explicit(
        addr, &expected, desired, memory_order_seq_cst, memory_order_seq_cst);
}
static inline uintptr_t lf_load_uptr(lf_atomic_uptr_t *addr) {
    return atomic_load_explicit(addr, memory_order_seq_cst);
}
static inline void lf_store_uptr(lf_atomic_uptr_t *addr, uintptr_t v) {
    atomic_store_explicit(addr, v, memory_order_seq_cst);
}
#define lf_fence()       atomic_thread_fence(memory_order_seq_cst)
#define lf_load_fence()  atomic_thread_fence(memory_order_acquire)
#define lf_store_fence() atomic_thread_fence(memory_order_release)

#elif defined(LF_TOOLCHAIN_MSVC)

#include <intrin.h>
typedef volatile uintptr_t lf_atomic_uptr_t;

static __inline int lf_cas_uptr(lf_atomic_uptr_t *addr, uintptr_t expected, uintptr_t desired) {
#if defined(_WIN64)
    return (uintptr_t)_InterlockedCompareExchange64(
        (volatile __int64 *)addr, (__int64)desired, (__int64)expected) == expected;
#else
    return (uintptr_t)_InterlockedCompareExchange(
        (volatile long *)addr, (long)desired, (long)expected) == expected;
#endif
}
static __inline uintptr_t lf_load_uptr(lf_atomic_uptr_t *addr) {
    uintptr_t v = *addr;
    _ReadBarrier();
    return v;
}
static __inline void lf_store_uptr(lf_atomic_uptr_t *addr, uintptr_t v) {
    _WriteBarrier();
    *addr = v;
    _WriteBarrier();
}
#define lf_fence()       MemoryBarrier()
#define lf_load_fence()  _ReadBarrier()
#define lf_store_fence() _WriteBarrier()

#elif defined(LF_TOOLCHAIN_GNU)

/* Covers x86 gcc/clang, ARM Linux gcc/clang, arm-none-eabi-gcc
 * (MCUXpresso), and NXP S32 Design Studio gcc for both the ARM and
 * e200/e500 PowerPC cores -- __atomic builtins lower to the correct
 * native instruction on every one of these: LOCK CMPXCHG (x86),
 * LDREX/STREX or CASAL (ARM32/AArch64), lwarx/stwcx. (PowerPC). */
typedef volatile uintptr_t lf_atomic_uptr_t;

/* seq_cst throughout, not acq_rel/acquire/release -- see the long
 * comment on the stdatomic branch above ("Why every atomic op here is
 * seq_cst"): the hazard-pointer publish/validate pattern is IRIW-
 * shaped and needs the full total-order guarantee, not just pairwise
 * release/acquire on a single location. */
static inline int lf_cas_uptr(lf_atomic_uptr_t *addr, uintptr_t expected, uintptr_t desired) {
    return __atomic_compare_exchange_n((uintptr_t *)addr, &expected, desired,
                                        0 /* strong */,
                                        __ATOMIC_SEQ_CST, __ATOMIC_SEQ_CST);
}
static inline uintptr_t lf_load_uptr(lf_atomic_uptr_t *addr) {
    return __atomic_load_n((uintptr_t *)addr, __ATOMIC_SEQ_CST);
}
static inline void lf_store_uptr(lf_atomic_uptr_t *addr, uintptr_t v) {
    __atomic_store_n((uintptr_t *)addr, v, __ATOMIC_SEQ_CST);
}
#define lf_fence()       __atomic_thread_fence(__ATOMIC_SEQ_CST)
#define lf_load_fence()  __atomic_thread_fence(__ATOMIC_ACQUIRE)
#define lf_store_fence() __atomic_thread_fence(__ATOMIC_RELEASE)

#elif defined(LF_TOOLCHAIN_ARMCC) && defined(LF_ARCH_ARM32)

/* Legacy ARM Compiler (armcc) on Cortex-M/A without C11 atomics:
 * hand-rolled LDREX/STREX exclusive-access loop. Valid on ARMv6+ /
 * Cortex-M3 and above (Cortex-M0/M0+ have no LDREX/STREX -- see
 * README for the interrupt-mask fallback used on those cores). */
#include <cmsis_compiler.h>
typedef volatile uintptr_t lf_atomic_uptr_t;

static __inline int lf_cas_uptr(lf_atomic_uptr_t *addr, uintptr_t expected, uintptr_t desired) {
    uintptr_t old;
    do {
        old = __LDREXW((volatile uint32_t *)addr);
        if (old != expected) { __CLREX(); return 0; }
    } while (__STREXW((uint32_t)desired, (volatile uint32_t *)addr) != 0);
    __DMB();
    return 1;
}
static __inline uintptr_t lf_load_uptr(lf_atomic_uptr_t *addr) {
    uintptr_t v = *addr; __DMB(); return v;
}
static __inline void lf_store_uptr(lf_atomic_uptr_t *addr, uintptr_t v) {
    __DMB(); *addr = v; __DMB();
}
#define lf_fence()       __DMB()
#define lf_load_fence()  __DMB()
#define lf_store_fence() __DMB()

#elif defined(LF_TOOLCHAIN_IAR) && defined(LF_ARCH_ARM32)

#include <intrinsics.h>
typedef volatile uintptr_t lf_atomic_uptr_t;

static inline int lf_cas_uptr(lf_atomic_uptr_t *addr, uintptr_t expected, uintptr_t desired) {
    uintptr_t old;
    do {
        old = __LDREX((unsigned long *)addr);
        if (old != expected) { __CLREX(); return 0; }
    } while (__STREX((unsigned long)desired, (unsigned long *)addr) != 0);
    __DMB();
    return 1;
}
static inline uintptr_t lf_load_uptr(lf_atomic_uptr_t *addr) {
    uintptr_t v = *addr; __DMB(); return v;
}
static inline void lf_store_uptr(lf_atomic_uptr_t *addr, uintptr_t v) {
    __DMB(); *addr = v; __DMB();
}
#define lf_fence()       __DMB()
#define lf_load_fence()  __DMB()
#define lf_store_fence() __DMB()

#else
#  error "atomic_hashdefs.h: no supported atomic backend for this toolchain/architecture. " \
         "Add a branch below (see README 'Porting to a new toolchain')."
#endif

/* --------------------------------------------------------------------
 * 4. CPU relax / spin-wait hint (reduces contention & power on a
 *    busy CAS retry loop; a no-op is always a correct fallback).
 * ------------------------------------------------------------------ */
#if defined(LF_ARCH_X86) && defined(LF_TOOLCHAIN_GNU)
#  define lf_cpu_relax() __asm__ __volatile__("pause" ::: "memory")
#elif defined(LF_ARCH_X86) && defined(LF_TOOLCHAIN_MSVC)
#  include <intrin.h>
#  define lf_cpu_relax() _mm_pause()
#elif defined(LF_ARCH_ARM) && defined(LF_TOOLCHAIN_GNU)
#  define lf_cpu_relax() __asm__ __volatile__("yield" ::: "memory")
#elif defined(LF_ARCH_ARM) && (defined(LF_TOOLCHAIN_ARMCC) || defined(LF_TOOLCHAIN_IAR))
#  define lf_cpu_relax() __YIELD()
#elif defined(LF_ARCH_PPC) && defined(LF_TOOLCHAIN_GNU)
   /* "or 27,27,27" is the classic Power ISA no-op hint used as a
    * software-visible low-priority yield on e500/e600 cores; on
    * e200 (no SMT hint support) it is simply a harmless no-op. */
#  define lf_cpu_relax() __asm__ __volatile__("or 27,27,27" ::: "memory")
#else
#  define lf_cpu_relax() do { /* no-op */ } while (0)
#endif

/* --------------------------------------------------------------------
 * 5. Cache-line size / alignment hint -- a performance optimization
 *    (avoiding false sharing on hot per-thread data such as
 *    hazard_ptr.c's per-thread rows), never a correctness requirement,
 *    so an unrecognized toolchain silently falls back to no alignment
 *    guarantee (LF_ALIGN(n) expands to nothing) rather than a #error.
 * ------------------------------------------------------------------ */
#if !defined(LF_CACHELINE_SIZE)
   /* 64 bytes covers the overwhelming majority of relevant cores: all
    * x86-64 since the Pentium 4, all Cortex-A, and e500/e600 PowerPC.
    * Cortex-M and e200 (no cache at all) simply pay a few bytes of
    * unused static RAM for the padding, which is harmless. Override
    * with -DLF_CACHELINE_SIZE=N at build time for a target where this
    * default is wrong and it actually matters. */
#  define LF_CACHELINE_SIZE 64
#endif

#if defined(LF_TOOLCHAIN_MSVC)
#  define LF_ALIGN(n) __declspec(align(n))
#elif defined(LF_TOOLCHAIN_GNU) || defined(LF_TOOLCHAIN_ARMCC)
#  define LF_ALIGN(n) __attribute__((aligned(n)))
#else
#  define LF_ALIGN(n) /* no portable spelling on this toolchain (e.g.
                        * IAR uses a #pragma, not an attribute, so it
                        * can't be expressed as a drop-in type
                        * qualifier here) -- padding-without-alignment
                        * still reduces false sharing somewhat but
                        * can't guarantee it; extend this branch for
                        * your toolchain if it matters there. */
#endif

#endif /* LF_ATOMIC_HASHDEFS_H */
