# =======================================================================
# Makefile -- lock-free linked list
#
# Native targets (built and tested in this repo's CI/sandbox):
#   make test          native x86_64 build + run self-test/stress test
#   make tsan           same, instrumented with ThreadSanitizer
#   make asan           same, instrumented with AddressSanitizer
#   make clang          same as `test` but forced through clang
#   make pc-test        single-producer/single-consumer handoff test
#   make pc-tsan         same, instrumented with ThreadSanitizer
#   make pc-asan         same, instrumented with AddressSanitizer
#   make qtest          lock-free MPMC queue (insert/fetch_first) self-test + stress test
#   make qtsan           same, instrumented with ThreadSanitizer
#   make qasan           same, instrumented with AddressSanitizer
#
# Cross targets (commands documented; require the vendor toolchain
# installed on your machine -- see README.md "Building for each
# target" for where to get each one):
#   make arm-linux       32-bit ARM Linux (arm-linux-gnueabihf-gcc)
#   make aarch64-linux    64-bit ARM Linux (aarch64-linux-gnu-gcc)
#   make nxp-cortexm      bare-metal NXP Cortex-M, compile-only object
#                         (arm-none-eabi-gcc, e.g. MCUXpresso/S32DS toolchain)
#   make nxp-ppc          bare-metal NXP e200/e500 PowerPC, compile-only
#                         object (powerpc-eabi-gcc / S32DS PowerPC gcc)
# =======================================================================

CC        ?= gcc
CSTD      := -std=c11
WARN      := -Wall -Wextra -Wpedantic
OPT       := -O2 -g
# e.g. make qtest CFLAGS_EXTRA="-DNUM_PRODUCERS=1 -DNUM_CONSUMERS=1" for an
# SPSC-shaped queue run comparable to `make pc-test`'s sorted-list numbers.
CFLAGS_EXTRA ?=
SRC       := lockfree_list.c hazard_ptr.c
TESTSRC   := test_lockfree_list.c
PCTESTSRC := test_producer_consumer.c
HDRS      := lockfree_list.h hazard_ptr.h atomic_hashdefs.h

# The MPMC queue is a separate data structure (see lockfree_queue.h)
# that reuses hazard_ptr.c but NOT lockfree_list.c -- it links into its
# own standalone binary rather than being folded into $(SRC)/$(HDRS)
# above, per the "keep the current code, add extra code" design: the
# existing targets are completely unaffected by the queue's presence.
QSRC      := lockfree_queue.c hazard_ptr.c
QTESTSRC  := test_lockfree_queue.c
QHDRS     := lockfree_queue.h hazard_ptr.h atomic_hashdefs.h

.PHONY: all test tsan asan clang pc-test pc-tsan pc-asan \
        qtest qtsan qasan clean \
        arm-linux aarch64-linux nxp-cortexm nxp-ppc

all: test

# ARGS is passed through to the test binary, e.g.:
#   make test ARGS="0 1"     # pin worker threads round-robin to cores 0,1
# See README.md "Pinning worker threads to specific cores". Empty by
# default, i.e. unchanged (unpinned) behavior.
ARGS ?=

test: test_lockfree_list
	./test_lockfree_list $(ARGS)

test_lockfree_list: $(SRC) $(TESTSRC) $(HDRS)
	$(CC) $(CSTD) $(WARN) $(OPT) -pthread -o $@ $(SRC) $(TESTSRC)

tsan: $(SRC) $(TESTSRC) $(HDRS)
	$(CC) $(CSTD) $(WARN) -O1 -g -fsanitize=thread -pthread \
	    -o test_lockfree_list_tsan $(SRC) $(TESTSRC)
	./test_lockfree_list_tsan $(ARGS)

asan: $(SRC) $(TESTSRC) $(HDRS)
	$(CC) $(CSTD) $(WARN) -O1 -g -fsanitize=address,undefined -pthread \
	    -o test_lockfree_list_asan $(SRC) $(TESTSRC)
	./test_lockfree_list_asan $(ARGS)

clang: $(SRC) $(TESTSRC) $(HDRS)
	clang $(CSTD) $(WARN) $(OPT) -pthread -o test_lockfree_list_clang $(SRC) $(TESTSRC)
	./test_lockfree_list_clang

# Single-producer/single-consumer handoff test -- see README.md
# "Producer/consumer test" and the comment block at the top of
# test_producer_consumer.c. PC_ARGS is passed through, e.g.:
#   make pc-test PC_ARGS="50000 0 1"   # 50000 items, producer on core 0, consumer on core 1
PC_ARGS ?=

pc-test: test_producer_consumer
	./test_producer_consumer $(PC_ARGS)

test_producer_consumer: $(SRC) $(PCTESTSRC) $(HDRS)
	$(CC) $(CSTD) $(WARN) $(OPT) -pthread -o $@ $(SRC) $(PCTESTSRC)

pc-tsan: $(SRC) $(PCTESTSRC) $(HDRS)
	$(CC) $(CSTD) $(WARN) -O1 -g -fsanitize=thread -pthread \
	    -o test_producer_consumer_tsan $(SRC) $(PCTESTSRC)
	./test_producer_consumer_tsan $(PC_ARGS)

pc-asan: $(SRC) $(PCTESTSRC) $(HDRS)
	$(CC) $(CSTD) $(WARN) -O1 -g -fsanitize=address,undefined -pthread \
	    -o test_producer_consumer_asan $(SRC) $(PCTESTSRC)
	./test_producer_consumer_asan $(PC_ARGS)

# Lock-free MPMC queue (insert/fetch_first, no key) -- see
# lockfree_queue.h. QARGS is passed through, e.g.:
#   make qtest QARGS="0 1 2 3 4 5 6 7"   # pin the 8 worker threads to cores 0-7
QARGS ?=

qtest: test_lockfree_queue
	./test_lockfree_queue $(QARGS)

test_lockfree_queue: $(QSRC) $(QTESTSRC) $(QHDRS)
	$(CC) $(CSTD) $(WARN) $(OPT) $(CFLAGS_EXTRA) -pthread -o $@ $(QSRC) $(QTESTSRC)

qtsan: $(QSRC) $(QTESTSRC) $(QHDRS)
	$(CC) $(CSTD) $(WARN) -O1 -g -fsanitize=thread -pthread \
	    -o test_lockfree_queue_tsan $(QSRC) $(QTESTSRC)
	./test_lockfree_queue_tsan $(QARGS)

qasan: $(QSRC) $(QTESTSRC) $(QHDRS)
	$(CC) $(CSTD) $(WARN) -O1 -g -fsanitize=address,undefined -pthread \
	    -o test_lockfree_queue_asan $(QSRC) $(QTESTSRC)
	./test_lockfree_queue_asan $(QARGS)

# ---- Cross builds -----------------------------------------------------
# These validate compilation on non-x86 architectures. Runtime testing
# on real hardware/QEMU is outside this Makefile's scope -- see README.

arm-linux: $(SRC) $(HDRS)
	arm-linux-gnueabihf-gcc $(CSTD) $(WARN) $(OPT) -pthread \
	    -o test_lockfree_list_armhf $(SRC) $(TESTSRC)

aarch64-linux: $(SRC) $(HDRS)
	aarch64-linux-gnu-gcc $(CSTD) $(WARN) $(OPT) -pthread \
	    -o test_lockfree_list_aarch64 $(SRC) $(TESTSRC)

# Bare-metal NXP Cortex-M (e.g. LPC/Kinetis/i.MX RT/S32K): no OS, no
# pthread -- compile the list itself (not the pthread test harness) to
# confirm it builds standalone for your MCUXpresso/S32DS project.
nxp-cortexm: $(SRC) $(HDRS) lockfree_queue.c lockfree_queue.h
	arm-none-eabi-gcc $(CSTD) $(WARN) -O2 -g -mcpu=cortex-m4 -mthumb \
	    -DLF_TARGET_NXP=1 -c lockfree_list.c lockfree_queue.c hazard_ptr.c
	@echo "Built lockfree_list.o / lockfree_queue.o / hazard_ptr.o for Cortex-M4 (adjust -mcpu for your part)."

# Bare-metal NXP QorIQ / MPC5xx (e200/e500 Power Architecture core).
nxp-ppc: $(SRC) $(HDRS) lockfree_queue.c lockfree_queue.h
	powerpc-eabi-gcc $(CSTD) $(WARN) -O2 -g -mcpu=e200z4 \
	    -DLF_TARGET_NXP=1 -c lockfree_list.c lockfree_queue.c hazard_ptr.c
	@echo "Built lockfree_list.o / lockfree_queue.o / hazard_ptr.o for e200z4 (adjust -mcpu for your part, e.g. e500v2/e6500)."

clean:
	rm -f test_lockfree_list test_lockfree_list_tsan test_lockfree_list_asan \
	      test_lockfree_list_clang test_lockfree_list_armhf test_lockfree_list_aarch64 \
	      test_producer_consumer test_producer_consumer_tsan test_producer_consumer_asan \
	      test_lockfree_queue test_lockfree_queue_tsan test_lockfree_queue_asan \
	      *.o
