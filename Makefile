# I guess that requiring people to instal Scons is silly, so I'll add this
# Makefile.

UNAME := $(shell uname -m)
MODEL := $(shell cat /proc/device-tree/model)
WARNING_FLAGS := -Wall -Wextra -Wshadow -Wswitch-enum \
    -Wswitch-default
C_WARNING_FLAGS := -Wmissing-prototypes -Wmissing-declarations \
    -Wstrict-prototypes

# It would be nice to use MODEL for all of these checks, but I only have a
# 3 and a Zero to test with
ifeq ($(UNAME), armv6l)
	override CFLAGS += ${WARNING_FLAGS} ${C_WARNING_FLAGS} -DRASPI=1
else ifeq ($(findstring Pi 4,$(MODEL)),Pi 4)
	# The 4 is weird, because some kernels boot in AARCH32 mode, so it might
	# report armv7l instead of arm8vl even though it's a Pi 4
	echo "*** Pi 4 support is experimental, because I don't have one to test ***"
	override CFLAGS += ${WARNING_FLAGS} ${C_WARNING_FLAGS} -DRASPI=4
else ifeq ($(UNAME), armv7l)
	override CFLAGS += ${WARNING_FLAGS} ${C_WARNING_FLAGS} -DRASPI=2
else ifeq ($(UNAME), x86_64)
	# Options for testing on non-Pi hardware
	override CFLAGS += ${WARNING_FLAGS} ${C_WARNING_FLAGS} -g -Wno-pointer-to-int-cast \
		-Wno-unused-function -DTEST_COMPILATION=1
else
	override CFLAGS += ${WARNING_FLAGS} ${C_WARNING_FLAGS}
endif

LDLIBS := -lm

pi_pcm: pi_pcm.o mailbox.o

.PHONY:	clean
clean:
	rm -f *.o
	rm -f *.pyc
	rm -f pi_pcm

