# I guess that requiring people to instal Scons is silly, so I'll add this
# Makefile.

UNAME := $(shell uname -m)
WARNING_FLAGS := -Wall -Wextra -Wshadow -Wswitch-enum \
    -Wswitch-default
C_WARNING_FLAGS := -Wmissing-prototypes -Wmissing-declarations \
    -Wstrict-prototypes

ifeq ($(UNAME), armv6l)
	override CFLAGS += ${WARNING_FLAGS} ${C_WARNING_FLAGS} -DRASPI=1
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

