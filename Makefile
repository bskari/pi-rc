# I guess that requiring people to instal Scons is silly, so I'll add this
# Makefile.

WARNING_FLAGS := -Wall -Wextra -Wshadow -Wswitch-enum \
    -Wswitch-default
C_WARNING_FLAGS := -Wmissing-prototypes -Wmissing-declarations \
    -Wstrict-prototypes

CFLAGS := ${WARNING_FLAGS} ${C_WARNING_FLAGS}
LDFLAGS := -lm -ljansson

all:	pi_pcm

pi_pcm.o:	pi_pcm.c

.PHONY:	clean
clean:
	rm -f *.o
	rm -f *.pyc
	rm -f pi_pcm
