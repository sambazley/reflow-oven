CFLAGS+=-Wall -Wextra -Og -ggdb
CFLAGS+=-I..

CFLAGS+=$(shell pkgconf --cflags libusb-1.0)
LDLIBS+=$(shell pkgconf --libs libusb-1.0)

ovenctl: ovenctl.o

clean:
	rm -f *.o ovenctl

PHONY: clean
