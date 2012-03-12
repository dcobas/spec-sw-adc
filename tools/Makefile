
CFLAGS = -ggdb -Wall
LDFLAGS = -L. -lspec

LIB = libspec.a
LIBOBJ = speclib.o

PROGS = specmem spec-cl


all: $(LIB) $(PROGS)

$(PROGS): $(LIB)

$(LIB): $(LIBOBJ)
	ar r $@ $^


clean:
	rm -f *.o $(LIB) $(PROGS) *~