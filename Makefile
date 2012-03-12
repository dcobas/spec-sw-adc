
DIRS = kernel tools doc

all clean:
	for d in $(DIRS); do $(MAKE) -C $$d $@ || exit 1; done
