
.PHONY: all firmware docs

all: firmware docs

firmware:
	cd src; $(MAKE) $(MFLAGS)

docs:
	doxygen docs/Doxyfile

