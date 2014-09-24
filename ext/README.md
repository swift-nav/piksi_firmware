### External Module Support

External modules to be linked with the Piksi firmware at build time must
be placed in subdirectories.  Each modules must provide a `Makefile.include`
which will be included when building the Piksi firmware.  This makefile should
add source files to the variable `CSRC`, and add the name of a setup function
to the variable `EXT_SETUP`.  For example:
```
THIS_EXT_DIR := $(dir $(lastword $(MAKEFILE_LIST)))
CSRC += $(THIS_EXT_DIR)sample.c

EXT_SETUP += sample_setup
```

The setup function will be called on initialisation after all the internal
setup is complete.  It may create a new thread for the module and/or register
callback functions for other event.

The module may make calls directly to functions in the Piksi firmware, but
all calls to the external module should be through callback functions
registered by the setup function.

