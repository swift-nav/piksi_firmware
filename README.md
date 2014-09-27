piksi_firmware
==============

Firmware for the Swift Navigation Piksi GPS Receiver.

Documentation available online at http://docs.swift-nav.com/piksi_firmware

For toolchain installation, see
  http://docs.swift-nav.com/wiki/Piksi_Developer_Getting_Started_Guide

Checking Out Submodules
=========================

ChibiOS, libopencm3 and libswiftnav are submodules of this git repository.
Check them out using:

	git submodule init
	git submodule update

Remember to run `git submodule update` after pulling in the latest changes to
ensure all the submodules are in sync.

Installation
=========================

To install these development tools for your platform, run the setup
script via `bash setup.sh`. The Vagrant file is currently used for
testing installation setup.sh, but can also be used to provision a
development VM.
