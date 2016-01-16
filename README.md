piksi_firmware
==============

[![Build status][1]][2]

Firmware for the Swift Navigation Piksi GPS Receiver.

Documentation available online at http://docs.swift-nav.com/piksi_firmware

Checking Out Submodules
=========================

ChibiOS, libopencm3, libsbp and libswiftnav are submodules of this git repository.
Check them out using:

	git submodule update --init

Remember to run `git submodule update` after pulling in the latest changes to
ensure all the submodules are in sync.

Installation
============

There are a few options:

* **Normal usage**. If you're only using the Piksi console, binary
  installers (Windows and OS X) are
  [here](http://downloads.swiftnav.com/piksi_console/) and source
  for the console can be found in
  [piksi_tools](https://github.com/swift-nav/piksi_tools).

* **Development (native)**. To install dependencies for the
  development tools on your platform (OS X, Ubuntu, or Debian), run
  the setup script in this repository via `bash setup.sh -x
  install`. If you're also building the firmware, you'll need to
  checkout the submodules as well.

* **Development (VM)**. The Vagrant file is currently used for testing
  installation setup.sh, but can also be used to provision a
  development VM. To do so, you will need to download
  [VirtualBox](https://www.virtualbox.org/wiki/Downloads) and
  [Vagrant](http://www.vagrantup.com/downloads.html), and then run
  `vagrant up trusty` in this repository.

For additional details about the toolchain installation, please see
  http://docs.swift-nav.com/wiki/Piksi_Developer_Getting_Started_Guide .

[1]: https://travis-ci.org/swift-nav/piksi_firmware.svg?branch=master
[2]: https://travis-ci.org/swift-nav/piksi_firmware

