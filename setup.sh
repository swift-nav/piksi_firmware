#!/usr/bin/bash

# Copyright (C) 2011-2014 Swift Navigation Inc.
# Contact: Bhaskar Mookerji <mookerji@swiftnav.com>

# This source is subject to the license found in the file 'LICENSE' which must
# be be distributed together with this source. All other rights reserved.

# THIS CODE AND INFORMATION IS PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
# EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED
# WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A PARTICULAR PURPOSE.
#
# Script for setting up piksi_firmware development environment across
# different development environments. It's not guaranteed to be
# idempotent, or have any other guarantees for that matter, but if
# you're having issues with your particular development platform,
# please let us know.

####################################################################
## Utilities.

function color () {
    # Print with color.
    printf '\033[%sm%s\033[m\n' "$@"
}

####################################################################
## Linux dependency management and build

function piksi_splash_linux () {
    color '35;5' "
          _/\/\/\/\/\____/\/\____/\/\____________________/\/\___
          _/\/\____/\/\__________/\/\__/\/\____/\/\/\/\_________
          _/\/\/\/\/\____/\/\____/\/\/\/\____/\/\/\/\____/\/\___
          _/\/\__________/\/\____/\/\/\/\__________/\/\__/\/\___
         _/\/\__________/\/\/\__/\/\__/\/\__/\/\/\/\____/\/\/\__

         Welcome to piksi_firmware development installer!

    "
}

function sys_dependencies_apt () {
    # Install system and python library dependencies using an
    # apt-based installer.
    # Tested with Ubuntu 12.04 running in Vagrant/Virtualbox.
    color '35;1' 'Checking system dependencies for Linux...'
    color '35;1' 'Please enter your password for ...'
    sudo apt-get update
    sudo apt-get -y install \
        git \
        build-essential \
        python \
        python-setuptools \
        python-virtualenv \
        cmake \
        doxygen \
        libftdi \
        swig \
        wget \
        cmake==2.8.9
    sudo pip install 'pip>=1.5.6' 'setuptools>=5.3'
    sudo apt-get -y install \
        libicu-dev \
        libqt4-scripttools \
        python-enable \
        python-chaco \
        python-vtk \
        python-wxgtk2.8 \
        python-pyside \
        python-qt4-dev \
        python-sip \
        python-qt4-gl
    # Cleanup any existing files that may be here.
    find . -name \*.pyc -delete
    # TODO (Buro): Tested with a vagrant Ubuntu: perhaps move to a
    # virtual env?
    sudo pip install \
        traits \
        traitsui \
        pyserial \
        pylibftdi \
        pyparsing==1.5.7 \
        pygments \
        intelhex \
        --allow-unverified PIL \
        --allow-external intelhex \
        --allow-unverified intelhex
    color '35;1' 'Setting up ARM GCC toolchain...'
    sudo apt-get install python-software-properties
    sudo add-apt-repository ppa:terry.guo/gcc-arm-embedded
    sudo apt-get update
    sudo apt-get install gcc-arm-none-eabi
}

function build () {
    # Pulls down git submodules and builds the project, assuming that
    # all other system, ARM GCC, and python dependencies have been
    # installed.
    color '35;1' 'Initializing Git submodules for ChibiOS, libopencm3, and libswiftnav...'
    git submodule init
    git submodule update
    color '35;1' 'Building piksi_firmware...'
    make clean
    make
}

####################################################################
## Mac OS X dependency management and build

function piksi_splash_osx () {
    color '35;5' "
          `7MM\"\"\"Mq.  db  `7MM                db
           MM   `MM.        MM
           MM   ,M9 `7MM    MM  ,MP',pP\"Ybd `7MM
           MMmmdM9    MM    MM ;Y   8I   `\"   MM
           MM         MM    MM;Mm   `YMMMa.   MM
           MM         MM    MM `Mb. L.   I8   MM
         .JMML.     .JMML..JMML. YA.M9mmmP' .JMML.

         Welcome to piksi_firmware development installer!

    "
}

function homebrew_install () {
    # Provides homebrew for OS X and fixes permissions for brew
    # access. Run this if you need to install brew by:
    #    source ./setup.sh
    #    homebrew_install
    ruby -e "$(curl -fsSL https://raw.githubusercontent.com/Homebrew/install/master/install)"
    brew doctor
    brew update
    # Homebrew apparently requires the contents of /usr/local to be
    # chown'd to your username.  See:
    # http://superuser.com/questions/254843/cant-install-brew-formulae-correctly-permission-denied-in-usr-local-lib
    sudo chown -R `whoami` /usr/local
}

function sys_dependencies_osx () {
    # Installs system dependencies for OS X. Requires that XCode and
    # Homebrew be installed and will exit otherwise.
    color '35;1' 'Checking system dependencies for OS X...'
    # TODO (Buro): Check using xcode-select -p
    if [ ! -x "/usr/bin/python" ]; then
        color '31;1' 'Please install Xcode developer tools and try again:'
        color '31;1' '    /usr/bin/xcode-select --install'
        exit 1
    fi
    if [ ! -x "/usr/local/bin/brew" ]; then
        color '31;1' 'Could not find homebrew.'
        color '31;1' 'Please install homebrew and try again. See homebrew_install() in this file.'
    fi
    brew install cmake \
        doxygen \
        qt \
        pyqt \
        libftdi \
        swig \
        wget \
        pyside
    color '35;1' 'Done with system dependencies for OS X...'
}

function python_setup_osx () {
    # Sets up python development environment using a virtualenv. This
    # is tailored to the particular idiosyncrasies of Enthought's
    # python installation on OSX.
    color '35;1' 'Setting up Python project build environment...'
    # Cleanup any existing files that may be here.
    find . -name \*.pyc -delete
    # Install an up-to-date pip.
    color '35;1' 'Installing pip and virtualenv...'
    sudo easy_install 'pip>=1.5.6'
    sudo pip install pip --upgrade
    sudo pip install virtualenv --upgrade
    # Build of enthought enable fails for some kind of legacy OS X
    # library dependency. If using a recent version of OS X, move the
    # offending to properly resolve build.
    # https://trac.macports.org/ticket/35735?cversion=0&cnum_hist=11
    OSX_MINOR_VERSION=`sw_vers -productVersion | cut -d . -f 2`
    if [[ $OSX_MINOR_VERSION > 6 ]]; then
        color '35;1' 'Moving /usr/local/include/uuid/uuid.h (see setup.sh) ...'
        mv /usr/local/include/uuid/uuid.h /usr/local/include/uuid/uuid.bak
    fi
    color '35;1' 'Create up Python a virtualenv named dev...'
    virtualenv dev
    source ./dev/bin/activate
    # Pip and virtualenv are having issues with safe dependency
    # resolution, so install numpy and cython first.
    pip install numpy==1.9.0 cython==0.21 distribute==0.7.3
    pip install chaco==4.4.1 enable==4.4.1 \
        --allow-unverified enable \
        --allow-unverified PIL \
        --allow-external intelhex \
        --allow-unverified intelhex
    pip install \
        traits==4.5.0 \
        traitsui==4.4.0 \
        pyface==4.4.0 \
        wsgiref==0.1.2 \
        pyside==1.2.2 \
        pyserial==2.7 \
        pylibftdi==0.14.2 \
        pyparsing==1.5.7 \
        pygments==1.6 \
        intelhex==1.5 \
        --allow-external intelhex \
        --allow-unverified intelhex
}

function install_arm_osx () {
    # Installs ARM GCC toolchain for OS X.
    color '35;1' 'Setting up ARM GCC toolchain...'
    wget https://launchpad.net/gcc-arm-embedded/4.7/4.7-2013-q1-update/+download/gcc-arm-none-eabi-4_7-2013q1-20130313-mac.tar.bz2
    tar -xf gcc-arm-none-eabi-4_7-2013q1-20130313-mac.tar.bz2
    mv gcc-arm-none-eabi-4_7-2013q1 ~/gcc-arm-none-eabi
    echo 'export PATH=$PATH:~/gcc-arm-none-eabi/gcc-arm-none-eabi-4_7-2013q1/bin' >> ~/.bash_profile
    source ~/.bash_profile
    mv gcc-arm-none-eabi-4_7-2013q1-20130313-mac.tar.bz2 ~/.Trash/
}

function build_osx () {
    build
    color '35;1' 'Importing piksi_firmware virtualenv dev into $PATH...'
    source ./dev/bin/activate
}

####################################################################
## Entry points

function run_all_platforms {
    if [[ "$OSTYPE" == "linux-gnu" ]]; then
        piksi_splash_linux
        sys_dependencies_apt
        build
    elif [[ "$OSTYPE" == "darwin"* ]]; then
        piksi_splash_osx
        sys_dependencies_osx
        python_setup_osx
        install_arm_osx
        build_osx
    else
        color '31;1' 'This script does not support this platform. Please contact mookerji@swiftnav.com'
        exit 1
    fi
    color '35;1' 'Done!.'
}

set -e

if [ ! -e ./setup.sh ] ; then
    color '31;1' "Error: setup.sh should be run from piksi_firmware toplevel." >&2
    exit 1
else
    run_all_platforms
fi
