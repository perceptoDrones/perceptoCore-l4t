#!/bin/bash

# Build the Kernel for PerceptoCore/Percepto-1 Boards.

# Tested on Ubuntu 12.04 with GCC 4.8

# Requirements:
# 1. build-essential
# 2. arm-linux-gnueabihf-* toolchain

export ARCH=arm
export CROSS_COMPILE=/usr/bin/arm-linux-gnueabihf-

make percepto_defconfig

# Uncomment to launch kernel configuration (in X mode).
# Requires some QT libraries to be installed.
#make xconfig

make -j4

# We are cross-compiling, put modules in this folder:

mkdir -p $PWD/target_modules
make modules_install INSTALL_MOD_PATH=$PWD/target_modules

echo "Build Complete. Image Version Info: "
strings arch/arm/boot/Image  | grep "Linux version"
