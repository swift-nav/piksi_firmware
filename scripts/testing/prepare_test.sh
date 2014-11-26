#!/bin/bash -vxe

# Prepare a pair of devices for testing
# Usage:
#   ./prepare_test.sh DEV1 DEV2 STM_FW NAP_FW

# Check devices exist

[ -e $1 ]
[ -e $2 ]

# Check firmware files exist

[ -e $3 ]
[ -e $4 ]

# Build hub_ctrl

make

# For each device:
# 1) Power cycle
# 2) Load STM firmware using bootloader, doing a full erase of the STM flash
# 3) Power cycle again
# 4) Load NAP firmware

# ==== DEVICE 1 =============================================

./cycle_device_power.py $1
sleep 1
cd ..
./bootload.py -e -s -p $1 testing/$3 > testing/bootload_log1
cd testing
./cycle_device_power.py $1
sleep 1
cd ..
./bootload.py -m -p $1 testing/$4  >> testing/bootload_log1
cd testing

# ==== DEVICE 2 =============================================

./cycle_device_power.py $2
sleep 1
cd ..
./bootload.py -e -s -p $2 testing/$3 > testing/bootload_log1
cd testing
./cycle_device_power.py $2
sleep 1
cd ..
./bootload.py -m -p $2 testing/$4  >> testing/bootload_log1
cd testing

