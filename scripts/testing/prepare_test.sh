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

# Power down devices
./device_power.py $1 0
./device_power.py $2 0

sleep 2

# Power up devices
./device_power.py $1 1
./device_power.py $2 1

sleep 2

# Bootload STM firmware, doing a full erase
cd ..
pwd
./bootload.py -e -s -p $1 testing/$3 > testing/bootload_log1 &
./bootload.py -e -s -p $2 testing/$3 > testing/bootload_log2
wait

cd testing

# Power down devices again
./device_power.py $1 0
./device_power.py $2 0

sleep 2

# Power up devices
./device_power.py $1 1
./device_power.py $2 1

sleep 2

# Bootload NAP firmware
cd ..
./bootload.py -m -p $1 testing/$4  >> testing/bootload_log1 &
./bootload.py -m -p $2 testing/$4  >> testing/bootload_log2
wait

cd testing
