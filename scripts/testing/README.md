Hardware in the Loop Testing
============================

These scripts are used for automated testing of `piksi_firmware`.

Preparation
-----------

The first task in any automated test is to set the hardware to a known
configuration. This is achieved by running the script:

```
./prepare_test.sh DEV1 DEV2 STM_FW NAP_FW
```

where `DEV1` and `DEV2` are the Piksi devices to prepare, e.g. `/dev/ttyUSB0`.

The `prepare_test.sh` script performs several operations:

 * Powers down both devices (using `device_power.py` and `hub_ctrl`)
 * Powers up both devices
 * Fully erases the STM flash memory of both devices
 * In parallel, programs each device with `STM_FW` and `NAP_FW` using
   the bootloader

NOTE: The port power control script `device_power.py` is designed for use with
the Digi HUBPORT/14, but should be easy to modify to uspport any hub that has
PPPS.


