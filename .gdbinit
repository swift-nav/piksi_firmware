set mem inaccessible-by-default off
target extended-remote /dev/ttyACM0
mon jtag_scan 4 5 6
attach 1
