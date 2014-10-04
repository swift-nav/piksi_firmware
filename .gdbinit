set mem inaccessible-by-default off
target extended-remote /dev/tty.usbmodemDDE58AC1
mon jtag_scan 4 5 6
attach 1
