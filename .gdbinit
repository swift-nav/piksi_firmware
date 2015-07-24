python
import sys
sys.path.append("./gdb_chibios")
import coredump
end

break _screaming_death
commands
gcore
cont
end

break debug_threads
commands
gcore
cont
end

catch signal SIGSEGV
command
run
end

set mem inaccessible-by-default off
target extended-remote /dev/ttyACM0
mon jtag_scan 4 5 6
attach 1
