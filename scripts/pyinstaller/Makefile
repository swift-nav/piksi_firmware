ksi_tools_directory=../
spec_file=console.spec
pyinstaller=pyinstaller

LIBS=-lm

ifneq (,$(findstring /cygdrive/,$(PATH)))
    UNAME := Cygwin
    PLAT_ARGS = "-F"
else
ifneq (,$(findstring WINDOWS,$(PATH)))
    UNAME := Windows
    PLAT_ARGS=""
else
    UNAME := $(shell uname -s)
    PLAT_ARGS="-w"
endif
endif


console: $(spec_file)
	make clean
	pyinstaller --clean --log-level=DEBUG --paths=$(piksi_tools_directory) --debug $(PLAT_ARGS) $(spec_file) -y >& manifest.out
	echo "Pyinstaller completed. View output in manifest.out"
	make $(UNAME)

Windows:
	echo "Makefile not configured to build Windows installers"
Cygwin: 
	echo "Makefile not configured to build Windows installers"


Darwin:
	sudo ./create-dmg-installer.sh
clean:
	rm -rf build dist manifest.out


