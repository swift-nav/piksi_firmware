#!/bin/bash -e
#
# Create an .dmg installer
# Based on script from Orange3 project (https://github.com/biolab/orange3)

# Copyright (c) 2013 Laboratory of Bioinformatics, Faculty of Computer and
# Information Science, University of Ljubljana, Slovenia
#
# Copyright (c) 2014 Swift Navigation Inc.
#
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
# 1. Redistributions of source code must retain the above copyright notice, this
#    list of conditions and the following disclaimer.
# 2. Redistributions in binary form must reproduce the above copyright notice,
#    this list of conditions and the following disclaimer in the documentation
#    and/or other materials provided with the distribution.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
# ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
# WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR
# ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
# (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
# ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
# (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
# SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

function print_usage() {
	echo 'create-dmg-installer.sh --app BUILD_APP_PATH --out OUTPUT_BUNDLE.dmg

Create an disk image installer (.dmg) for Piksi Console OSX application.

Options:

    -a --app PATH      Path to a build Piksi Console.app to include in the disk image.
                       (default dist/Piksi Console.app)
    -o --out PATH      Name for the generated the disk image.
                       (default dist/piksi_console_VERSION.dmg)
    -k --keep-temp     Keep the temporary files after creating the final image.
    -h --help          Print this help
'
}

DIRNAME=$(dirname "$0")

# Path to dmg resources (volume icon, background, ...)
RES=$DIRNAME/dmg-resources

APP="dist/Piksi Console.app"
DMG="dist/piksi_console_`git describe --dirty`.dmg"

KEEP_TEMP=0

while [[ ${1:0:1} = "-" ]]; do
    case $1 in
        -a|--app)
            APP=$2
            shift 2
            ;;
        -o|--out)
            DMG=$2
            shift 2
            ;;
        -k|--keep-temp)
            KEEP_TEMP=1
            shift 1
            ;;
        -h|--help)
            print_usage
            exit 0
            ;;
        -*)
            echo "Unknown option $1"
            print_usage
            exit 1
            ;;
    esac
done

if [[ ! $DMG ]]; then
    echo "No output bundle dmg specified."
    print_usage
    exit 1
fi

if [[ ! -d $APP ]]; then
    echo "$APP path does not exits or is not a directory."
    print_usage
    exit 1
fi

TMP_DIR=$(mktemp -d -t console-dmg)
TMP_TEMPLATE=$TMP_DIR/template
TMP_DMG=$TMP_DIR/piksi_console_`git describe --dirty`.dmg

echo "Preparing an image template in $TMP_TEMPLATE"
echo "============================================="

# Copy neccessary resources into the template
#mkdir -p "$TMP_TEMPLATE"/.background
#cp -a "$RES"/background.png "$TMP_TEMPLATE"/.background

cp -a "$RES"/VolumeIcon.icns "$TMP_TEMPLATE"/.VolumeIcon.icns

#cp -a "$RES"/DS_Store "$TMP_TEMPLATE"/.DS_Store

# Create a link to the Applications folder.
ln -s /Applications/ "$TMP_TEMPLATE"/Applications

# Copy the .app directory in place
cp -a "$APP" "$TMP_TEMPLATE/Piksi Console.app"

# Remove unnecesary files.
#find "$TMP_TEMPLATE"/Piksi Console.app/Contents/ \( -name '*~' -or -name '*.bak' -or -name '*.pyc' -or -name '*.pyo' \) -delete

# Create a regular .fseventsd/no_log file
# (see http://hostilefork.com/2009/12/02/trashes-fseventsd-and-spotlight-v100/ )

mkdir "$TMP_TEMPLATE"/.fseventsd
touch "$TMP_TEMPLATE"/.fseventsd/no_log


echo "Creating a temporary disk image"
hdiutil create -format UDRW -volname "Piksi Console" -fs HFS+ \
       -fsargs "-c c=64,a=16,e=16" \
       -srcfolder "$TMP_TEMPLATE" \
       "$TMP_DMG"

# Force detatch an image it it is mounted
hdiutil detach "/Volumes/Piksi Console" -force || true

# Mount in RW mode
echo "Mounting temporary disk image"
MOUNT_OUTPUT=$(hdiutil attach -readwrite -noverify -noautoopen "$TMP_DMG" | egrep '^/dev/')

DEV_NAME=$(echo -n "$MOUNT_OUTPUT" | head -n 1 | awk '{print $1}')
MOUNT_POINT=$(echo -n "$MOUNT_OUTPUT" | tail -n 1 | awk '{print substr($0, index($0,$3))}')

echo "Fixing permissions."

chmod -Rf go-w "$TMP_TEMPLATE" || true

# Makes the disk image window open automatically when mounted
bless -openfolder "$MOUNT_POINT"

# Hides background directory even more
#SetFile -a V "$MOUNT_POINT/.background/"

# Sets the custom icon volume flag so that volume has nice
# icon after mount (.VolumeIcon.icns)
SetFile -a C "$MOUNT_POINT"

hdiutil detach "$DEV_NAME" -force

echo "Converting temporary image to a compressed image."

if [[ -e $DMG ]]; then
	rm -f "$DMG"
fi

hdiutil convert "$TMP_DMG" -format UDZO -imagekey zlib-level=9 -o "$DMG"

if [ ! $KEEP_TEMP ]; then
    echo "Cleaning up."
    rm -rf "$TMP_DIR"
fi
