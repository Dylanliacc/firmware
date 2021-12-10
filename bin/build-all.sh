#!/bin/bash

set -e

VERSION=`bin/buildinfo.py long`
SHORT_VERSION=`bin/buildinfo.py short`

BOARDS_ESP32="tlora-v2 tlora-v1 tlora_v1_3 tlora-v2-1-1.6 tbeam heltec-v2.0 heltec-v2.1 tbeam0.7 meshtastic-diy-v1"
#BOARDS_ESP32=tbeam

# FIXME note nrf52840dk build is for some reason only generating a BIN file but not a HEX file nrf52840dk-geeksville is fine
BOARDS_NRF52="rak4631_5005 rak4631_19003 t-echo"
#BOARDS_NRF52=""

OUTDIR=release/latest

# We keep all old builds (and their map files in the archive dir)
ARCHIVEDIR=release/archive 

rm -f $OUTDIR/firmware*

mkdir -p $OUTDIR/bins $ARCHIVEDIR
rm -r $OUTDIR/bins/* || true
mkdir -p $OUTDIR/bins/universal $OUTDIR/elfs/universal

# build the named environment and copy the bins to the release directory
function do_build() {
	BOARD=$1
	isNrf=$3
	
    echo "Building for $BOARD with $PLATFORMIO_BUILD_FLAGS"
    rm -f .pio/build/$BOARD/firmware.*

    # The shell vars the build tool expects to find
    export APP_VERSION=$VERSION

    # Are we building a universal/regionless rom?
    export HW_VERSION="1.0"
    basename=universal/firmware-$BOARD-$VERSION

    pio run --environment $BOARD # -v
    SRCELF=.pio/build/$BOARD/firmware.elf
    cp $SRCELF $OUTDIR/elfs/$basename.elf

    if [ "$isNrf" = "false" ]
    then
        echo "Copying ESP32 bin file"
        SRCBIN=.pio/build/$BOARD/firmware.bin
        cp $SRCBIN $OUTDIR/bins/$basename.bin
    else
        echo "Generating NRF52 uf2 file"
        SRCHEX=.pio/build/$BOARD/firmware.hex
        bin/uf2conv.py $SRCHEX -c -o $OUTDIR/bins/$basename.uf2 -f 0xADA52840
    fi
}

function do_boards() {
	declare boards=$1
	declare isNrf=$2
	for board in $boards; do
		# Build universal
		do_build $board "" "$isNrf" 
	done
}

# Make sure our submodules are current
git submodule update 

# Important to pull latest version of libs into all device flavors, otherwise some devices might be stale
platformio lib update 

do_boards "$BOARDS_ESP32" "false"
do_boards "$BOARDS_NRF52" "true"

pio run --environment native
cp .pio/build/native/program $OUTDIR/bins/universal/meshtasticd_linux_amd64

echo "Building SPIFFS for ESP32 targets"
pio run --environment tbeam -t buildfs
cp .pio/build/tbeam/spiffs.bin $OUTDIR/bins/universal/spiffs-$VERSION.bin

# keep the bins in archive also
cp $OUTDIR/bins/universal/spiffs* $OUTDIR/bins/universal/firmware* $OUTDIR/elfs/universal/firmware* $ARCHIVEDIR

echo Updating android bins $OUTDIR/forandroid
rm -rf $OUTDIR/forandroid
mkdir -p $OUTDIR/forandroid
cp -a $OUTDIR/bins/universal/*.bin $OUTDIR/forandroid/

cat >$OUTDIR/curfirmwareversion.xml <<XML
<?xml version="1.0" encoding="utf-8"?>

<!-- This file is kept in source control because it reflects the last stable
release.  It is used by the android app for forcing software updates.  Do not edit.
Generated by bin/buildall.sh -->

<resources>
    <string name="cur_firmware_version" translatable="false">$VERSION</string>
    <string name="short_firmware_version" translatable="false">$SHORT_VERSION</string>
</resources>
XML

echo Generating $ARCHIVEDIR/firmware-$VERSION.zip
rm -f $ARCHIVEDIR/firmware-$VERSION.zip
zip --junk-paths $ARCHIVEDIR/firmware-$VERSION.zip $ARCHIVEDIR/spiffs-$VERSION.bin $OUTDIR/bins/universal/firmware-*-$VERSION.* $OUTDIR/bins/universal/meshtasticd* images/system-info.bin bin/device-install.* bin/device-update.*
echo Generating $ARCHIVEDIR/elfs-$VERSION.zip
rm -f $ARCHIVEDIR/elfs-$VERSION.zip
zip --junk-paths $ARCHIVEDIR/elfs-$VERSION.zip $OUTDIR/elfs/universal/firmware-*-$VERSION.* 

echo BUILT ALL
