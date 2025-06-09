#!/bin/bash
ROOT_DIR=$1
if [ -z "$ROOT_DIR" ]; then
    ROOT_DIR="."
fi
FILENAME_TEMPL=ublox_
DEVICE=`echo /dev/serial/by-id/usb-u-blox*`
BAUDRATE=115200

for file_id in $( seq 0 999 ); do
    FILE_DIR=$( printf "$FILENAME_TEMPL%03d.ubx" $file_id )
    FILE_PATH=$ROOT_DIR/$FILE_DIR
    if [ ! -e "$FILE_PATH" ]; then
	break
    fi
done

if [ ! -z $BAUDRATE ]; then
    stty -F $DEVICE $BAUDRATE
fi

python ublox_copyer.py $FILE_PATH $DEVICE $BAUDRATE

echo "$0 finished."

