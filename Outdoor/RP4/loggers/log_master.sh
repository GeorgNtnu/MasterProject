#!/bin/bash
ROOT_DIR=$HOME/data
FLIGHT_DIR_NAME=flight_
INTERFACE=  

for flight_id in $( seq 0 999 ); do
    FLIGHT_DIR=$( printf "$FLIGHT_DIR_NAME%03d" $flight_id )
    FLIGHT_PATH=$ROOT_DIR/$FLIGHT_DIR
    if [ ! -e "$FLIGHT_PATH" ]; then
	break
    fi
done

echo "Logging to $FLIGHT_DIR"
mkdir -p $FLIGHT_PATH/logs
if [ ! $? -eq 0 ]; then
    echo "Could not create logging directory."
    exit 1
fi

PID=$$
PGID=$(ps -o pgid= $PID | grep -o '[0-9]*')


./ublox.sh  $FLIGHT_PATH >> $FLIGHT_PATH/logs/ublox.log &
ublox_pid=$!


running=true
trap ctrl_c INT
function ctrl_c() {
    trap - INT
    running=false
}

while $running ; do
    sleep 1
done

kill -- -$PGID

sleep 0.5;

STILL_RUNNING=$(ps aux | grep -v grep | grep -E '(ublox|vector|tshark)')
if [ ! -z "$STILL_RUNNING" ]; then
    echo "Processes still running:"
    echo "$STILL_RUNNING"
fi
