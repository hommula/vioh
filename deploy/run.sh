#!/bin/sh
# Runs on the APB: start vioh in the foreground. Stop with Ctrl-C.
# vioh changes the IMU rate, the IMU clock and the barometer rate; they stay changed until the next reboot.
exec /data/nadirhold/vioh /data/nadirhold/config.txt /data/nadirhold/logs
