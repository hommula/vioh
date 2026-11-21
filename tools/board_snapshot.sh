#!/bin/sh
# Read-only picture of the APB: nothing is streamed, nothing is written outside /tmp.
# Run before and after a session and compare the two files to see whether we changed anything:
#   ssh root@192.168.75.1 sh -s < tools/board_snapshot.sh > before.txt 2>&1
echo "== video devices"
v4l2-ctl --list-devices
for d in /dev/video*; do
    echo "-- $d"
    v4l2-ctl -d $d --info | grep -iE "driver|card|bus"
    v4l2-ctl -d $d --get-fmt-video 2>&1 | grep -iE "width|pixel format|size image"
done
echo "== media setup (links and formats)"
for m in /dev/media*; do echo "-- $m"; media-ctl -d $m -p; done
echo "== who holds our devices"
for pid in /proc/[0-9]*; do
    ls -l $pid/fd 2>/dev/null | grep -E "/dev/(video|ttymxc|iio:device|media)" | sed "s|^|pid ${pid#/proc/} $(cat $pid/comm 2>/dev/null): |"
done
echo "== sensors"
for d in /sys/bus/iio/devices/iio:device*; do
    echo "-- $d $(cat $d/name)"
    grep . $d/sampling_frequency $d/current_timestamp_clock $d/in_pressure_sampling_frequency \
        $d/in_pressure_oversampling_ratio $d/buffer/enable $d/buffer/length $d/scan_elements/*_en 2>/dev/null
done
echo "== services"
systemctl list-units --type=service --state=running --no-pager --no-legend | awk '{print $1}'
systemctl list-unit-files --type=service --state=enabled --no-pager --no-legend | awk '{print "enabled: " $1}'
echo "== /data"
ls -la /data
df -h /data | tail -1
