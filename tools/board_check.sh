#!/bin/sh
# Board checklist, steps 1 to 7 and 9 of the list in src/io.hpp (8 and 10 need vioh running).
# Run from the laptop, with the APB plugged in over USB-C:
#   ssh root@192.168.75.1 sh -s < tools/board_check.sh 2>&1 | tee board_check.txt
# It only reads, except step 3: that streams 120 camera frames, which is the thing step 4 measures.
# It changes no setting on purpose and restores nothing.

echo "== 1. kernel and camera driver (expect 6.6.52-orqa..., mxc-isi-cap_v1)"
uname -r
v4l2-ctl -d /dev/video2 --info | grep -i driver

echo "== services (expect orqa-digital-osd inactive)"
for s in orqa-digital-osd rth-event-setup rth-flight-logger vioh nadirhold; do echo "$s: $(systemctl is-active $s)"; done

echo "== 2. who holds our devices (expect nobody)"
BUSY=""
for pid in /proc/[0-9]*; do
    for dev in /dev/video2 /dev/ttymxc2 /dev/iio:device1 /dev/iio:device2; do
        if ls -l $pid/fd 2>/dev/null | grep -q " $dev\$"; then
            echo "$dev is held by pid ${pid#/proc/} ($(cat $pid/comm))"
            BUSY="$BUSY $dev"
        fi
    done
done
[ -z "$BUSY" ] && echo "nobody"

echo "== 3. camera stream, 1920x1080 YUYV (expect bytesused 4147200, about 60 fps)"
for m in /dev/media*; do media-ctl -d $m -p; done > /tmp/media_before 2>&1
case "$BUSY" in
    *video2*) echo "SKIPPED: /dev/video2 is in use" ;;
    *) v4l2-ctl -d /dev/video2 --set-fmt-video=width=1920,height=1080,pixelformat=YUYV --stream-mmap=4 \
           --stream-count=120 --verbose 2>&1 | tail -4 ;;
esac

echo "== 4. did streaming change the sensor / media setup? (expect no lines below)"
for m in /dev/media*; do media-ctl -d $m -p; done > /tmp/media_after 2>&1
diff /tmp/media_before /tmp/media_after && echo "no change"
echo "-- sensor formats now:"
grep -iE "entity|fmt:" /tmp/media_after | grep -iB1 "fmt:" | head -20

echo "== 5. IIO devices (expect dps310, icm42605-gyro, icm42605-accel)"
grep . /sys/bus/iio/devices/iio:device*/name

G=$(dirname $(grep -l icm42605-gyro /sys/bus/iio/devices/iio:device*/name))
A=$(dirname $(grep -l icm42605-accel /sys/bus/iio/devices/iio:device*/name))
B=$(dirname $(grep -l dps310 /sys/bus/iio/devices/iio:device*/name))

echo "== 6. IMU record layout (expect x y z be:s16/16>>0 index 0 1 2, temp index 3, timestamp le:s64/64>>0 index 4)"
grep . $G/scan_elements/*_type $G/scan_elements/*_index $A/scan_elements/*_type

echo "== 7. IMU rates, scales, clock (expect 12.5..4000; 0.001065264; 0.004788403; realtime or monotonic)"
grep . $G/sampling_frequency_available $G/sampling_frequency $A/sampling_frequency $G/in_anglvel_scale \
    $A/in_accel_scale $G/current_timestamp_clock $A/current_timestamp_clock
echo "-- IMU bus (I2C or SPI):"
readlink -f $G
dmesg | grep -i "fifo full" | tail -3

echo "== 9. barometer (expect kPa near 100; read time <= 1 / rate)"
grep . $B/in_pressure_sampling_frequency $B/in_pressure_oversampling_ratio
time cat $B/in_pressure_input

echo "== space for logs and recordings"
df -h /data | tail -1
echo "== done"
