#!/bin/sh
# Measured evidence for the "timing and rolling shutter" bonus, from our own aircraft.
# Run from the repo folder in Git Bash, aircraft plugged in, props off:
#
#   sh tools/record_timing.sh
#
# For 25 seconds, hold the aircraft and ROTATE it only: tip it left-right and nose up-down, a few
# degrees, quickly, without walking. Point the camera at something at least 2 m away (a wall or the
# floor from a table edge), so that moving the aircraft sideways hardly moves the picture.
# The script stops vioh, records, starts vioh again, and runs the measurement on the laptop.
set -e
APB=root@192.168.75.1
OUT=/d/MISC/new/data/board
echo "Recording 25 s: rotate the aircraft now (tip left-right, nose up-down)..."
ssh $APB 'systemctl stop nadirhold; pkill -x vioh; sleep 1; cd /data/nadirhold && ./vioh config.txt record /dev/shm/timing.vlog 25 2'
scp $APB:/dev/shm/timing.vlog $OUT/timing.vlog
ssh $APB 'rm -f /dev/shm/timing.vlog; systemctl start nadirhold'
echo "vioh is running again. Measuring..."
MSYS_NO_PATHCONV=1 docker run --rm -v "D:/MISC/new/repo:/src" -v "D:/MISC/new/data:/data" vioh-dev bash -c \
  "/data/build-x86/calib /data/board/timing.vlog 250 /data/board/timing_sweep.csv | tee /data/board/timing_result.txt && \
   python /src/tools/evidence_plots.py /data/board/timing_sweep.csv /data/sim_relock.csv /data/sim_norelock.csv /src/docs/evidence"
echo "Result: $OUT/timing_result.txt   Picture: docs/evidence/evidence_camera_delay.png"
echo "Copy the lines under '# config lines' into config.example if the residual is clearly lower than 'with unrelated gyro'."
