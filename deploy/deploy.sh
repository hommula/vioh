#!/bin/sh
# Copy vioh to the APB. Run from the repo folder after the ARM build.
# Our folder and unit are called "nadirhold": the board already has a vioh.service and /data/vioh
# path from another logger (vioh_log), which we must not touch.
#
#   deploy/deploy.sh copy path/to/build-arm [config.txt]   copy the files only. Nothing starts by itself.
#   deploy/deploy.sh autostart                             also start vioh at every boot (bench step 8)
#   deploy/deploy.sh remove                                stop it and delete everything we put on the board
#
# "copy" touches only /data/nadirhold. The aircraft is shared: use "autostart" only for our own judged
# session and "remove" before handing the aircraft on.
# The config is only copied when given, so a config tuned on the board is not overwritten by accident.
set -e
APB=root@192.168.75.1
case "$1" in
copy)
    ssh $APB 'mkdir -p /data/nadirhold/logs; systemctl stop nadirhold 2>/dev/null || true'
    scp "$2/vioh" deploy/run.sh $APB:/data/nadirhold/
    [ -n "$3" ] && scp "$3" $APB:/data/nadirhold/config.txt
    ssh $APB 'chmod +x /data/nadirhold/vioh /data/nadirhold/run.sh'
    echo "Copied. To run it by hand:  ssh $APB /data/nadirhold/run.sh    (Ctrl-C stops it)"
    ;;
autostart)
    scp deploy/nadirhold.service $APB:/etc/systemd/system/nadirhold.service
    ssh $APB 'systemctl daemon-reload && systemctl enable nadirhold && systemctl restart nadirhold && sleep 2 && systemctl --no-pager status nadirhold | head -12'
    ;;
remove)
    ssh $APB 'systemctl disable --now nadirhold 2>/dev/null || true; rm -f /etc/systemd/system/nadirhold.service; systemctl daemon-reload; rm -rf /data/nadirhold /dev/shm/nadirhold_*'
    echo "Removed. Reboot the APB to put the IMU and barometer settings back to their defaults."
    ;;
*)
    sed -n '2,10p' "$0"
    exit 2
    ;;
esac
