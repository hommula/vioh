"""Wait for one carried move and print it: hold still, move at least 0.3 m, hold still again.

    python tools/watch_move.py

Used to find the forward and right directions without pressing keys: the aircraft is zeroed when it
is first held still, and the position is printed when it is held still again after a move.
"""
import math
import subprocess
import sys
import time

BOARD = "root@192.168.75.1"
STREAM = ["ssh", "-o", "BatchMode=yes", "-o", "ConnectTimeout=5", BOARD,
          "while :; do cat /dev/shm/nadirhold_status.txt; sleep 0.1; done"]
STILL_M = 0.04        # moved less than this over STILL_S seconds = held still
STILL_S = 1.5
MOVE_M = 0.30
GIVE_UP_S = 120


def main():
    source = subprocess.Popen(STREAM, stdout=subprocess.PIPE, text=True)
    recent = []                 # (time, x, y) of the last STILL_S seconds
    zeroed_at = None
    start = time.monotonic()
    for line in source.stdout:
        fields = dict(pair.split("=", 1) for pair in line.split() if "=" in pair)
        if "x" not in fields:
            continue
        now = time.monotonic()
        if now - start > GIVE_UP_S:
            print("gave up: no complete move in", GIVE_UP_S, "s")
            break
        x, y = float(fields["x"]), float(fields["y"])
        if fields["ready"] != "1":
            recent = []
            continue
        recent = [r for r in recent if now - r[0] <= STILL_S] + [(now, x, y)]
        covered = recent[-1][0] - recent[0][0] >= STILL_S * 0.8
        still = covered and max(math.hypot(r[1] - x, r[2] - y) for r in recent) < STILL_M
        if zeroed_at is None:
            # 1 m of assumed height: on the floor the camera height reads far below zero.
            if still and abs(float(fields["tiltx"])) < 15 and abs(float(fields["tilty"])) < 15:
                subprocess.run(["ssh", "-o", "BatchMode=yes", BOARD, "touch /dev/shm/nadirhold_zero"], check=True)
                zeroed_at = now
                recent = []
                print("held still: position zeroed, waiting for the move", flush=True)
        elif now - zeroed_at > 1.0 and still and math.hypot(x, y) >= MOVE_M:
            # x and y are in the axes the aircraft had when vioh started. Turn them into the axes it has
            # now (the same rotation the controller applies), because forward and right belong to the aircraft.
            turn = math.radians(float(fields["turn"]))
            bx = math.cos(turn) * x - math.sin(turn) * y
            by = math.sin(turn) * x + math.cos(turn) * y
            print(f"MOVE in aircraft axes: x {bx:+.3f} y {by:+.3f}, length {math.hypot(x, y):.2f} m, "
                  f"direction {math.degrees(math.atan2(by, bx)):+.0f} deg (turn since start {fields['turn']} deg, points {fields['points']})")
            break
    source.kill()


if __name__ == "__main__":
    main()
