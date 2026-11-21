"""Find which way is "forward" and "right" for the aircraft, and write it into the config (bench item C).

Run from the repo folder, with the aircraft plugged in and vioh running on it:

    python tools/find_axes.py

It asks you to carry the aircraft 0.5 m forward (the way the nose points), then 0.5 m to its right,
reads the position vioh reports each time, and sets forward_from_* and right_from_* in config.example
and in the config on the aircraft. Until this is done roll and pitch stay at 1500.
Hold the aircraft level, about 1 m above the floor, the whole time.
"""
import math
import re
import subprocess
import sys

BOARD = "root@192.168.75.1"
SSH = ["ssh", "-o", "BatchMode=yes", "-o", "ConnectTimeout=5", BOARD]
BOARD_CONFIG = "/data/nadirhold/config.txt"
LOCAL_CONFIG = "config.example"


def on_board(command):
    return subprocess.run(SSH + [command], capture_output=True, text=True, check=True).stdout


def position():
    """x, y in metres and the ready flag, from the status line vioh writes."""
    fields = dict(pair.split("=", 1) for pair in on_board("cat /dev/shm/nadirhold_status.txt").split() if "=" in pair)
    # x and y are in the axes the aircraft had when vioh started; turn them into the axes it has now.
    x, y, turn = float(fields["x"]), float(fields["y"]), math.radians(float(fields["turn"]))
    return (math.cos(turn) * x - math.sin(turn) * y, math.sin(turn) * x + math.cos(turn) * y, fields["ready"] == "1")


def measure(direction):
    input(f"\nHold the aircraft still and level, about 1 m up. Press Enter when ready... ")
    on_board("touch /dev/shm/nadirhold_zero; sleep 0.5")
    input(f"Now carry it about 0.5 m {direction}, keep it level, hold it still, then press Enter... ")
    x, y, ready = position()
    if not ready:
        sys.exit("vioh shows NOT READY, so this reading cannot be trusted. Start again.")
    length = math.hypot(x, y)
    print(f"   vioh saw x {x:+.2f} m, y {y:+.2f} m (length {length:.2f} m)")
    if length < 0.25:
        sys.exit("That is less than 0.25 m. Either the aircraft was not moved or tracking was lost. Start again.")
    return x, y


def nearest_axis(x, y):
    """The config wants one of +x, -x, +y, -y. Also say how far off that axis the move was."""
    if abs(x) >= abs(y):
        return (math.copysign(1, x), 0.0), math.degrees(math.atan2(abs(y), abs(x)))
    return (0.0, math.copysign(1, y)), math.degrees(math.atan2(abs(x), abs(y)))


def main():
    forward, forward_off = nearest_axis(*measure("FORWARD (the way the nose points)"))
    right, right_off = nearest_axis(*measure("to the aircraft's RIGHT"))
    print(f"\nforward is {forward} (the move was {forward_off:.0f} degrees off that axis)")
    print(f"right   is {right} (the move was {right_off:.0f} degrees off that axis)")
    if forward[0] * right[0] + forward[1] * right[1] != 0:
        sys.exit("Forward and right came out on the same axis. One of the moves was wrong. Start again.")
    if max(forward_off, right_off) > 30:
        sys.exit("A move was more than 30 degrees off an axis: too unclear to trust. Start again.")

    values = {"forward_from_x": forward[0], "forward_from_y": forward[1], "right_from_x": right[0], "right_from_y": right[1]}
    for name, value in values.items():
        print(f"{name} {value:+.0f}")
    if input("\nWrite these into config.example and the aircraft, and restart vioh there? (y/n) ").strip().lower() != "y":
        return

    text = open(LOCAL_CONFIG, encoding="utf-8", newline="").read()
    for name, value in values.items():
        text, count = re.subn(rf"(?m)^{name}\s+\S+", f"{name} {value:.0f}", text)
        if count != 1:
            sys.exit(f"{LOCAL_CONFIG}: expected exactly one line for {name}")
    open(LOCAL_CONFIG, "w", encoding="utf-8", newline="").write(text)
    edits = " ".join(f"-e 's/^{name} .*/{name} {value:.0f}/'" for name, value in values.items())
    print(on_board(f"sed -i {edits} {BOARD_CONFIG} && grep -E '^(forward|right)_from' {BOARD_CONFIG} && "
                   "(systemctl is-enabled nadirhold >/dev/null 2>&1 && systemctl restart nadirhold && echo 'vioh restarted' "
                   "|| echo 'autostart is off: restart vioh by hand')"))
    print("Done. With the engage switch on, roll and pitch now oppose a move (item C).")


if __name__ == "__main__":
    main()
