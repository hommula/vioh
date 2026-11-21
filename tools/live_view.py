"""Live bench dashboard for vioh: the camera picture, READY state, position, graphs and a status table.

vioh (on the APB) writes one line of numbers to /dev/shm/nadirhold_status.txt about 10 times a second
and a small camera picture to /dev/shm/nadirhold_preview.pgm about 5 times a second. This script
fetches both over SSH and shows them in one window, laid out for the Stage 2 bench protocol.
Needs nothing but Python 3 (tkinter) and ssh.

    python tools/live_view.py                          # from the APB
    python tools/live_view.py <command ...>            # any command that prints the same stream
    python tools/live_view.py --quit-after 8 <...>     # close the window by itself after 8 seconds

The "Zero (z)" button and the z key ask vioh to zero the displayed position. The request is only
sent to the board when the stream comes from the APB (no command given); with any other command
the button only clears the path on the screen.
"""
import collections
import math
import subprocess
import sys
import threading
import time
import tkinter

BOARD = "root@192.168.75.1"
APB_COMMAND = ["ssh", BOARD,
               "while :; do cat /dev/shm/nadirhold_status.txt /dev/shm/nadirhold_preview.pgm 2>/dev/null; sleep 0.1; done"]
# BatchMode and ConnectTimeout: never wait for a password or for a board that is not there.
ZERO_COMMAND = ["ssh", "-o", "BatchMode=yes", "-o", "ConnectTimeout=3", BOARD, "touch /dev/shm/nadirhold_zero"]
ZERO_TIMEOUT_S = 8

ZOOM = 2              # the picture is 480 x 270; show it twice as large
STALE_AFTER_S = 2     # say so if nothing arrived for this long
GRAPH_S = 30          # the time graphs show this many seconds
TRAIL_S = 60          # the path plot shows this many seconds
GAP_S = 1.0           # do not join two points with a line if the data stopped for longer than this
READY_LIMIT_S = 60    # bench item E2: READY within this many seconds of power-on
REDRAW_IDLE_S = 0.5   # with no new data, still redraw this often so the "seconds ago" texts keep counting

# Window layout, in pixels. The laptop screen is 1707 x 1067 as Tk sees it (Windows enlarges the window
# by 1.5), so the window is kept narrower than the 1800 that a plain 1920 x 1080 screen would allow.
MARGIN = 8
PICTURE_W, PICTURE_H = 960, 540
BANNER_H = 84
TABLE_H = 284
NUMBERS_W = 348
PATH_SIZE = 340
GRAPHS_W = NUMBERS_W + MARGIN + PATH_SIZE
GRAPHS_H = 576
WINDOW_W = MARGIN + PICTURE_W + MARGIN + GRAPHS_W + MARGIN
WINDOW_H = MARGIN + BANNER_H + MARGIN + PICTURE_H + MARGIN + TABLE_H + MARGIN
STRIP_LEFT = 50       # room for the numbers on the left of a time graph
STRIP_RIGHT = 44      # room for the labels of the dashed lines on the right
ROW_H = 26            # one row of the status table

# Colours. Blue, orange and aqua tell lines apart; green, red and amber are kept for good / bad / attention.
PAGE = "#f9f9f7"
SURFACE = "#fcfcfb"
INK = "#0b0b0b"
INK_2 = "#52514e"
MUTED = "#898781"
GRID = "#e1e0d9"
AXIS = "#c3c2b7"
NEUTRAL = "#e1e0d9"
DARK = "#1a1a19"
BLUE = "#2a78d6"
ORANGE = "#eb6834"
AQUA = "#1baf7a"
GOOD = "#0ca30c"
GOOD_TEXT = "#006300"
GOOD_PALE = "#d6efd6"
CRITICAL = "#d03b3b"
CRITICAL_PALE = "#f6d9d9"
WARNING = "#fab219"

FONT = "Segoe UI"
MONO = "Consolas"     # same width for every character, so columns of numbers do not jump about

# The fields that are drawn over time. Each status line is kept as numbers for these names only.
PLOTTED_FIELDS = ["x", "y", "z", "roll", "pitch", "thr", "points", "ready", "gx", "gy", "gz"]

# The status table: (text on the left, fields shown on the right, how to show them).
CAMERA_ROWS = [
    ("frames per second", ["fps"], "{:.1f}"),
    ("ms of work per frame", ["ms"], "{:.1f}"),
    ("frames replaced", ["replaced"], "{:.0f}"),
    ("camera alive", ["cam"], "yes/NO"),
    ("tracking", ["tracking"], "yes/NO"),
    ("points tracked", ["points"], "{:.0f}"),
    ("height above ground Z (m)", ["Z"], "{:.2f}"),
    ("Z comes from", ["zsrc"], "text"),
]
IMU_ROWS = [
    ("samples per second", ["imu"], "{:.0f}"),
    ("gyro x y z (rad/s)", ["gx", "gy", "gz"], "{:+.3f}"),
    ("accel x y z (m/s2)", ["ax", "ay", "az"], "{:+.2f}"),
    ("tilt x (deg)", ["tiltx"], "{:+.2f}"),
    ("tilt y (deg)", ["tilty"], "{:+.2f}"),
    ("turn (deg)", ["turn"], "{:+.2f}"),
    ("barometer alive", ["baro"], "yes/NO"),
    ("barometer pressure (Pa)", ["pa"], "{:.0f}"),
]
LINK_ROWS = [
    ("flight controller link", ["fc"], "yes/NO"),
    ("armed", ["armed"], "yes/no"),
    ("rc throttle", ["rcthr"], "{:.0f}"),
    ("rc arm switch", ["rcarm"], "{:.0f}"),
    ("rc engage switch", ["rceng"], "{:.0f}"),
    ("board up (s)", ["up"], "{:.1f}"),
    ("program running (s)", ["run"], "{:.1f}"),
]

# Shared with the other threads. The reader thread fills the first three, the zero thread the last two.
latest_picture = None                    # (pgm bytes, counter, arrival time)
new_status_lines = collections.deque()   # (arrival time, text of the line without the '#')
skipped_lines = 0                        # lines of the stream that were neither a status line nor a picture
zero_message = ""                        # what happened to the last zero request
trail_start_time = 0.0                   # the path plot only shows points newer than this


def read_stream(stream):
    """Read status lines and binary PGM pictures (P5, maxval 255) from the stream, for ever.

    The two can come in any order: a status line after a status line, or pictures with no status line.
    Anything else is counted in skipped_lines (shown in the window) and reading carries on at the next line.
    """
    global latest_picture, skipped_lines
    picture_count = 0
    while True:
        line = stream.readline()
        if not line:
            return                                   # the source ended
        if line.startswith(b"#"):                    # vioh writes its numbers as a '#' line
            new_status_lines.append((time.monotonic(), line[1:].decode("ascii", "replace").strip()))
            continue
        if line.strip() != b"P5":
            skipped_lines += 1
            continue
        size = stream.readline().split()             # the rest of the picture header: "480 270" and "255"
        largest_value = stream.readline().strip()
        if len(size) != 2 or not size[0].isdigit() or not size[1].isdigit() or largest_value != b"255":
            skipped_lines += 1
            continue
        width, height = int(size[0]), int(size[1])
        pixels = stream.read(width * height)
        if len(pixels) < width * height:
            return                                   # the source ended in the middle of a picture
        picture_count += 1
        latest_picture = (b"P5\n%d %d\n255\n" % (width, height) + pixels, picture_count, time.monotonic())


def send_zero_request():
    """Ask vioh to zero the displayed position. Runs in its own thread so the window never waits for ssh."""
    global zero_message, trail_start_time
    try:
        result = subprocess.run(ZERO_COMMAND, stdin=subprocess.DEVNULL, capture_output=True, text=True,
                                timeout=ZERO_TIMEOUT_S)
    except subprocess.TimeoutExpired:
        zero_message = f"zero FAILED: ssh gave no answer in {ZERO_TIMEOUT_S} s"
        return
    if result.returncode != 0:
        zero_message = f"zero FAILED: ssh exit code {result.returncode}: {result.stderr.strip()}"
        return
    zero_message = "zero sent to the board"
    trail_start_time = time.monotonic()    # drop the points that arrived between the key press and the zero


def parse_status_line(text):
    """Turn 'up=312.44 run=41.2 ...' into {'up': '312.44', 'run': '41.2', ...}. Words without '=' are left out."""
    fields = {}
    for word in text.split():
        name, equals, value = word.partition("=")
        if equals:
            fields[name] = value
    return fields


def number(fields, name):
    """One field as a number, or None if it is missing or not a number that can be drawn (nan, inf, text)."""
    if name not in fields:
        return None
    try:
        value = float(fields[name])
    except ValueError:
        return None          # show() displays the text as the board sent it, so nothing is hidden
    if not math.isfinite(value):
        return None
    return value


def show(fields, name, kind):
    """The text to display for one field.

    kind is 'text', 'yes/no', 'yes/NO' (a 0 is a fault, drawn in red) or a pattern such as '{:.2f}'.
    """
    if name not in fields:
        return "--"
    text = fields[name]
    if kind == "text":
        return text
    if kind == "yes/no" or kind == "yes/NO":
        if text == "1":
            return "yes"
        if text == "0":
            return "NO" if kind == "yes/NO" else "no"
        return text
    value = number(fields, name)
    if value is None:
        return text          # for example "nan": show what the board sent
    return kind.format(value)


def unbroken_runs(samples, names):
    """Split the samples into runs that may be joined by a line.

    A run ends where one of the named values is missing or where the data stopped for more than GAP_S.
    """
    runs = []
    run = []
    for sample in samples:
        usable = all(sample[1][name] is not None for name in names)
        if run and (not usable or sample[0] - run[-1][0] > GAP_S):
            runs.append(run)
            run = []
        if usable:
            run.append(sample)
    if run:
        runs.append(run)
    return runs


def round_step(span, most_intervals):
    """The smallest round step (1, 2 or 5 times a power of ten) that cuts the span into at most this many parts."""
    power = math.floor(math.log10(span / most_intervals))
    while True:
        for multiple in (1, 2, 5):
            step = multiple * 10.0 ** power
            if span / step <= most_intervals + 1e-9:      # 1e-9: 0.6 / 0.2 is not exactly 3 in a computer
                return step
        power += 1


def draw_strip(canvas, top, height, title, series, fields, samples, now, must_show,
               centre=None, references=(), shade_ready=False, time_labels=True):
    """Draw one time graph across the canvas, in the band that starts at `top`.

    series       list of (name in the legend, field, colour, pattern for the current value)
    fields       the newest status line, for the current values in the legend
    samples      list of (arrival time, {field: number or None}), oldest first, no older than GRAPH_S
    must_show    (low, high) the axis always covers; it grows to fit the data
    centre       if given, the axis reaches the same distance above and below this value
    references   list of (value, label) drawn as dashed lines; the first ones get their label first
    shade_ready  colour the background green where ready=1 and red where ready=0
    """
    left = STRIP_LEFT
    right = GRAPHS_W - STRIP_RIGHT
    plot_top = top + 22
    plot_bottom = top + height - (16 if time_labels else 6)

    lowest, highest = must_show
    for sample_time, values in samples:
        for legend_name, field, colour, pattern in series:
            if values[field] is not None:
                lowest = min(lowest, values[field])
                highest = max(highest, values[field])
    if centre is not None:
        reach = max(highest - centre, centre - lowest)
        lowest, highest = centre - reach, centre + reach
    most_intervals = max(2, min(5, (plot_bottom - plot_top) // 15))    # numbers about 15 pixels apart or more
    step = round_step(highest - lowest, most_intervals)
    edge = 0.05 * (highest - lowest)        # a little room, so a line at the very top or bottom is not cut
    low = lowest - edge
    high = highest + edge

    def x_of(sample_time):
        return right - (now - sample_time) / GRAPH_S * (right - left)

    def y_of(value):
        return plot_bottom - (value - low) / (high - low) * (plot_bottom - plot_top)

    if shade_ready:
        for run in unbroken_runs(samples, ["ready"]):
            band_start = run[0]
            for sample in run[1:] + [None]:          # None: close the last band at the end of the run
                if sample is None or sample[1]["ready"] != band_start[1]["ready"]:
                    band_end_time = run[-1][0] if sample is None else sample[0]
                    colour = GOOD_PALE if band_start[1]["ready"] == 1 else CRITICAL_PALE
                    canvas.create_rectangle(x_of(band_start[0]), plot_top, x_of(band_end_time), plot_bottom,
                                            fill=colour, outline="")
                    band_start = sample

    # Grid: a numbered line at every step, and a line every 10 seconds.
    decimals = max(0, -math.floor(math.log10(step) + 1e-9))
    for index in range(math.ceil(lowest / step - 1e-9), math.floor(highest / step + 1e-9) + 1):
        value = index * step
        canvas.create_line(left, y_of(value), right, y_of(value), fill=GRID)
        canvas.create_text(left - 6, y_of(value), text=f"{value:.{decimals}f}", anchor="e",
                           font=(FONT, -11), fill=INK_2)
    for seconds_ago in range(0, GRAPH_S + 1, 10):
        x = right - seconds_ago / GRAPH_S * (right - left)
        canvas.create_line(x, plot_top, x, plot_bottom, fill=GRID)
        if time_labels:
            text = "now" if seconds_ago == 0 else f"-{seconds_ago} s"
            anchor = "ne" if seconds_ago == 0 else ("nw" if seconds_ago == GRAPH_S else "n")
            canvas.create_text(x, plot_bottom + 2, text=text, anchor=anchor, font=(FONT, -11), fill=INK_2)
    canvas.create_line(left, plot_top, left, plot_bottom, fill=AXIS)

    labelled_at = []          # a label is left out if it would sit on top of one already drawn
    for value, label in references:
        if low <= value <= high:
            y = y_of(value)
            canvas.create_line(left, y, right, y, fill=INK_2, dash=(5, 4))
            if all(abs(y - other) >= 10 for other in labelled_at):
                canvas.create_text(right + 4, y, text=label, anchor="w", font=(FONT, -11), fill=INK_2)
                labelled_at.append(y)

    for legend_name, field, colour, pattern in series:
        for run in unbroken_runs(samples, [field]):
            if len(run) >= 2:
                points = []
                for sample_time, values in run:
                    points += [x_of(sample_time), y_of(values[field])]
                canvas.create_line(points, fill=colour, width=2, joinstyle="round")

    # Title on the left; on the right one legend entry per line, with the current value.
    canvas.create_text(left, top + 11, text=title, anchor="w", font=(FONT, -13, "bold"), fill=INK)
    entries = [(legend_name + "  " + show(fields, field, pattern), colour, "line")
               for legend_name, field, colour, pattern in series]
    if shade_ready:
        entries += [("READY", GOOD_PALE, "block"), ("NOT READY", CRITICAL_PALE, "block")]
    for index, (text, colour, mark) in enumerate(entries):
        x = right - (len(entries) - index) * 130
        if mark == "line":
            canvas.create_line(x, top + 11, x + 16, top + 11, fill=colour, width=3)
        else:
            canvas.create_rectangle(x, top + 5, x + 16, top + 17, fill=colour, outline=MUTED)
        canvas.create_text(x + 22, top + 11, text=text, anchor="w", font=(FONT, -13), fill=INK)


def draw_graphs(canvas, fields, history, now):
    """Draw the time graphs of the last GRAPH_S seconds, one under the other."""
    canvas.delete("all")
    samples = [sample for sample in history if now - sample[0] <= GRAPH_S]
    draw_strip(canvas, 0, 100, "x and y position (m)",
               [("x", "x", BLUE, "{:+.3f}"), ("y", "y", ORANGE, "{:+.3f}")],
               fields, samples, now, must_show=(-0.5, 0.5), centre=0.0)
    # Bench item B3: a raise of 0.5 m must read between 0.35 and 0.65 m.
    draw_strip(canvas, 100, 120, "height change z (m)",
               [("z", "z", AQUA, "{:+.3f}")],
               fields, samples, now, must_show=(-0.2, 0.8),
               references=[(0.5, "0.50"), (0.35, "0.35"), (0.65, "0.65")])
    # Throttle sits far from 1500, so it gets its own small graph: on one axis with roll and pitch
    # it would squash them into a flat line and hide which way they move.
    draw_strip(canvas, 220, 86, "roll and pitch commands (µs)",
               [("roll", "roll", BLUE, "{:.0f}"), ("pitch", "pitch", ORANGE, "{:.0f}")],
               fields, samples, now, must_show=(1400, 1600), centre=1500.0,
               references=[(1500, "1500")], time_labels=False)
    draw_strip(canvas, 306, 76, "throttle command (µs)",
               [("thr", "thr", INK_2, "{:.0f}")],
               fields, samples, now, must_show=(1000, 2000))
    draw_strip(canvas, 382, 94, "tracked points and READY",
               [("points", "points", BLUE, "{:.0f}")],
               fields, samples, now, must_show=(0, 200), shade_ready=True)
    draw_strip(canvas, 476, 100, "gyro (rad/s)",
               [("x", "gx", BLUE, "{:+.3f}"), ("y", "gy", ORANGE, "{:+.3f}"), ("z", "gz", AQUA, "{:+.3f}")],
               fields, samples, now, must_show=(-0.5, 0.5), centre=0.0)


def draw_path(canvas, history, now):
    """Draw the path seen from above: x to the right, y up, zero in the middle, same scale both ways."""
    canvas.delete("all")
    centre_x = PATH_SIZE / 2
    centre_y = PATH_SIZE / 2 + 6
    radius_pixels = PATH_SIZE / 2 - 28

    samples = [sample for sample in history if sample[0] >= trail_start_time and now - sample[0] <= TRAIL_S]
    runs = unbroken_runs(samples, ["x", "y"])

    # Zoom out in steps of 0.5 m if the trail leaves the plot.
    furthest = 0.0
    for run in runs:
        for sample_time, values in run:
            furthest = max(furthest, abs(values["x"]), abs(values["y"]))
    half_range = max(1.0, math.ceil(furthest * 1.1 / 0.5) * 0.5)
    pixels_per_metre = radius_pixels / half_range

    # A ring every 0.5 m. If the estimate runs far away that would be hundreds of rings, so then
    # the spacing is doubled until there are at most 10; the text under the plot says which.
    ring_step = 0.5
    while half_range / ring_step > 10:
        ring_step *= 2
    label_every = math.ceil(24 / (ring_step * pixels_per_metre))
    for index in range(1, round(half_range / ring_step) + 1):
        ring_pixels = index * ring_step * pixels_per_metre
        canvas.create_oval(centre_x - ring_pixels, centre_y - ring_pixels,
                           centre_x + ring_pixels, centre_y + ring_pixels, outline=GRID)
        if index % label_every == 0:
            canvas.create_text(centre_x + ring_pixels - 2, centre_y + 3, text=f"{index * ring_step:g}",
                               anchor="ne", font=(FONT, -11), fill=INK_2)
    canvas.create_line(centre_x - radius_pixels, centre_y, centre_x + radius_pixels, centre_y, fill=AXIS)
    canvas.create_line(centre_x, centre_y - radius_pixels, centre_x, centre_y + radius_pixels, fill=AXIS)
    canvas.create_text(centre_x + radius_pixels + 3, centre_y, text="+x", anchor="w", font=(FONT, -12), fill=INK_2)
    canvas.create_text(centre_x, centre_y - radius_pixels - 2, text="+y", anchor="s", font=(FONT, -12), fill=INK_2)

    # Bench item B2: after the loop the position must be inside this circle.
    loop_pixels = 0.3 * pixels_per_metre
    canvas.create_oval(centre_x - loop_pixels, centre_y - loop_pixels, centre_x + loop_pixels, centre_y + loop_pixels,
                       outline=GOOD_TEXT, dash=(5, 4))

    for run in runs:
        if len(run) >= 2:
            points = []
            for sample_time, values in run:
                points += [centre_x + values["x"] * pixels_per_metre, centre_y - values["y"] * pixels_per_metre]
            canvas.create_line(points, fill=BLUE, width=2, joinstyle="round")
    if runs:
        newest = runs[-1][-1][1]
        dot_x = centre_x + newest["x"] * pixels_per_metre
        dot_y = centre_y - newest["y"] * pixels_per_metre
        canvas.create_oval(dot_x - 7, dot_y - 7, dot_x + 7, dot_y + 7, fill=BLUE, outline=SURFACE, width=2)

    canvas.create_text(6, 10, text=f"path seen from above, last {TRAIL_S} s", anchor="w",
                       font=(FONT, -13, "bold"), fill=INK)
    canvas.create_text(6, PATH_SIZE - 9, text=f"rings every {ring_step:g} m", anchor="w", font=(FONT, -12), fill=INK_2)
    canvas.create_line(PATH_SIZE - 142, PATH_SIZE - 9, PATH_SIZE - 118, PATH_SIZE - 9, fill=GOOD_TEXT, dash=(5, 4))
    canvas.create_text(PATH_SIZE - 6, PATH_SIZE - 9, text="0.3 m around zero", anchor="e",
                       font=(FONT, -12), fill=INK_2)


def build_table_column(table, x, width, rows_top, rows):
    """Make one column of the status table. Returns (the column's frame, one value label per row)."""
    column = tkinter.Frame(table, bg=PAGE)
    column.place(x=x, y=0, width=width, height=TABLE_H)
    value_labels = []
    for index, (text, names, kind) in enumerate(rows):
        y = rows_top + index * ROW_H
        tkinter.Label(column, text=text, font=(FONT, -15), bg=PAGE, fg=INK_2, anchor="w").place(
            x=0, y=y, width=width, height=ROW_H)
        value_label = tkinter.Label(column, text="--", font=(MONO, -15), bg=PAGE, fg=INK, anchor="e")
        value_label.place(relx=1.0, y=y, height=ROW_H, anchor="ne")
        value_labels.append(value_label)
    return column, value_labels


def main():
    arguments = sys.argv[1:]
    quit_after_s = None
    if arguments and arguments[0] == "--quit-after":
        if len(arguments) < 2:
            sys.exit("usage: live_view.py [--quit-after SECONDS] [command ...]")
        quit_after_s = float(arguments[1])
        arguments = arguments[2:]
    command = arguments or APB_COMMAND
    zero_reaches_board = not arguments      # with a test source there is no board to send the request to

    source = subprocess.Popen(command, stdout=subprocess.PIPE)
    threading.Thread(target=read_stream, args=(source.stdout,), daemon=True).start()

    window = tkinter.Tk()
    window.title("vioh bench dashboard: waiting for the first data")
    window.geometry(f"{WINDOW_W}x{WINDOW_H}+0+0")     # top left corner: anywhere lower it would not fit the screen
    window.resizable(False, False)
    window.configure(bg=PAGE)
    right_x = MARGIN + PICTURE_W + MARGIN

    # Top left: READY banner, ENGAGED / pilot, and two lines of timing for bench items E1 and E2.
    banner = tkinter.Frame(window, bg=PAGE)
    banner.place(x=MARGIN, y=MARGIN, width=PICTURE_W, height=BANNER_H)
    ready_label = tkinter.Label(banner, text="WAITING", font=(FONT, -44, "bold"), bg=NEUTRAL, fg=INK)
    ready_label.place(x=0, y=0, width=290, height=BANNER_H)
    engaged_label = tkinter.Label(banner, text="--", font=(FONT, -30, "bold"), bg=NEUTRAL, fg=INK)
    engaged_label.place(x=298, y=0, width=170, height=BANNER_H)
    changed_label = tkinter.Label(banner, text="", font=(FONT, -17), bg=PAGE, fg=INK, anchor="w")
    changed_label.place(x=480, y=1, width=480, height=27)
    first_ready_label = tkinter.Label(banner, text="", font=(FONT, -17), bg=PAGE, fg=INK, anchor="w")
    first_ready_label.place(x=480, y=29, width=480, height=27)
    program_start_label = tkinter.Label(banner, text="", font=(FONT, -14), bg=PAGE, fg=INK_2, anchor="w")
    program_start_label.place(x=480, y=57, width=480, height=26)

    # Left, under the banner: the camera picture.
    picture_label = tkinter.Label(window, text="waiting for the first picture...", font=(FONT, -28, "bold"),
                                  bg=DARK, fg=WARNING, compound="center")
    picture_label.place(x=MARGIN, y=MARGIN + BANNER_H + MARGIN, width=PICTURE_W, height=PICTURE_H)

    # Bottom left: the status table (bench item A), three columns.
    table = tkinter.Frame(window, bg=PAGE)
    table.place(x=MARGIN, y=MARGIN + BANNER_H + MARGIN + PICTURE_H + MARGIN, width=PICTURE_W, height=TABLE_H)
    camera_column, camera_values = build_table_column(table, 0, 296, 42, CAMERA_ROWS)
    camera_live_label = tkinter.Label(camera_column, text="camera --", font=(FONT, -22, "bold"), bg=NEUTRAL, fg=INK)
    camera_live_label.place(x=0, y=0, width=296, height=36)
    imu_column, imu_values = build_table_column(table, 304, 344, 42, IMU_ROWS)
    imu_live_label = tkinter.Label(imu_column, text="IMU --", font=(FONT, -22, "bold"), bg=NEUTRAL, fg=INK)
    imu_live_label.place(x=0, y=0, width=344, height=36)
    link_column, link_values = build_table_column(table, 656, 304, 82, LINK_ROWS)
    tkinter.Label(link_column, text="stick commands we send (µs, 1500 = centre)", font=(FONT, -13), bg=PAGE,
                  fg=INK_2, anchor="w").place(x=0, y=0, width=304, height=18)
    commands_label = tkinter.Label(link_column, text="", font=(MONO, -20, "bold"), bg=PAGE, fg=INK,
                                   anchor="w", justify="left")
    commands_label.place(x=0, y=20, width=304, height=54)
    skipped_label = tkinter.Label(camera_column, text="", font=(FONT, -15), bg=PAGE, fg=CRITICAL, anchor="w")
    skipped_label.place(x=0, y=42 + len(CAMERA_ROWS) * ROW_H, width=296, height=ROW_H)

    # Top right: the big numbers the judge reads for bench items B1, B2 and B3, and the zero button.
    numbers = tkinter.Frame(window, bg=PAGE)
    numbers.place(x=right_x, y=MARGIN, width=NUMBERS_W, height=PATH_SIZE)
    tkinter.Label(numbers, text="horizontal distance from zero", font=(FONT, -15), bg=PAGE, fg=INK_2,
                  anchor="w").place(x=0, y=0, width=NUMBERS_W, height=20)
    distance_label = tkinter.Label(numbers, text="-- m", font=(FONT, -80, "bold"), bg=PAGE, fg=INK, anchor="w")
    distance_label.place(x=0, y=20, width=NUMBERS_W, height=92)
    big_labels = {}
    for index, (field, text) in enumerate([("x", "x"), ("y", "y"), ("z", "height change z")]):
        y = 116 + index * 50
        tkinter.Label(numbers, text=text, font=(FONT, -17), bg=PAGE, fg=INK_2, anchor="w").place(
            x=0, y=y, width=130, height=50)
        big_labels[field] = tkinter.Label(numbers, text="-- m", font=(FONT, -40, "bold"), bg=PAGE, fg=INK, anchor="e")
        big_labels[field].place(x=130, y=y, width=NUMBERS_W - 138, height=50)

    def request_zero():
        global zero_message, trail_start_time
        trail_start_time = time.monotonic()
        if zero_reaches_board:
            zero_message = "zero: sending..."
            threading.Thread(target=send_zero_request, daemon=True).start()
        else:
            zero_message = "path cleared. Zero NOT sent to the board: this is not the APB stream."

    zero_button = tkinter.Button(numbers, text="Zero (z)", font=(FONT, -16, "bold"), command=request_zero)
    zero_button.place(x=0, y=272, width=120, height=36)
    zero_label = tkinter.Label(numbers, text="", font=(FONT, -13), bg=PAGE, fg=INK_2, anchor="w", justify="left",
                               wraplength=NUMBERS_W - 136)
    zero_label.place(x=128, y=268, width=NUMBERS_W - 128, height=68)
    window.bind("<Key-z>", lambda event: request_zero())
    window.bind("<Key-Z>", lambda event: request_zero())     # the same key with caps lock on

    path_canvas = tkinter.Canvas(window, bg=SURFACE, highlightthickness=0)
    path_canvas.place(x=right_x + NUMBERS_W + MARGIN, y=MARGIN, width=PATH_SIZE, height=PATH_SIZE)

    # Right, under those: the time graphs.
    graphs_canvas = tkinter.Canvas(window, bg=SURFACE, highlightthickness=0)
    graphs_canvas.place(x=right_x, y=MARGIN + PATH_SIZE + MARGIN, width=GRAPHS_W, height=GRAPHS_H)

    history = collections.deque()     # (arrival time, {field: number or None}) for the last TRAIL_S seconds
    state = {
        "fields": {},                 # the newest status line, name -> text
        "status_text": "",            # the newest status line as it came (shown when it has no numbers)
        "status_time": None,          # when it arrived
        "ready": None,                # the last value of ready that was seen
        "ready_changed_time": None,   # when ready last changed
        "watching_since": time.monotonic(),
        "picture_counter": 0,
        "image": None,
        "last_draw_time": 0.0,
    }

    def take_new_status_lines():
        """Move what the reader thread collected into state and history. Returns True if there was anything."""
        got_any = False
        while new_status_lines:
            arrival_time, text = new_status_lines.popleft()
            got_any = True
            fields = parse_status_line(text)
            state["fields"] = fields
            state["status_text"] = text
            state["status_time"] = arrival_time
            if "ready" in fields:
                if state["ready"] is not None and fields["ready"] != state["ready"]:
                    state["ready_changed_time"] = arrival_time
                state["ready"] = fields["ready"]
            if fields:
                history.append((arrival_time, {name: number(fields, name) for name in PLOTTED_FIELDS}))
        return got_any

    def set_ready_label(text, text_height, background, foreground):
        ready_label.configure(text=text, font=(FONT, -text_height, "bold"), bg=background, fg=foreground)

    def draw_banner(now, fields, stale):
        if state["status_time"] is None:
            set_ready_label("WAITING", 44, NEUTRAL, INK)
            changed_label.configure(text="waiting for the first status line...")
        elif stale:
            silent_for = now - state["status_time"]
            if source.poll() is not None:
                set_ready_label("SOURCE ENDED", 32, DARK, WARNING)
                changed_label.configure(text=f"no status line for {silent_for:.0f} s: the source command ended")
            else:
                set_ready_label("NO DATA", 44, DARK, WARNING)
                changed_label.configure(text=f"no status line for {silent_for:.0f} s (is vioh running?)")
        elif not fields:
            # Record mode sends a line of text with no numbers: show the text.
            set_ready_label("NO ESTIMATE", 36, NEUTRAL, INK)
            changed_label.configure(text=state["status_text"])
        else:
            if fields.get("ready") == "1":
                set_ready_label("READY", 44, GOOD, "white")
            elif fields.get("ready") == "0":
                set_ready_label("NOT READY", 44, CRITICAL, "white")
            else:
                set_ready_label("ready " + show(fields, "ready", "text"), 44, NEUTRAL, INK)
            if state["ready_changed_time"] is None:
                changed_label.configure(text=f"no READY change seen in the last {now - state['watching_since']:.0f} s")
            else:
                changed_label.configure(text=f"READY state changed {now - state['ready_changed_time']:.1f} s ago")

        if stale or "engaged" not in fields:
            engaged_label.configure(text="--", bg=NEUTRAL, fg=INK)
        elif fields["engaged"] == "1":
            engaged_label.configure(text="ENGAGED", bg=WARNING, fg=INK)
        elif fields["engaged"] == "0":
            engaged_label.configure(text="pilot", bg=NEUTRAL, fg=INK)
        else:
            engaged_label.configure(text=fields["engaged"], bg=NEUTRAL, fg=INK)

        # Bench item E2: after a power cycle the program must be READY within READY_LIMIT_S.
        first_ready = number(fields, "first_ready")
        board_up = number(fields, "up")
        program_run = number(fields, "run")
        late = False
        if first_ready is None:
            text = "first READY: " + show(fields, "first_ready", "text")
        elif first_ready < 0:
            text = "not READY yet, " + show(fields, "up", "{:.1f}") + " s after power-on"
            late = board_up is not None and board_up > READY_LIMIT_S
        else:
            text = f"first READY {first_ready:.1f} s after power-on"
            late = first_ready > READY_LIMIT_S
        if late:
            first_ready_label.configure(text=" " + text + f"   OVER {READY_LIMIT_S} s", bg=CRITICAL, fg="white")
        else:
            first_ready_label.configure(text=text, bg=PAGE, fg=INK)

        # The line above counts from power-on. If the program was started by hand long after power-on
        # it is red for no fault of the program, so also say when the program started.
        if board_up is None or program_run is None:
            program_start_label.configure(text="")
        else:
            program_start = board_up - program_run
            text = f"program started {program_start:.1f} s after power-on"
            if first_ready is not None and first_ready >= 0:
                text += f"; first READY {first_ready - program_start:.1f} s later"
            program_start_label.configure(text=text)

    def draw_numbers(fields, stale):
        colour = MUTED if stale else INK      # grey: these are old numbers
        x = number(fields, "x")
        y = number(fields, "y")
        if x is not None and y is not None:
            distance_label.configure(text=f"{math.hypot(x, y):.2f} m", fg=colour)
        else:
            distance_label.configure(text="-- m", fg=colour)
        for field, label in big_labels.items():
            label.configure(text=show(fields, field, "{:+.2f}") + " m", fg=colour)
        zero_label.configure(text=zero_message, fg=CRITICAL if "FAILED" in zero_message else INK_2)

    def draw_live_label(label, name, live):
        """live is True, False, or None when the fields that decide it are missing."""
        if live is None:
            label.configure(text=name + " --", bg=NEUTRAL, fg=INK)
        elif live:
            label.configure(text=name + " LIVE", bg=GOOD, fg="white")
        else:
            label.configure(text=name + " NOT LIVE", bg=CRITICAL, fg="white")

    def draw_table(fields, stale):
        frames_per_second = number(fields, "fps")
        imu_per_second = number(fields, "imu")
        camera_live = None
        imu_live = None
        if stale:
            camera_live = False
            imu_live = False
        else:
            if frames_per_second is not None and "cam" in fields:
                camera_live = frames_per_second > 0 and fields["cam"] == "1"
            if imu_per_second is not None:
                imu_live = imu_per_second > 0
        draw_live_label(camera_live_label, "camera", camera_live)
        draw_live_label(imu_live_label, "IMU", imu_live)

        for rows, value_labels in [(CAMERA_ROWS, camera_values), (IMU_ROWS, imu_values), (LINK_ROWS, link_values)]:
            for (text, names, kind), value_label in zip(rows, value_labels):
                shown = [show(fields, name, kind) for name in names]
                value_label.configure(text="  ".join(shown), fg=CRITICAL if "NO" in shown else INK)
        commands_label.configure(text="roll {:>4}  pitch {:>4}\nyaw  {:>4}  thr   {:>4}".format(
            show(fields, "roll", "{:.0f}"), show(fields, "pitch", "{:.0f}"),
            show(fields, "yaw", "{:.0f}"), show(fields, "thr", "{:.0f}")))
        if skipped_lines > 0:
            skipped_label.configure(text=f"unreadable lines in the stream: {skipped_lines}")

    def draw_picture(now):
        if latest_picture is None:
            return
        pgm, counter, arrival_time = latest_picture
        if counter != state["picture_counter"]:
            state["picture_counter"] = counter
            state["image"] = tkinter.PhotoImage(data=pgm, format="PPM").zoom(ZOOM)   # keep a reference
            picture_label.configure(image=state["image"], text="")
        elif now - arrival_time > STALE_AFTER_S:
            picture_label.configure(text=f"NO NEW PICTURE for {now - arrival_time:.0f} s")

    def refresh():
        now = time.monotonic()
        got_new_status = take_new_status_lines()
        while history and now - history[0][0] > TRAIL_S:
            history.popleft()
        draw_picture(now)
        if got_new_status or now - state["last_draw_time"] >= REDRAW_IDLE_S:
            state["last_draw_time"] = now
            fields = state["fields"]
            stale = state["status_time"] is None or now - state["status_time"] > STALE_AFTER_S
            draw_banner(now, fields, stale)
            draw_numbers(fields, stale)
            draw_table(fields, stale)
            draw_path(path_canvas, history, now)
            draw_graphs(graphs_canvas, fields, history, now)
            if state["status_time"] is None:
                window.title("vioh bench dashboard: waiting for the first data")
            elif not stale:
                window.title("vioh bench dashboard: live")
            elif source.poll() is not None:
                window.title("vioh bench dashboard: SOURCE ENDED")
            else:
                window.title("vioh bench dashboard: NO DATA (is vioh running?)")
        window.after(50, refresh)

    if quit_after_s is not None:
        window.after(int(quit_after_s * 1000), window.destroy)
    refresh()
    window.mainloop()
    source.kill()


if __name__ == "__main__":
    main()
