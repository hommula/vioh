"""Build docs/bonus-evidence.html: one page with the bonus evidence, pictures inside the file.

    python tools/evidence_page.py            (from the repo folder, after tools/evidence_plots.py)

Every number on the page is either measured or marked as simulated. Nothing is made up: the sources are
named in docs/bonus-evidence.md.
"""
import base64
import html

FOLDER = "docs/evidence"


def picture(name, caption):
    data = base64.b64encode(open(f"{FOLDER}/{name}", "rb").read()).decode("ascii")
    return f'<figure><img src="data:image/png;base64,{data}" alt="{html.escape(caption)}"><figcaption>{html.escape(caption)}</figcaption></figure>'


def table(header, rows):
    cells = "".join(f"<th>{html.escape(h)}</th>" for h in header)
    body = "".join("<tr>" + "".join(f"<td>{html.escape(c)}</td>" for c in row) + "</tr>" for row in rows)
    return f"<table><tr>{cells}</tr>{body}</table>"


page = f"""<!doctype html>
<html lang="en"><head><meta charset="utf-8"><meta name="viewport" content="width=device-width, initial-scale=1">
<title>Bonus evidence</title>
<style>
 body {{ font-family: Segoe UI, Arial, sans-serif; max-width: 980px; margin: 24px auto; padding: 0 16px; color: #1c1c1c; background: #fafafa; line-height: 1.45; }}
 h1 {{ font-size: 28px; }} h2 {{ margin-top: 36px; border-bottom: 2px solid #1f5fbf; padding-bottom: 4px; }}
 table {{ border-collapse: collapse; width: 100%; margin: 12px 0; background: #fff; }}
 th, td {{ border: 1px solid #cfcfcf; padding: 6px 10px; text-align: left; vertical-align: top; }} th {{ background: #e9eef7; }}
 figure {{ margin: 16px 0; }} img {{ max-width: 100%; border: 1px solid #cfcfcf; background: #fff; }}
 figcaption {{ font-size: 14px; color: #555; }} .tag {{ font-weight: 600; }} .note {{ background: #fff6dd; border-left: 4px solid #e0a800; padding: 8px 12px; }}
</style></head><body>
<h1>Bonus evidence: camera timing, rolling shutter, and onboard recovery</h1>
<p>GNSS-free position hold on the Orqa M2-AI. Each number says where it comes from:
<span class="tag">our aircraft</span>, <span class="tag">recording of this aircraft type by the organizers</span>, or <span class="tag">simulation</span>.</p>

<h2>1. Timing and rolling shutter</h2>
<p><b>The problem.</b> The camera reads a picture row by row, and its timestamp is not the moment the picture was taken.
The aircraft tilts in between. One degree of tilt moves the picture by about 6 pixels, which at 50 m height looks like 0.9 m of movement.</p>
<p><b>Our treatment</b> (<code>src/hold.hpp</code>, <code>levelPoints</code>): each tracked point becomes a viewing ray through the fisheye lens model;
the time of that point's own image row is computed (frame timestamp + camera delay - readout time x share of the picture below the row);
the gyro attitude at exactly that time (1000 samples a second, same clock) rotates the ray to what a level camera would see. Only then are pictures compared.</p>
{table(["What", "Value", "Source"], [
    ["Camera frame rate", "60.00 per second", "our aircraft, 180 frames"],
    ["Camera timestamp", "end of frame, monotonic clock; checked on every frame, the program stops otherwise", "our aircraft"],
    ["IMU rate and spacing", "1003 samples per second, 0.98 to 1.02 ms apart", "our aircraft, 12 s"],
    ["IMU clock", "set to the same monotonic clock as the camera", "our aircraft"],
    ["Gyro noise at rest / hand-held", "0.010 / 0.031 rad/s (3 axes, 0.5 s)", "our aircraft"],
    ["Camera delay against the gyro", "-14 ms", "organizers' recording, 2100 frame pairs"],
    ["Delay of top / middle / bottom third of the picture", "-14.33 / -14.26 / -13.85 ms", "organizers' recording"],
    ["Readout time, top to bottom", "about 0.9 ms", "organizers' recording (spread of the three bands)"],
    ["Picture motion left unexplained", "16.8 px with the gyro at the best delay, 24.9 px with an unrelated gyro signal", "organizers' recording"],
])}
{picture("evidence_camera_delay.png", "How the delay is measured: picture motion near the centre against gyro rotation, for every delay from -80 to +80 ms. The minimum is the camera delay.")}
<p><b>What a timestamp error costs</b> (simulation, 1 m height, tilting 5 degrees every 1.3 s):</p>
{table(["Timestamp error", "Worst position error at 1 m", "Extra error, scaled to 50 m"], [
    ["0 ms", "3.1 cm", "-"],
    ["14 ms (the measured delay, if ignored)", "3.4 cm", "about 0.15 m"],
    ["30 ms", "3.8 cm", "about 0.35 m"],
    ["60 ms", "5.6 cm", "about 1.25 m"],
])}
<p class="note"><b>Limits.</b> The delay was measured on a walk through a room, not on a pure rotation, so the minimum is clear but the numbers are not final.
On that slow recording the tracker scored the same with and without the correction (80.7 % of frames, 160 agreeing points): the correction matters at fast tilts, which that recording does not contain.
The scaling to 50 m is arithmetic on the simulation result, not a flight measurement.</p>

<h2>2. Onboard recovery</h2>
<p><b>The rule.</b> The judge covers the camera for 3 s in the middle of the loop. Back at the start mark the displayed position must be within 0.5 m of zero.</p>
<p><b>What happens while covered:</b> NOT READY in under 0.1 s (measured), roll, pitch and yaw 1500, position frozen.
<b>After uncovering:</b> tracking starts again from the frozen position. In addition, four times a second the picture is compared directly with a
"home picture" stored at the zero (400 corners with descriptors, corrected for lens, tilt and row time). When they match, the position is taken from that match alone.</p>
{table(["Step of the run on our aircraft", "What the log shows"], [
    ["Zero above the start mark", "0.00 m"],
    ["Walk out", "up to about 2.3 m shown"],
    ["Lens covered for 4.4 s in the middle of the loop", "tracked points below 30, NOT READY, position frozen"],
    ["Lens uncovered", "READY again about 1 s later"],
    ["Back above the start mark, held for 15 s", "0.15 to 0.34 m from zero, mean 0.22 m (limit 0.5 m)"],
])}
{picture("evidence_recovery_aircraft.png", "Our aircraft, hand-carried at about 1 m, props off. Raw log: docs/evidence/recovery_log_aircraft.txt.")}
{table(["Simulation: 3 s cover while the aircraft keeps moving", "Position error afterwards"], [
    ["Without the home-picture comparison", "up to 0.52 m"],
    ["With it", "0.02 m after 1 correction"],
])}
{picture("evidence_recovery_sim.png", "Simulation of the same case, with and without the home-picture comparison.")}
<p class="note"><b>Limits.</b> One run on the aircraft, carried by hand, "above the mark" judged by eye. In that run the home-picture comparison made no correction:
the 0.22 m is the tracker alone. The comparison is proven in simulation only.</p>
</body></html>
"""
open("docs/bonus-evidence.html", "w", encoding="utf-8").write(page)
print("wrote docs/bonus-evidence.html, %d kB" % (len(page) // 1024))
