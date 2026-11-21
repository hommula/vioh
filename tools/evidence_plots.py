"""Draw the evidence pictures for the two bonuses (timing / rolling shutter, and onboard recovery).

    python tools/evidence_plots.py sweep.csv sim_with.csv sim_without.csv out_folder

sweep.csv comes from `calib` (delay_s, rms_px). The two sim files come from `simtest`, once with the
recovery and once with it switched off. Needs matplotlib.
"""
import csv
import math
import sys

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

sweep_path, with_path, without_path, out = sys.argv[1:5]

# --- camera delay: how well the gyro explains the image motion, for every tried delay
rows = list(csv.DictReader(open(sweep_path)))
delay_ms = [float(r["delay_s"]) * 1000 for r in rows]
rms = [float(r["rms_px"]) for r in rows]
best = min(range(len(rms)), key=lambda i: rms[i])
plt.figure(figsize=(9, 4.5))
plt.plot(delay_ms, rms, color="#1f5fbf")
plt.axvline(delay_ms[best], color="#c0392b", linestyle="--")
plt.title(f"Camera timestamp against the gyro: best fit at {delay_ms[best]:+.0f} ms")
plt.xlabel("delay added to the camera timestamp (ms)")
plt.ylabel("image motion the gyro cannot explain (px rms)")
plt.grid(alpha=0.3)
plt.tight_layout()
plt.savefig(f"{out}/evidence_camera_delay.png", dpi=110)

# --- recovery: position error over time with the lens covered from 30 to 33 s
def error_series(path):
    t, e = [], []
    for r in csv.DictReader(open(path)):
        t.append(float(r["t_s"]))
        e.append(math.hypot(float(r["est_x"]) - float(r["true_x"]), float(r["est_y"]) - float(r["true_y"])))
    return t, e

plt.figure(figsize=(9, 4.5))
for path, label, colour in ((without_path, "without recovery", "#c0392b"), (with_path, "with recovery", "#1f5fbf")):
    t, e = error_series(path)
    plt.plot(t, e, label=label, color=colour)
plt.axvspan(30, 33, color="#888888", alpha=0.25, label="lens covered")
plt.axhline(0.5, color="#555555", linestyle=":", label="bonus limit 0.5 m")
plt.title("Simulation: position error when the lens is covered for 3 s while the aircraft keeps moving")
plt.xlabel("time (s)")
plt.ylabel("position error (m)")
plt.legend()
plt.grid(alpha=0.3)
plt.tight_layout()
plt.savefig(f"{out}/evidence_recovery_sim.png", dpi=110)
print("best delay %.1f ms, rms %.2f px; pictures written to %s" % (delay_ms[best], rms[best], out))
