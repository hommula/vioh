import sys, csv
import matplotlib; matplotlib.use("Agg")
import matplotlib.pyplot as plt
rows = list(csv.DictReader(open(sys.argv[1])))
t = [float(r["t_s"]) for r in rows]
fig, ax = plt.subplots(5, 1, figsize=(12, 13), sharex=True)
ax[0].plot(t, [float(r["Ux"]) for r in rows], label="Ux"); ax[0].plot(t, [float(r["Uy"]) for r in rows], label="Uy"); ax[0].legend(); ax[0].set_ylabel("position / height")
ax[1].plot(t, [float(r["scale"]) for r in rows]); ax[1].set_ylabel("scale")
ax[2].plot(t, [float(r["Z0_m"]) for r in rows]); ax[2].set_ylabel("Z0 m"); ax[2].set_ylim(-3, 3)
ax[3].plot(t, [float(r["excitation"]) for r in rows]); ax[3].set_ylabel("excitation")
ax[4].plot(t, [int(r["inliers"]) for r in rows]); ax[4].plot(t, [int(r["keyframes"]) for r in rows]); ax[4].set_ylabel("inliers / keyframes")
plt.tight_layout(); plt.savefig(sys.argv[2], dpi=70)
