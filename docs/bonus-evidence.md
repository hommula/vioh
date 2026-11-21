# Evidence for the two bonuses

Written 21 Nov. Each line says whether it was measured on our aircraft, measured on the organizers'
recording of this aircraft, or only simulated.

## 1. Timing and rolling shutter (up to +5, judges' panel)

### What the problem is

The camera reads a picture row by row (rolling shutter), and its timestamp is not the moment the
picture was taken. The aircraft tilts while that happens. A tilt of 1 degree moves the picture by about
6 pixels, which at 50 m height looks like 0.9 m of movement. So every point must be matched with the
attitude at the moment its own row was read.

### What the code does (`src/hold.hpp`, `levelPoints`)

1. Each tracked point is turned into a viewing ray with the fisheye lens model.
2. The time of the point's row is worked out: frame timestamp + camera delay - readout time x (share of
   the picture below that row). The driver stamps the END of the frame (checked on the board: flag
   "end of frame", monotonic clock).
3. The gyro attitude at exactly that time (the IMU runs at 1000 samples a second on the same clock) is
   used to rotate the ray to what a level camera would see.
4. Only then is the picture compared with the keyframe.

### Measured

| What | Value | Where from |
|---|---|---|
| Camera frame rate | 60.00 per second | Our aircraft, `v4l2-ctl`, 180 frames |
| Camera timestamp | end of frame, monotonic clock | Our aircraft, checked on every frame by `io::Camera` (it stops if not) |
| IMU rate and spacing | 1003 samples per second, 0.98 to 1.02 ms apart | Our aircraft, 12 s recording |
| IMU clock | set to the same monotonic clock as the camera | Our aircraft, `io::Imu` |
| Camera delay against the gyro | -14 ms (curve in `evidence/evidence_camera_delay.png`) | Organizers' recording of this aircraft, `calib`, 2100 frame pairs |
| Delay of the top, middle and bottom third of the picture | -14.33, -14.26, -13.85 ms | Same recording |
| Readout time top to bottom | 0.9 ms (from the 0.5 ms spread of the three bands) | Same recording |
| Gyro explains the picture motion | 16.8 px left over, against 24.9 px with an unrelated gyro signal | Same recording |

How the delay is measured (`src/calib.cpp`): the picture motion between two frames near the centre is
compared with the rotation the gyro saw in the same time, for every delay from -80 to +80 ms in 1 ms
steps. The delay with the smallest leftover is the camera delay. Doing the same for three horizontal
bands of the picture gives one delay per band; their spread from top to bottom is the readout time.

### Honest limits

- The recording used is a walk through a room, not a pure rotation. Walking also moves the picture,
  which is why 16.8 px is left over. The delay curve has a clear minimum, but the numbers are not final.
- The three band delays differ by only 0.5 ms. That says the readout is short for this camera mode; it
  does not prove the exact 0.9 ms.
- To replace these with a clean measurement from our aircraft: `sh tools/record_timing.sh` (25 s of
  rotating the aircraft by hand). It redraws the picture and prints the new numbers.

## 2. Onboard recovery (+5, bench step 3)

### What the rule asks

The judge covers the camera for 3 s in the middle of the loop and keeps walking. Back at the start
mark our position must be within 0.5 m of zero.

### What the code does (`src/hold.hpp`, `makeHome` and `relock`)

- At every zero (key `z`, the start mark) the program stores a "home picture": about 600 corners with
  descriptors (ORB), each already corrected for lens, tilt and rolling shutter.
- While covered, the estimate says NOT READY, keeps its last position, and sends 1500s.
- Four times a second the current picture is compared directly with the home picture. When the
  aircraft is back over the start mark the two match, and the position follows from that match alone.
  So what was missed while covered is gone, and so is the slow drift of the keyframe chain.
- A match is only accepted with at least 30 agreeing corners, at least half of all matches agreeing,
  and a zoom between 0.5 and 2. A carpet repeats itself, so a weak match is refused.

### Result

| Case | Position error after the cover | Where from |
|---|---|---|
| Without recovery | up to 0.52 m (fails the 0.5 m limit) | Simulation, 3 s cover while moving |
| With recovery | 0.02 m, after 1 correction | Same simulation |

Picture: `evidence/evidence_recovery_sim.png`.

### Measured on our aircraft (21 Nov, hand-carried at about 1 m, props off)

Picture: `evidence/evidence_recovery_aircraft.png`. Raw log: `evidence/recovery_log_aircraft.txt`.

| Step | What the log shows |
|---|---|
| Zero above the start mark | distance 0.00 m |
| Walk out | distance rises to about 2.3 m |
| Lens covered for 4.4 s in the middle of the loop | tracked points fall below 30, NOT READY, position frozen |
| Lens uncovered | READY again about 1 s later |
| Back above the start mark, held for 15 s | **0.15 to 0.34 m from zero, mean 0.22 m (limit 0.5 m)** |

In this run the direct comparison with the home picture made no correction (the counter did not
change). The 0.22 m is what the tracker alone gave after losing 4.4 s of the walk.

### Honest limits

- One run. The aircraft was carried by hand and "above the mark" was judged by eye.
- The home-picture correction works in the simulation; on the real floor it has not been seen to fire yet.
- It needs the start mark to be in view again. Far from the start mark nothing is corrected.
- To get the real evidence: put the new program on the aircraft, hold it at 1 m over a mark, press `z`,
  walk a loop, cover the lens for 3 s on the way while still walking, come back over the mark. The
  dashboard distance must come back to under 0.5 m. The status line has a counter `relocks` that goes
  up by one each time a correction is made:
  `ssh root@192.168.75.1 "tr ' ' '\n' < /dev/shm/nadirhold_status.txt | grep relocks"`
