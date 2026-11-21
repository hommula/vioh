# vioh: GNSS-free position hold for the Orqa M2-AI

One C++ program (`vioh`) on the APB. It reads the nadir camera, the APB IMU and the APB barometer,
estimates where the aircraft is relative to the engage point, and sends stick values to Betaflight
with `MSP_SET_RAW_RC` at 50 Hz.

## Status: what is proven and what is not

| Claim | How it was checked |
|---|---|
| Geometry and signs of the estimator are right | **Ran** `simtest`: a simulated fisheye camera over a flat floor with a gyro bias. Worst error 8 cm in position and 6 cm in height at 1 m; accelerometer height fit 9 % low. The height error grows slowly with each new keyframe; cause not fully found. |
| The lens is a fisheye | **Measured**, weakly: straightening floor lines in the recording fits an equal-angle fisheye with a focal length near 500 px (960 x 540 image). That is 8 % off a distortion-free camera at 250 px from the centre and over 40 % at the corners. |
| The fisheye is handled | **Ran** `simtest` with a simulated fisheye. With the right lens numbers it passes; with wrong ones the scale error was 22 %. The real lens numbers are **placeholders** until a checkerboard calibration. |
| Live view on the laptop | **Ran** `tools/live_view.py` against recorded frames. The board side (saving the preview picture) is **not run on the board**. |
| Command rules of Stage 2 (safe values from the first tick, 1500s when not ready, commands oppose the move) | **Ran** `selftest`: scripted scenarios on the decision code. |
| MSP bytes | **Ran** `selftest` against the frame format read from Betaflight 2025.12.5 source. |
| The ARM build works | **Ran** `selftest` and `simtest` as aarch64 binaries under emulation. `vioh` needs only libc and libm. |
| It works on the real recording | **Ran** `replay`: it locks on where a flat floor is in view (about 195 of 200 points). Most of that recording is a walk past chairs and people, not a flat floor, so it proves the pipeline runs and nothing about accuracy. |
| Camera, IMU, barometer and serial code | **Only read** against the NXP kernel source. **Never run on the board.** |
| Calibration numbers in `config.example` | From one unsuitable recording. **Placeholders.** |
| Controller gains | **Assumed** from physics. Never flown. |
| Metric scale on the bench from the accelerometer | Works in simulation. On the real recording it gives implausibly small heights. **Unproven.** |

## How it works (short)

1. **Estimator** (`src/hold.hpp`). Corners are tracked from a keyframe to each new frame (Lucas-Kanade).
   Every point is first turned into a viewing ray with the fisheye lens model, then rotated to what a
   level, distortion-free camera would see, using the gyro attitude at the time its image row was
   read (this is the lens, rolling-shutter and tilt correction). A 4-number fit
   (shift, rotation, zoom) then gives the position in units of height. While the keyframe stays in
   view there is no drift. When the view moves too far a new keyframe is chained on.
2. **Height** (`src/control.hpp`). APB barometer change, smoothed with the accelerometer. Height above
   ground for the scale is: barometer since arming (in flight), else an accelerometer fit (bench, after
   the aircraft has been moved), else 1 m (the bench protocol height).
3. **Commands** (`Pilot` in `src/control.hpp`). Switch off: 1500/1500/1500 and the pilot's own throttle.
   Switch on and ready: hold the position and height seen at engage. Switch on and not ready:
   1500/1500/1500 and the height loop only.
4. **GNSS**: the program asks the flight controller for `MSP_RC` only. It never requests GPS,
   `MSP_ALTITUDE` or `MSP_ATTITUDE` (the last two can contain GPS).

## Layout

| Path | What |
|---|---|
| `src/hold.hpp` | Estimator. No devices. |
| `src/control.hpp` | Height filter, controller, command rules. No devices. |
| `src/msp.hpp` | MSP v1 frames. |
| `src/io.hpp` | Camera (V4L2), IMU and barometer (IIO), serial. Board facts and a 10-step board checklist are in its header comment. |
| `src/main.cpp` | The onboard program, and `record` mode. |
| `src/calib.cpp`, `replay.cpp`, `simtest.cpp`, `selftest.cpp` | Laptop tools and tests. |
| `tools/mcap2log.py` | Converts an MCAP recording to the flat log the tools read. |
| `tools/live_view.py` | Shows the nadir camera in a window on the laptop while `vioh` runs (normal or record mode). Python 3 only. |
| `tools/board_check.sh` | The board checklist as one command. |
| `config.example` | Every setting, with where each number came from. |
| `deploy/` | systemd unit and copy script. |
| `docs/research.md`, `docs/blackbox-findings.md` | Research with sources. |

## Build and test (Docker, from Git Bash on the laptop)

```sh
# laptop tools and tests (image: vioh-dev = hold-onboard-dev + lz4 + mcap)
MSYS_NO_PATHCONV=1 docker run --rm -v "D:/MISC/new/repo:/src" -v "D:/MISC/new/data:/data" vioh-dev bash -c \
  "cmake -S /src -B /data/build-x86 -G Ninja && cmake --build /data/build-x86 && \
   /data/build-x86/selftest /src/config.example && /data/build-x86/simtest /src/config.example /data/sim.csv"

# the board binary (image: hold-onboard-arm)
MSYS_NO_PATHCONV=1 docker run --rm -v "D:/MISC/new/repo:/src" -v "D:/MISC/new/data:/data" $(docker images -q hold-onboard-arm) bash -c \
  "cmake -S /src -B /data/build-arm -G Ninja -DCMAKE_TOOLCHAIN_FILE=/src/toolchain-aarch64.cmake && cmake --build /data/build-arm"

# a recording: convert, replay, plot
MSYS_NO_PATHCONV=1 docker run --rm -v "D:/MISC/new:/w" vioh-dev bash -c \
  "python /w/repo/tools/mcap2log.py '/w/real footage/20261121T160159Z.mcap' /w/data/rec.vlog && \
   /w/data/build-x86/replay /w/repo/config.example /w/data/rec.vlog /w/data/replay.csv && \
   python /w/repo/tools/plot_replay.py /w/data/replay.csv /w/data/replay.png"
```

## First hour with the aircraft, props off, in this order

1. **Board checklist.** `ssh root@192.168.75.1 sh -s < tools/board_check.sh 2>&1 | tee board_check.txt`.
   Watch item 4: starting the camera may change the organizers' sensor format. If it does, stop and ask them.
2. **Deploy.** `deploy/deploy.sh copy /d/MISC/new/data/build-arm config.example`, then
   `ssh root@192.168.75.1 /data/nadirhold/run.sh` to run it by hand and see the status line (Ctrl-C stops it).
   This puts files in `/data/nadirhold` only; nothing starts by itself. See "Sharing the aircraft" below.
   In a second terminal on the laptop, `python tools/live_view.py` shows the camera live.
   This alone is bench item A (camera and IMU live in our process).
3. **Calibrate.** Record 30 s while only ROTATING the aircraft (roll and pitch wiggles, no walking),
   camera pointed at something at least 3 m away:
   `/data/nadirhold/vioh /data/nadirhold/config.txt record /data/nadirhold/cal.vlog 30 2`. Copy it to the laptop, run
   `calib cal.vlog 250 sweep.csv`, and paste its "config lines" into the config. Keep `sweep.csv`: it is
   the measured evidence for the timing and rolling-shutter bonus.
4. **Find the axis signs.** With the program running, press Enter, carry the aircraft 0.5 m forward
   (nose direction) and read which of x / y changed and its sign; then 0.5 m right. Set
   `forward_from_*` and `right_from_*` so that forward and right both read positive. Until then roll
   and pitch stay at 1500.
5. **Rehearse the bench protocol** (Stage 2 section 05) with a tape measure, in order, and write down
   every reading. Step 2 is the first real test of the metric scale.
6. **Only then** ask for a flight. Start with a low hover and expect to change the gains.

## Sharing the aircraft: what we change on it

| Action | What changes on the aircraft | Lasts until |
|---|---|---|
| `tools/board_check.sh` | Nothing on purpose. It streams 120 camera frames, which may change the sensor format (that is what it tests). | Reboot |
| `deploy.sh copy` | Files in `/data/nadirhold` | `deploy.sh remove` |
| `run.sh` (vioh running) | Holds the camera, IMU, barometer and the FC serial port, so no other program can use them. Sends stick commands to the FC all the time; **the FC obeys them whenever the engage switch is on.** Sets IMU rate 1000 Hz, IMU clock monotonic, barometer 32 Hz. | Ctrl-C frees the devices and stops the commands (the FC keeps the last one). Sensor settings: reboot. |
| `vioh ... record` | A recording file in `/data/nadirhold` (about 15 MB per second at every 2nd frame) | Delete it |
| `deploy.sh autostart` | A systemd unit in `/etc/systemd/system`: vioh runs at every boot | `deploy.sh remove` |
| Never | Flight controller settings or firmware, device tree, kernel, GPIO, the organizers' services | |

Before handing the aircraft on: Ctrl-C, `deploy/deploy.sh remove` if autostart was on, and reboot the APB.

## Blanks: values that must come from our aircraft

| Blank | Where | How to fill it |
|---|---|---|
| `tilt_to_px_*`, `cam_delay_s`, `rolling_shutter_readout_s` | config | Step 3 above |
| `forward_from_*`, `right_from_*` | config | Step 4 above |
| `pos_*`, `height_*` gains, `max_tilt_deg` | config | Flight tuning. Starting values are physics, not measurements. |
| Barometer rate and noise | `deploy/nadirhold.service` | 32 Hz and 16x are from the data sheet budget. Log a still aircraft and look. |
| `roi_radius_px`, image size | config, `main.cpp` | Check the airframe is outside the circle; check frame time on the board (`ms` in the status line). |
| Betaflight `angle_limit`, rates, `deadband`, `map` | config | `get` in the CLI on the day; values here are from the 21 Nov dump. |
| `lens_*` (8 numbers) | config | Checkerboard recording and `cv::fisheye::calibrate` for the 960 x 540 image. The tool for this is not built yet. Cross-check: `lens_fx` should match the "focal length implied" that `calib` prints. |

## Not built (decide after the first bench session)

- **Lens calibration tool.** Needed as soon as we can record a checkerboard with the aircraft.
- **Recovery bonus (+5).** After the lens is covered the estimate does not include the motion made
  while covered. Needs re-registration against the first keyframe.
- **Loop closure** for bench step 3. Keyframe chaining drifts a few cm per keyframe.
- **Stopping if the sensor format changes at camera start** (see step 1).

## First session on the real board (21 Nov): what was measured

Ran over SSH, no battery, props off. Snapshots before and after are in `data/board/`.

| Finding | Meaning for the code |
|---|---|
| `/dev/video2` is the digital nadir camera (sensor entity `dummy_csi2_sensor`); `/dev/video3` is the analog one (`adv7180`, 720 x 576). | Device choice is right. |
| The camera gives 60 frames per second at 1920 x 1080. Asking for a smaller size gives a crop of the top-left corner, not a smaller picture, and it also changes the size in the organizers' sensor format. | Always ask for 1920 x 1080 and shrink in our code. Done. |
| Reading the camera buffer byte by byte took 55 ms a frame (14 frames per second). Copying it out in one block first gives about 60. | Fixed in `io::Camera`. |
| IMU: 1000 samples per second arrive, record layout as expected. | Works. |
| Flight controller answers `MSP_RC` on `/dev/ttymxc2`. | Works. |
| Estimator: 10 to 13 ms a frame while locked, about 70 ms when it has lost the view and makes a new keyframe every frame. | OPEN: too slow when lost. |
| Barometer: the first value read was wrong (120493 Pa), so every height after it was wrong by 1400 m. | OPEN: must drop the first values. |
| Barometer: the rate set by `run.sh` falls back to 1 per second as soon as a value is read. Each read then waits about 1 s. | OPEN: 1 value per second is too slow for the height loop. |
| The board already has a `vioh.service` from another logger. | Our folder and unit are named `nadirhold`. |
| `deploy.sh remove` plus writing the IMU settings back left two differences until reboot: `/dev/video2` default format and the serial port speed (115200, was 9600). | Reboot before handing on. |
