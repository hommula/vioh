# Research: GNSS-free position hold on the Orqa M2-AI (DTK-APB + Betaflight)

Date: 2026-11-21. Scope narrowed by the Stage 2 package (Betaflight, MSP override route).

## How to read this file

Every claim carries one of these tags.

- **[SRC]** I read the primary source in this session (file and line, or page).
- **[MEM]** From memory of the primary source named. I did not re-read it. Treat as **UNVERIFIED** until someone checks.
- **[INF]** My inference from sources that are tagged [SRC]. Not stated by the source itself.
- **UNVERIFIED** Could not be checked. Must be checked on the aircraft or with the organizers.

Nothing here was run on the aircraft. All Betaflight facts come from **upstream** Betaflight tag `2025.12.5` (commit `7348054f`, MSP API 1.47). The aircraft runs "Orqa Betaflight 2025.12.5, board APB". There is no `APB` target in the public Betaflight config repo (only `ORQA/ORQAF7`) [SRC: https://github.com/betaflight/config, tree listing]. So the Orqa build may differ from upstream. **UNVERIFIED** for every Betaflight line below until tested with props off.

Short names for local sources:

- **S2** = `D:\MISC\new\resources\redteam-grand-final\docs\Red-Team-Grand-Final-Stage-2.md` (section numbers).
- **JUDGE** = `D:\MISC\new\resources\redteam-grand-final\tools\hold_judge.py` (line numbers).
- **APB** = `D:\MISC\new\resources\additional details\OEM Manuals\DTK-APB-User-Manual-v1.7.2.pdf` (printed page numbers).
- **JD** = `Justice-D-Datasheet-v1.0.pdf`, **M2** = `M2-AI-Datasheet-v1.0.pdf`, **MRM2** = `MRM2-10AI-User-Manual-v1.0.pdf` (same folder).
- **BF** = `https://github.com/betaflight/betaflight/blob/2025.12.5/src/main/` + the path given.
- **LNX** = `https://github.com/torvalds/linux/blob/v6.6/` + path. **NXP** = `https://github.com/nxp-imx/linux-imx/blob/lf-6.6.52-2.2.0/` + path.
- **DPS** = Infineon DPS310 datasheet V1.2, 2020-10-15, https://community.infineon.com/gfawx74859/attachments/gfawx74859/KnowledgeBaseArticles/10104/3/Infineon-DPS310-DataSheet.pdf

---

## 0. Recommended stack (max 5)

1. **Control route: our loop sends `MSP_SET_RAW_RC` at 50 Hz, Betaflight flies ANGLE mode under MSP OVERRIDE.** This is the event configuration (S2 §03). We own roll, pitch, throttle and yaw. Send from process start, always.
2. **Horizontal estimate: register each frame against a keyframe taken at engage.** Track sparse corners, remove rotation with the APB gyro, fit a 4-parameter similarity (shift, yaw, scale). Hover then has no drift while the keyframe is in view. No full VIO.
3. **Height and scale: APB barometer (DPS310) relative to a zero taken on the ground.** Metric shift = pixel shift x height / focal length. The image scale change is a cross-check. The height loop must not depend on the camera.
4. **Sensors direct from Linux: V4L2 multiplanar mmap for the camera, IIO buffer for the ICM-42605, IIO sysfs polling for the DPS310.** All stamped on CLOCK_MONOTONIC. Log raw data to MCAP.
5. **Measure, do not assume: readout time, camera-IMU offset, intrinsics at 1920x1080, stick-to-tilt map, hover throttle.** These are blanks (section 7). The timing ones also earn the bonus (S2 §07).

Leave for later: full VIO, NPU/GPU, ArduCopter, Kalibr. One line each in sections 2 and 3.

What would change this recommendation: (a) the Orqa build does not behave like upstream in the props-off test, (b) corner tracking fails on the real ground texture at 50 to 75 m, (c) baro height is too noisy in prop wash to hold 2 m.

---

## 1. What the documents say about the platform

### 1.1 Stage 2 settles these (S2 overrides the OEM manuals)

| Fact | Source |
|---|---|
| FC firmware is Orqa Betaflight 2025.12.5, board `APB`. ArduCopter 4.7.2 is optional. No PX4, no INAV. | S2 §01, §03 |
| Link: `/dev/ttymxc2`, 115200 baud, MSP v1. | S2 §03 |
| Command: `MSP_SET_RAW_RC`, 8 x uint16 microseconds, order roll, pitch, throttle, yaw, AUX1..4. FC uses only the first four. 50 Hz recommended. | S2 §03 |
| Engage: CH10 high selects MSP OVERRIDE and ANGLE together. Low returns control at once. | S2 §03 |
| No command timeout. FC keeps our last command. Before our first command the four channels read minimum (throttle cut, full tilt). | S2 §03 |
| Nadir camera: `/dev/video2`, 1920x1080, 60 fps, YUYV, rolling shutter, no exposure control. | S2 §02, §08 |
| Timestamps: cameras and IMU on CLOCK_MONOTONIC. Camera stamp marks the END of the frame. No hardware sync. | S2 §02 |
| APB sensors: ICM-42605 (up to 4 kHz) and DPS310 through Linux IIO. A second IMU and baro sit on the FC. | S2 §02, §08 |
| Not present: rangefinder, magnetometer. | S2 §02 |
| Board: 4 x A53, about 3.9 GB RAM, Python 3.12 with numpy, no compiler, no ROS 2. | S2 §08 |
| Do not change device tree, kernel, boot config or APB GPIO lines. Do not enable `orqa-digital-osd`. Keep files in `/data`. | S2 §02 |
| Hold limits: 3 m horizontal (GPS), 2 m vertical (FC baro), 30 s, at 10, 20, 35, 50, 75 m. | S2 §06, JUDGE lines 24, 148-176 |
| Bench: displacement within 0.25 m over 0.5 to 1.5 m at about 1 m height; loop closure within 0.3 m; 0.5 m height change within 0.15 m. | S2 §05 |
| No GNSS from engage to disengage. This includes `MSP_RAW_GPS` and "GPS-aided state". | S2 §04 |

### 1.2 What the APB manual adds

- Linux reaches the FC over **UART3 = `/dev/ttymxc2`** and CAN2 [SRC: APB p.14, §3.12 and §4.2].
- UART3 goes through a **GPIO switch** (SW1 = GPIO4_IO13, SW0 = GPIO4_IO00). The FC path needs SW1=1, SW0=0 [SRC: APB p.14, Table 4.1]. S2 §02 forbids touching GPIO, so assume the organizers set it. If the link is silent, read the lines, do not write them. **UNVERIFIED** on the aircraft.
- CSI 1 is `/dev/video2`, "fixed 1280x720@60 without any external controls" [SRC: APB p.12]. S2 says 1920x1080. S2 wins.
- Analog camera rule: always `VIDIOC_STREAMOFF` then `close()` before exit, or the ADV7282 stops responding [SRC: APB p.16]. Apply the same clean stop to our capture.
- Display stack is Wayland/Weston and GStreamer exists (`gst-launch-1.0 videotestsrc ! autovideosink`) [SRC: APB p.16].
- Network: USB RNDIS, board is 192.168.75.1, SSH as root, rsync works [SRC: APB p.10].
- Time is wrong after boot without NTP [SRC: APB p.10]. This is one more reason to use CLOCK_MONOTONIC only.
- Thermal: shutdown at 95 C, fan holds about 62 C [SRC: APB p.11]. Read `/sys/class/thermal/thermal_zone0/temp`.
- `/data` and `/nvram` survive updates [SRC: APB p.18].

**The manual does NOT state** baud rate, protocol, any camera GStreamer pipeline, pixel formats, toolchain, cross-compiler, OpenCV, ROS, or NPU/GPU access. It only points to `https://developer.orqafpv.com` for the SDK [SRC: APB p.14, §4.1]. I did not open that portal (login). **UNVERIFIED**: SDK contents.

### 1.3 Other OEM sheets

- Justice D: Sony 1/2.8" CMOS, 1280x720, F2.0, FOV 114 H / 86 V / 121.5 D, 60 fps, MIPI CSI [SRC: JD p.1]. Sensor model and shutter type are not stated in the datasheet. Rolling shutter comes from S2.
- M2-AI: 1359 g, 10" props, 6S, lists "GPS / GLONASS" [SRC: M2 p.2-3].
- MRM2 manual describes Betaflight position hold using "GPS, magnetometer, and barometer" [SRC: MRM2 p.18]. That is a different aircraft. S2 says our aircraft has no magnetometer.

---

## 2. Flight controller interface

### 2.1 Answer

Use MSP override. Betaflight's own position hold cannot be used here, and its altitude hold is not selected by the engage switch. Our code must produce four stick values: roll and pitch as tilt requests, throttle for height, yaw 1500.

### 2.2 MSP v1 frame (implement exactly this)

[SRC: BF `msp/msp_serial.c` lines 129-220, 325; `msp/msp_serial.h` lines 91-106]

```
request : '$' 'M' '<' size cmd payload[size] checksum
reply   : '$' 'M' '>' size cmd payload[size] checksum
error   : '$' 'M' '!' size cmd ... checksum
checksum = size XOR cmd XOR every payload byte        (one byte)
```

- `size` and `cmd` are one byte each. All multi-byte fields are little-endian.
- FC input buffer is 192 bytes (`MSP_PORT_INBUF_SIZE`). Larger frames are dropped.
- Every request gets a reply, also "set" commands (empty payload). Read and discard them, or the serial buffer fills. [MEM: BF `msp/msp_serial.c` reply path; not re-read line by line]
- MSP v2, not needed for the event: `'$' 'X' '<' flags(1) cmd(2) size(2) payload crc`. CRC is `crc8_dvb_s2`: init 0, polynomial 0xD5, MSB first, over flags, cmd, size and payload [SRC: BF `common/crc.h` line 33, `common/crc.c` lines 63-74, `msp/msp_serial.c` lines 259-287].

```
crc ^= byte; repeat 8 times: crc = (crc & 0x80) ? (crc << 1) ^ 0xD5 : crc << 1;
```

Serial settings: 115200 [SRC: S2 §03]. 8N1, raw mode, no flow control is the Betaflight default [MEM]. **UNVERIFIED**.

### 2.3 Messages we need

IDs from BF `msp/msp_protocol.h`. Payloads from BF `msp/msp.c` at the lines given. All [SRC].

| Message | ID | Dir | Payload |
|---|---|---|---|
| `MSP_SET_RAW_RC` | 200 | to FC | N x uint16, microseconds. Raw receiver order (see 2.4). `msp.c` 2690-2705 |
| `MSP_RC` | 105 | from FC | channelCount x uint16 = `rcData[]`. **Internal order: roll, pitch, yaw, throttle, AUX1...** `msp.c` 1317-1321, `fc/rc_controls.h` 29-37 |
| `MSP_ATTITUDE` | 108 | from FC | int16 roll (0.1 deg), int16 pitch (0.1 deg), int16 yaw (whole deg). `msp.c` 1323-1327 |
| `MSP_ALTITUDE` | 109 | from FC | int32 estimated altitude (cm), int16 vario (cm/s). `msp.c` 1329-1336 |
| `MSP_RAW_IMU` | 102 | from FC | 3 x int16 acc (raw ADC counts), 3 x int16 gyro (deg/s), 3 x int16 mag. `msp.c` 1145-1164 |
| `MSP_STATUS` | 101 | from FC | uint16 cycle time, uint16 i2c errors, uint16 sensor bits, uint32 mode flags (first 32 bits), ... `msp.c` 1095-1117 |
| `MSP_BOXIDS` | 119 | from FC | one byte per active box: its permanent ID, in bit order. `msp.c` 2336-2341 |
| `MSP_RC_TUNING` | 111 | from FC | rates and expo. Payload not read by me. |
| `MSP_API_VERSION` | 1 | from FC | protocol, major, minor. Expect 1.47. `msp_protocol.h` 61-62 |

Reading the mode: bit *i* of the `MSP_STATUS` flags is the *i*-th entry of `MSP_BOXIDS` [SRC: BF `msp/msp_box.c` 402-418]. Permanent IDs: ARM 0, ANGLE 1, ALTHOLD 3, POS HOLD 11, MSP OVERRIDE 50 [SRC: `msp_box.c` 49-99]. Simpler: read CH10 from `MSP_RC` index 9. The judge uses the same channel (`rcData[9]`, threshold 1800) [SRC: JUDGE 97, 184].

### 2.4 How MSP override works in the source

- `MSP_SET_RAW_RC` copies the values into one static array. Channels not sent are set to 0. Nothing ever expires [SRC: BF `rx/msp.c` 36-62]. This is why the FC "has no command timeout".
- For each channel, the receiver code picks the MSP value when (a) the MSP OVERRIDE box is active and (b) the bit for that raw channel is set in `msp_override_channels_mask`. Otherwise it picks the pilot's value [SRC: BF `rx/msp_override.c` 31-44, `rx/rx.c` 664-685].
- The MSP value is clamped to `rx_min_usec`..`rx_max_usec` [SRC: `rx/msp_override.c` 35]. An array of zeros therefore reads as minimum. This matches the S2 warning (throttle cut, full tilt before the first command).
- **Channel order.** The MSP array index is the RAW receiver channel, `rcmap[channel]` [SRC: `rx/rx.c` 669]. With `map AETR1234` raw order is roll, pitch, throttle, yaw [SRC: `rx/rx.c` 76, 807-815]. That matches S2. Mask 15 = raw channels 0 to 3. Check `map` in the CLI once. **UNVERIFIED** on the aircraft.
- **`MSP_RC` during a hold returns OUR values for the four channels, not the pilot's sticks.** `rcData` is filled from the already-substituted samples [SRC: `rx/rx.c` 664-685, 690-770] [INF]. Before engage it returns the pilot's sticks. So the pilot's throttle just before engage is readable, and is a measured starting value for hover throttle.
- **`MSP_RC` and `MSP_SET_RAW_RC` use different orders.** Read: roll, pitch, yaw, throttle. Write: roll, pitch, throttle, yaw. Easy bug.
- New values are picked up when the receiver code runs, which is driven by real RC frames [SRC: `rx/rx.c` 577-584, 781-800] [INF]. If the RC link drops, normal Betaflight failsafe applies. `msp_override_failsafe` changes this [SRC: BF `pg/rx.h` 64]; its value on the aircraft is **UNVERIFIED**.
- CLI names: `msp_override_channels_mask`, `msp_override_failsafe` [SRC: BF `cli/settings.c` 898-899].

### 2.5 How ANGLE mode turns microseconds into tilt

[SRC: BF `fc/rc.c` 696-710, 656-668, 821-822, 186-238, 904; `flight/pid.c` 551-600]

```
rc          = clamp(us - midrc, -500, 500)
rcCommand   = deadband applied to rc            (CLI: deadband, yaw_deadband)
deflection  = rcCommand / (500 - deadband)      in -1 .. 1
angleTarget = angle_limit * rates(deflection) / rates(1.0)     degrees
```

- `rates()` is the pilot's rate curve (rate type, rc rate, expo, super rate). So the map is **not linear** unless expo is zero. Read it from the CLI (`get rates`, `get expo`, `get angle_limit`, `get deadband`) or `MSP_RC_TUNING`. Do not guess. This is a blank.
- Yaw stays a rate command in ANGLE mode. 1500 means zero yaw rate [SRC: `fc/rc.c` 700-709] [INF]. With no magnetometer the FC holds heading by gyro only, so slow yaw drift is expected. The image rotation from the similarity fit measures it.
- RC smoothing and an angle feedforward filter sit after this [SRC: `flight/pid.c` 561-566]. They add lag. Measure the step response with props off (blackbox).
- Horizontal acceleration is about g x tan(tilt) [INF, physics]. So the position loop outputs a tilt, then inverts the map above.
- Bench step 5 checks the sign: roll and pitch must oppose the move [SRC: S2 §05].

### 2.6 How throttle maps

[SRC: BF `fc/rc.c` 712-725]

- `rcData[THROTTLE]` is clamped to `min_check`..2000, rescaled, then passed through the throttle curve (`thr_mid`, `thr_expo`) and any throttle limit.
- This sets motor output, not thrust and not climb rate. Hover throttle changes with battery voltage and payload [INF].
- So the height loop needs an integral term and a measured starting value. Take the pilot's throttle from `MSP_RC` at the moment of engage.
- `thr_corr_value` adds throttle with tilt in ANGLE mode if set [SRC: BF `flight/imu.c` 742-760, `cli/settings.c` 984]. Value on the aircraft is **UNVERIFIED**.

### 2.7 Can Betaflight ALT_HOLD or POS_HOLD help? No, as configured.

- The code exists: `flight/alt_hold_multirotor.c`, `flight/pos_hold_multirotor.c`, `flight/autopilot_multirotor.c` [SRC].
- **ALT_HOLD.** When active, the mixer ignores stick throttle and uses the autopilot throttle [SRC: BF `flight/mixer.c` 808-812]. It needs the ALTHOLD box active, armed, accelerometer, altitude data [SRC: BF `fc/core.c` 1044-1061]. The engage switch selects only MSP OVERRIDE and ANGLE [SRC: S2 §03]. So throttle is ours. Also, Betaflight's altitude mixes GPS altitude with baro whenever there is a GPS fix and `altitude_source` is DEFAULT [SRC: BF `flight/position.c` 113-190]. That would be "GPS-aided state" under S2 §04. Using it would need a config change and organizer approval. Not planned.
- **POS_HOLD (the "virtual GPS" route).** Dropped. It needs `GPS_FIX` and either a healthy compass or `pos_hold_without_mag` plus a GPS-course heading learned in flight above 1 m/s [SRC: BF `flight/pos_hold_multirotor.c` 67-80, `flight/imu.c` 87, 618-634, 676-700]. There is no magnetometer. Feeding it needs `gps_provider = MSP` [SRC: BF `cli/settings.c` 254-256, `io/gps.c` 1417-1432], which replaces the real GPS that the judges use to measure drift. Not better than override.
- For the record: `MSP_SET_RAW_GPS` = 201, `MSP2_SENSOR_GPS` = 0x1F03 [SRC: BF `msp/msp.c` 3674-3710]. `MSP2_SENSOR_OPTICALFLOW` (0x300B) is defined but has no handler, and no flight code reads optical flow in this tag [SRC: grep of BF `src/main`, `msp/msp_protocol_v2_betaflight.h` 32].

### 2.8 GNSS-rule traps in FC telemetry

- `MSP_ALTITUDE` returns the mixed estimate from 2.7. It can contain GPS altitude [SRC: BF `flight/position.c` 113-190, 234-237].
- `MSP_ATTITUDE` yaw is corrected by GPS course over ground when there is a fix, more than 4 satellites and no compass [SRC: BF `flight/imu.c` 673-700].
- Roll and pitch come from gyro and accelerometer [SRC: BF `flight/imu.c` 714-722] [INF].
- **So during a hold: do not use `MSP_ALTITUDE`, `MSP_ATTITUDE` yaw or `MSP_RAW_GPS`.** Use the APB sensors. This is my reading of S2 §04; ask the organizers if FC roll and pitch are allowed.

### 2.9 Safety behaviour we must build

All from S2 §03 and §05.

- Send continuously from process start. When not ready: roll, pitch, yaw = 1500 and a throttle that holds height.
- "Not ready" within 1 s of the camera being covered. Normal output only after it is uncovered (step 7).
- Auto-start after an APB power cycle, ready within 60 s, safe values from the first command (step 8).
- If our process dies, the FC keeps the last command. Use a supervisor that restarts it. Keep the height loop independent of the camera.

### 2.10 ArduCopter (fallback only)

One paragraph. Flash at own risk; recovery needs the organizers [SRC: S2 §03]. Link becomes MAVLink 2 at 230400 baud [SRC: S2 §03]. ArduPilot takes external position as `ODOMETRY` (preferred) or `VISION_POSITION_ESTIMATE`, at 4 Hz or more, with `VISO_TYPE = 3`, `EK3_SRC1_POSXY = 6`, `EK3_SRC1_VELXY = 6 or 0`, `EK3_SRC1_POSZ = 1` (baro), `EK3_SRC1_YAW = 6 or 1`, and the EKF origin must be set when no GPS is attached [SRC: https://ardupilot.org/dev/docs/mavlink-nongps-position-estimation.html]. Optical flow as the only horizontal source is tied to rangefinder range [SRC: https://ardupilot.org/copter/docs/common-optical-flow-sensor-setup.html], and there is no rangefinder. It also loses the +5 Betaflight bonus and starts at default parameters [SRC: S2 §03, §07]. Not needed.

---

## 3. Estimation

### 3.1 Answer

Use keyframe registration with gyro derotation and baro height. It is the smallest thing that has zero drift in hover. Full VIO is the wrong tool for this task.

### 3.2 The simple pipeline

1. Take the Y bytes of the YUYV frame as the grey image. Downscale. The factor is a blank: pick it from measured CPU time on the board.
2. At engage, store a keyframe: grey image, corner list, gyro-integrated attitude, baro height.
3. Each frame: track corners from the previous frame (pyramidal Lucas-Kanade).
4. Undistort the points to normalized coordinates. Rotate them by the attitude change since the keyframe. Use the row time of each point (section 4).
5. Fit a 4-parameter similarity (shift x, shift y, yaw, scale) between keyframe points and current points, with RANSAC.
6. Metric offset = shift x height. Yaw error = fitted rotation. Scale = height ratio.
7. Quality gate: inlier count and inlier ratio. Below the gate, report "not ready". Thresholds are blanks.

Why this fits:

- Hover over a fixed spot keeps the keyframe in view. Footprint grows with height, so this gets easier as the aircraft climbs [INF].
- A scale error only changes the loop gain. The pixel error is zero exactly when the aircraft is back over the engage point [INF].
- For the bench loop of about 3 m at 1 m height (step 3), the keyframe leaves the view. Chain keyframes, and try to re-register against the first keyframe when it returns. The same step gives the recovery bonus [SRC: S2 §05, §07] [INF].
- After a camera cover, Lucas-Kanade has no previous points. Re-register against the keyframe directly, for example with phase correlation on a small image, then re-detect corners [INF].

The main error at altitude is rotation, not scale. An attitude error of angle d moves the estimated position by height x tan(d). At 75 m, 1 degree is 1.3 m of the 3 m budget [INF, geometry]. A constant bias cancels because the keyframe has the same bias. A changing one does not. So gyro bias, camera-IMU time offset and camera-to-IMU rotation matter more than anything else here.

Source of the method: PX4Flow computes metric velocity as (flow rate - gyro rate) x height [MEM: Honegger et al., "An open source and open hardware embedded metric optical flow CMOS camera for indoor and outdoor applications", ICRA 2013, https://doi.org/10.1109/ICRA.2013.6630805]. INAV does the same in code: `(flowRate - bodyRate) * surface.alt` [SRC: https://github.com/iNavFlight/inav/blob/master/src/main/navigation/navigation_pos_estimator_flow.c lines 85-89].

### 3.3 Metric scale without a rangefinder

Metric shift = normalized image shift x height above the ground. So the relative height error equals the relative displacement error [INF, geometry].

DPS310 figures [SRC: DPS p.1 and Table 7 on p.9]:

| Quantity | Value |
|---|---|
| Pressure precision (noise, typical, RMS) | 1.0 Pa low power, 0.35 Pa standard, 0.2 Pa high precision |
| Front-page wording | +/-0.002 hPa (or +/-0.02 m) in high precision mode |
| Relative accuracy | +/-6 Pa (+/-0.06 hPa, "or +/-0.5 m"), for any 1 hPa step, 0 to 65 C |
| Absolute accuracy | +/-100 Pa (+/-1 hPa, "or +/-8 m") |
| Temperature sensitivity | 0.5 Pa/K |
| Measurement time | 5.2 ms low power, 27.6 ms standard (16x), 105 ms high precision |
| Rate | 1 to 128 Hz |

The datasheet's own conversion is about 8 m per hPa, so 1 Pa is about 8 cm [SRC: DPS p.1]. Noise is therefore about 2 cm (high precision) to 8 cm (low power) RMS. Temperature drift is about 4 cm per kelvin [INF from the table]. These are bench figures. Prop wash and wind in flight are not in the datasheet. **UNVERIFIED** in flight; measure by logging the baro while the pilot hovers.

Three options:

| Option | How | Good | Bad |
|---|---|---|---|
| A. Baro height above a ground zero | Zero the baro on the floor or at take-off. Height = baro change. | One line of code. Works in hover. | Needs flat ground under the hold point. Drifts with temperature and room pressure. At 1 m, a few cm of noise is a few percent. |
| B. Baro change over visual scale change | During a height change, image scale ratio s = h1/h2 and baro gives h2 - h1. Then h2 = (h2 - h1) / (1 - s). | No ground zero needed. Gives height above the real ground, not above take-off. | Only works while height changes. Error grows when the height step is small compared with the height. |
| C. Accelerometer against visual acceleration | Fit the scale that makes the second derivative of the visual track match the IMU. This is what VIO does. | No baro. | Needs acceleration, so nothing in hover. A 1 degree tilt error fakes 0.17 m/s2. Most code. |

Recommendation:

- **Flight (10 to 75 m): option A.** The hold only needs scale for loop gain, so 10 percent error is tolerable [INF]. Log option B during each climb between levels as a cross-check.
- **Bench (about 1 m): option A if the aircraft can sit on the floor first** to zero the baro. Whether the judge allows this is a blank. Bench step 4 (0.5 m rise within 0.15 m) is a baro-only measurement; the datasheet noise is well inside 0.15 m [INF from DPS p.9]. The image scale ratio is a second measurement of the same rise.
- Option C: not needed yet.

Vertical hold is judged on the FC barometer [SRC: S2 §06, JUDGE 155-159]. The APB baro is a different chip on the same board, so both see nearly the same pressure errors [INF]. Holding the APB baro constant is therefore the right target.

### 3.4 Full VIO: why not

- Monocular VIO gets metric scale from accelerometer excitation. In hover or constant velocity the scale is not observable [MEM: Martinelli, "Closed-form solution of visual-inertial structure from motion", IJCV 2014; Qin, Li, Shen, "VINS-Mono", IEEE T-RO 2018, https://arxiv.org/abs/1708.03852]. Our task is exactly hover.
- A nadir camera over flat ground is a planar scene. That is a known degenerate case for structure-from-motion initialisation [MEM: general result, e.g. the ORB-SLAM papers choose between homography and fundamental matrix for this reason].
- No ROS on the board [SRC: S2 §08], no compiler, and a 4 x A53 CPU shared with 1080p60 capture.

| System | Licence [SRC: GitHub licence API, today] | ROS needed [MEM] | Rolling shutter [MEM] | Hover / scale [MEM] |
|---|---|---|---|---|
| OpenVINS | GPL-3.0 | ROS-free build exists | Global shutter model; online time-offset calibration [SRC: https://docs.openvins.com/gs-calibration.html] | Has zero-velocity update; scale still needs excitation |
| VINS-Mono / VINS-Fusion | GPL-3.0 | Yes (ROS 1) | Has a rolling-shutter option and online time offset | Initialisation needs motion |
| Basalt | BSD-3-Clause | No | Calibration tools only | Stereo-first |
| ROVIO | BSD-3-Clause text (API reports no SPDX id) | Yes in practice | No | Scale drifts without excitation |
| Kimera-VIO | BSD-2-Clause | Optional | No | Stereo-first |
| ORB-SLAM3 | GPL-3.0 | No | No | IMU initialisation needs motion |
| SVO Pro | GPL-3.0 | Yes (catkin) | No | Same scale limit |

CPU cost on Cortex-A53: I found no primary-source numbers. **UNVERIFIED**. Not needed unless the simple pipeline fails.

---

## 4. Rolling shutter and camera-IMU timing

These earn up to +5 for "measured evidence" [SRC: S2 §07]. Keep the plots.

### 4.1 What the readout time means

- A rolling shutter exposes rows one after another. Readout time t_r is the time from the first row to the last row [MEM: Karpenko et al., "Digital Video Stabilization and Rolling Shutter Correction using Gyroscopes", Stanford CSTR 2011-03].
- The camera stamp marks the end of the frame [SRC: S2 §02]. In the NXP driver the stamp is `ktime_get_ns()` taken in the frame-done interrupt [SRC: NXP `drivers/staging/media/imx/imx8-isi-cap.c` line 195]. So for row v of H rows:

```
t_row(v) = t_stamp - t_d - t_r * (1 - v / H)
```

  t_d is the remaining fixed delay (half the exposure time, plus pipeline delay). Both t_r and t_d are blanks. t_r cannot exceed one frame period [INF].
- The sensor model is not in the datasheet [SRC: JD p.1], so t_r cannot be looked up. Measure it.

### 4.2 Measuring t_r

Two methods. Use the second first, because it reuses the code from 4.3.

1. **Flicker bands.** Point the camera at a light that flickers at a known frequency. Bands appear. Rows per band x band count gives the row time [MEM: common method, e.g. Karpenko 2011]. Needs a light with a trusted frequency. With no exposure control [SRC: S2 §08] the bands may be faint.
2. **Gyro against image rows.** Shake the aircraft by hand in yaw or pitch over a textured floor, props off. Compute image-based rotation rate separately for the top band and the bottom band of the image. Find the time lag of each band against the gyro (4.3). The difference in lag, divided by the row distance between the bands as a fraction of H, is t_r [INF from Karpenko 2011, which fits t_r and t_d jointly from gyro and feature tracks].

Kalibr also estimates shutter timing (`kalibr_calibrate_rs_cameras`, models `pinhole-radtan-rs`, `pinhole-equi-rs`, `omni-radtan-rs`), but needs a ROS bag, a target, and cannot calibrate an IMU in the same run [SRC: https://github.com/ethz-asl/kalibr/wiki/Rolling-Shutter-Camera-calibration]. Offline on a laptop only. Later.

### 4.3 Measuring the camera-IMU time offset

1. Record gyro (IIO) and frames (V4L2) with their kernel stamps while shaking the aircraft by hand.
2. Image rotation rate: the rotation angle of the frame-to-frame similarity fit gives yaw rate. Mean flow divided by focal length gives pitch and roll rate.
3. Resample both signals to one rate. Cross-correlate. The lag at the peak is the offset. Fit a parabola to the peak for sub-sample precision.
4. Repeat a few times. Report mean and spread.

This is the standard approach [MEM: Mair et al., "Spatio-temporal initialization for IMU to camera registration", ROBIO 2011; Qin and Shen, "Online Temporal Calibration for Monocular Visual-Inertial Systems", IROS 2018, https://arxiv.org/abs/1808.00692]. Kalibr's camera-IMU calibration estimates the same offset by default, from a ROS bag with a fixed target while moving the sensor [SRC: https://github.com/ethz-asl/kalibr/wiki/camera-imu-calibration]. Later.

IMU-side caveat: the ICM-42605 driver stamps samples from the FIFO interrupt time and spreads them by the sample period [SRC: LNX `drivers/iio/imu/inv_icm42600/inv_icm42600_core.c` 425-426, `inv_icm42600_gyro.c` 789-801, `drivers/iio/common/inv_sensors/inv_sensors_timestamp.c`]. So IMU stamps are estimated, not per-sample hardware stamps. Keep the FIFO watermark low to limit the error [INF].

### 4.4 Corrections, and what matters in hover

- **Per-point gyro derotation with row time.** Rotate each tracked point by the gyro rotation between its own row time and the frame reference time [MEM: Karpenko 2011]. This is a few lines once t_r and t_d are known. Do this.
- Rolling-shutter-aware estimators and camera models (VINS-Mono option, Kalibr `-rs` models). Not needed.
- In a near-stationary hover the slow motion is small, so the frame-wide skew is small [INF]. What remains is vibration: rows shift within one frame ("jello"). A global fit cannot remove it. Per-point derotation with kHz gyro data helps. The driver offers gyro rates up to 4 kHz [SRC: LNX `inv_icm42600_gyro.c` 272-289].
- Many corners spread over the whole image average the wobble out [INF].
- Motion blur cannot be reduced: no exposure control [SRC: S2 §08]. Check image sharpness on the first real flight log.

### 4.5 Intrinsics for a 114 degree lens

- The datasheet FOV is for 1280x720 [SRC: JD p.1]. The event mode is 1920x1080 [SRC: S2 §08]. Whether it is a crop or a rescale is unknown. **Calibrate at 1920x1080 on our own aircraft.** Blank.
- Model choice: Kalibr and OpenVINS advise `pinhole-equi` (the OpenCV fisheye model) for strong distortion and `pinhole-radtan` for low distortion [SRC: https://docs.openvins.com/gs-calibration.html]. At 114 degrees try both and keep the one with the lower reprojection error [INF].
- OpenCV: `cv::calibrateCamera` (radtan, optional `cv::CALIB_RATIONAL_MODEL`) or `cv::fisheye::calibrate` (equidistant) [MEM: https://docs.opencv.org/4.x/d9/d0c/group__calib3d.html and https://docs.opencv.org/4.x/db/d58/group__calib3d__fisheye.html]. Run it offline on the laptop from saved frames.
- S2 §05 says team-supplied aids are not used in **scored** tests. A checkerboard for our own calibration before scoring is my assumption of what is allowed. Ask.

---

## 5. Building blocks

### 5.1 Camera: V4L2 multiplanar mmap

The NXP capture driver reports `V4L2_CAP_STREAMING | V4L2_CAP_VIDEO_CAPTURE_MPLANE`, queue type `V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE`, I/O modes MMAP and DMABUF, timestamps `V4L2_BUF_FLAG_TIMESTAMP_MONOTONIC` [SRC: NXP `drivers/staging/media/imx/imx8-isi-cap.c` lines 807, 1657-1671]. Its driver name string is `mxc-isi-cap_v1` [SRC: NXP `drivers/staging/media/imx/imx8-isi-core.h` line 43]. Whether Orqa patched this driver is **UNVERIFIED**.

Call sequence [MEM: https://docs.kernel.org/userspace-api/media/v4l/mmap.html and `planar-apis.html`]:

```
fd = open("/dev/video2", O_RDWR)
VIDIOC_QUERYCAP                     check V4L2_CAP_VIDEO_CAPTURE_MPLANE
VIDIOC_G_FMT   type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE     read width, height, pixelformat,
                                                             plane_fmt[0].bytesperline, sizeimage
VIDIOC_REQBUFS count = n, type = ..._MPLANE, memory = V4L2_MEMORY_MMAP
per buffer: VIDIOC_QUERYBUF with buf.m.planes = planes[], buf.length = number of planes
            mmap(NULL, planes[0].length, PROT_READ, MAP_SHARED, fd, planes[0].m.mem_offset)
            VIDIOC_QBUF
VIDIOC_STREAMON
loop: poll(fd, POLLIN) -> VIDIOC_DQBUF -> use -> VIDIOC_QBUF
VIDIOC_STREAMOFF, munmap, close     always, see APB p.16
```

- Stamp: `buf.timestamp` (a `timeval`, CLOCK_MONOTONIC, end of frame). Check `buf.flags & V4L2_BUF_FLAG_TIMESTAMP_MASK`.
- Dropped frames: gaps in `buf.sequence`.
- S2 says the sensor format is set at boot [SRC: S2 §02]. Read it with `G_FMT`. Do not assume `S_FMT` can change it.
- YUYV byte order is Y0 U Y1 V. Grey image = every second byte, row stride = `bytesperline` [MEM: https://docs.kernel.org/userspace-api/media/v4l/pixfmt-packed-yuv.html].
- One 1920x1080 YUYV frame is 4.1 MB, about 250 MB/s at 60 fps [INF]. Downscale while extracting Y. Do not copy full frames.
- `cv::VideoCapture` with `cv::CAP_V4L2` on a multiplanar-only node: **UNVERIFIED**. Raw V4L2 is small and gives the kernel stamp. Use it.
- List formats on the board: `v4l2-ctl -d /dev/video2 --list-formats-ext` (if `v4l2-ctl` is installed, **UNVERIFIED**).

### 5.2 IMU: IIO buffered read (ICM-42605)

[SRC: LNX `drivers/iio/imu/inv_icm42600/` files as cited; ABI in LNX `Documentation/ABI/testing/sysfs-bus-iio`]

- The driver makes **two IIO devices**. Their `name` files read `icm42605-gyro` and `icm42605-accel` [SRC: `inv_icm42600_gyro.c` 727, `inv_icm42600_core.c` 81-82]. Find them by scanning `/sys/bus/iio/devices/iio:device*/name`. Do not hard-code N.
- Scan elements per device: x, y, z, temperature, timestamp [SRC: `inv_icm42600_gyro.c` 51-55, 64-71]. x, y, z are 16-bit big-endian [SRC: same file 41-45].
- Read the exact layout from `scan_elements/in_*_type` and `in_*_index` (for example `be:s16/16>>0`). Do not hard-code it. The timestamp is a signed 64-bit nanosecond value, aligned to 8 bytes [MEM: https://docs.kernel.org/driver-api/iio/buffers.html].
- Scale: `in_anglvel_scale` gives rad/s per count (for example 0.001065264 at +/-2000 dps). `in_accel_scale` gives m/s2 per count (for example 0.004788403 at +/-16 g) [SRC: `inv_icm42600_gyro.c` 202-222, `inv_icm42600_accel.c` 202-215]. Read the file; do not copy these numbers.
- Rate: `sampling_frequency`, allowed gyro values 12.5, 25, 50, 100, 200, 1000, 2000, 4000 Hz [SRC: `inv_icm42600_gyro.c` 272-289].
- Clock: `current_timestamp_clock` names the POSIX clock used for buffer stamps [SRC: `sysfs-bus-iio` lines 87-92]. The kernel default is `realtime` [MEM: LNX `drivers/iio/industrialio-core.c`]. S2 says the IMU is on CLOCK_MONOTONIC. **Read the file at start-up and refuse to run if it is not `monotonic`.** Writing `monotonic` to it is a config choice for the organizers' rules, not a kernel change [INF].
- Steps: write 1 to the wanted `scan_elements/in_*_en` files, set `buffer/length` and `buffer/watermark`, write 1 to `buffer/enable`, then `read()` fixed-size records from `/dev/iio:deviceN` (it supports `poll`) [MEM: https://docs.kernel.org/driver-api/iio/buffers.html].
- The device supports a hardware FIFO watermark [SRC: `inv_icm42600_gyro.c` 671-715]. A large watermark delivers samples in bursts.
- Axis directions of the APB IMU relative to the camera: **UNVERIFIED**. Measure with the shake test in 4.3. `in_mount_matrix` exists [SRC: `inv_icm42600_gyro.c` 59] but may be identity.
- Another process holding the buffer (for example an Orqa service) would block us. **UNVERIFIED**.

### 5.3 Barometer: IIO sysfs polling (DPS310)

- **The mainline 6.6 driver has no buffer.** It exposes processed values and settings only [SRC: LNX `drivers/iio/pressure/dps310.c` lines 95-108, 707-718]. The brief assumed buffered reads; mainline does not support that. Orqa's kernel may differ. **UNVERIFIED**.
- Files: `in_pressure_input` in **kPa** (the driver divides Pa by 1000) [SRC: `dps310.c` 707-718], `in_temp_input`, `in_pressure_sampling_frequency`, `in_pressure_oversampling_ratio`, and the same two for temperature [SRC: `dps310.c` 95-108].
- No kernel stamp. Stamp each read with `clock_gettime(CLOCK_MONOTONIC)` after the read returns [INF].
- Each read does a bus transaction. Poll from its own thread at a steady rate. The rate and oversampling are blanks; noise against measurement time is in the table in 3.3.
- Height from pressure: use the change only. About 8 cm per Pa near sea level [SRC: DPS p.1 conversion]. Use the standard barometric formula for the 75 m range [MEM: ICAO standard atmosphere].

### 5.4 OpenCV calls

There is no OpenCV on the board [SRC: S2 §08 lists only Python and numpy]. Link the needed modules statically (`core`, `imgproc`, `video`, `calib3d`) into our binary [INF]. All signatures below are [MEM: https://docs.opencv.org/4.x/]. Check against the version we build.

```cpp
cv::goodFeaturesToTrack(gray, corners, maxCorners, qualityLevel, minDistance, mask, blockSize);
cv::calcOpticalFlowPyrLK(prevGray, gray, prevPts, nextPts, status, err, winSize, maxLevel, criteria);
cv::Mat A = cv::estimateAffinePartial2D(fromPts, toPts, inliers, cv::RANSAC, reprojThreshold);
//   A is 2x3: [ s*cos(a)  -s*sin(a)  tx ;  s*sin(a)  s*cos(a)  ty ]  -> shift, yaw, scale
cv::Mat H = cv::findHomography(fromPts, toPts, cv::RANSAC, reprojThreshold, inlierMask);
cv::Point2d shift = cv::phaseCorrelate(prev32f, cur32f, hannWindow, &response);   // CV_32FC1 inputs
cv::undistortPoints(pixels, normalized, K, D);                 // radtan model
cv::fisheye::undistortPoints(pixels, normalized, K, D);        // equidistant model
```

- Use `estimateAffinePartial2D` for the main fit. Use the homography only if the ground is clearly not seen fronto-parallel. Not needed yet.
- `phaseCorrelate` is the fallback for re-finding the keyframe after a camera cover. It gives shift only.
- Every numeric argument above is a tuning value. Pick them on data from our own camera only.
- OpenCV uses NEON on aarch64 through its universal intrinsics [MEM: https://docs.opencv.org/4.x/df/d91/group__core__hal__intrin.html]. OpenCL/GPU and the NPU: not needed yet.

### 5.5 MCAP C++

[SRC: https://github.com/foxglove/mcap/blob/main/cpp/README.md]

- Header-only. MIT licence. Define `MCAP_IMPLEMENTATION` in exactly one `.cpp`.
- Dependencies are lz4 and zstd. Remove them with `MCAP_COMPRESSION_NO_LZ4` and `MCAP_COMPRESSION_NO_ZSTD`. For a static binary on the board, start with both off.
- Write: `mcap::McapWriter writer; writer.open(path, mcap::McapWriterOptions(profile)); writer.addSchema(s); writer.addChannel(c); writer.write(msg); writer.close();` with `msg.logTime` in nanoseconds.
- Read: `mcap::McapReader reader; reader.open(path); for (const auto& view : reader.readMessages()) { view.message.data, view.message.dataSize, view.message.logTime, view.channel->topic }` [MEM: https://github.com/foxglove/mcap/blob/main/cpp/mcap/include/mcap/reader.hpp].
- Use the sensor's CLOCK_MONOTONIC stamp as `logTime`. Put the write time in `publishTime` [INF].
- Raw 1080p60 is about 250 MB/s. Log the downscaled grey image or a subset of frames. `/data` size is a blank.

### 5.6 Cross-compiling

Already solved by the team (static aarch64 binary). Skipped on request.

---

## 6. Things that contradict the original brief

1. **Camera mode is 1920x1080 YUYV**, not 1280x720 [SRC: S2 §08 against JD p.1 and APB p.12]. The 114/86 degree FOV may not hold in this mode.
2. **The APB manual has no baud, protocol, camera pipeline, toolchain or OpenCV facts.** Only S2 gives them.
3. **The FC link depends on a GPIO switch** [SRC: APB p.14]. S2 forbids touching GPIO.
4. **No public Betaflight target for this board.** The firmware is an Orqa build. Upstream source facts are a strong guide, not proof.
5. **The DPS310 mainline driver cannot do buffered reads** [SRC: LNX `dps310.c`]. Poll sysfs.
6. **`MSP_ALTITUDE` and `MSP_ATTITUDE` yaw can be GPS-aided** [SRC: BF `flight/position.c`, `flight/imu.c`]. Using them during a hold risks the GNSS rule.
7. **`MSP_RC` and `MSP_SET_RAW_RC` use different channel orders**, and `MSP_RC` shows our values, not the pilot's, during a hold.
8. **No magnetometer** [SRC: S2 §02]. The brief did not say. It rules out Betaflight position hold without flying fast first.
9. **Betaflight has no optical-flow or external-position input that reaches the flight code** in this tag [SRC: grep of BF `src/main`].

---

## 7. Blanks to fill (measure on our aircraft, or ask)

Measure on the bench, props off:

- [ ] `MSP_API_VERSION` answers 1.47. `MSP_SET_RAW_RC` moves the Receiver tab only when CH10 is high.
- [ ] CLI `map`, `get msp_override`, `get rx_min_usec`, `get min_check`, `get deadband`, `get angle_limit`, `get rates`, `get expo`, `get thr_`, `get altitude_source`. Record a `diff all`.
- [ ] Stick microseconds to tilt degrees: compute from the values above, then confirm sign and size with blackbox.
- [ ] Which box bits `MSP_STATUS` shows for ANGLE and MSP OVERRIDE (`MSP_BOXIDS`).
- [ ] `/dev/video2` format from `VIDIOC_G_FMT`: size, fourcc, `bytesperline`. Real frame rate and drop rate from `sequence`.
- [ ] IIO: device numbers, `name`, `current_timestamp_clock`, scan element types, available sampling frequencies. Whether the DPS310 has a buffer on this kernel.
- [ ] IMU axes relative to image axes.
- [ ] Camera intrinsics and distortion model at 1920x1080.
- [ ] Rolling-shutter readout time t_r (section 4.2). Keep the plot.
- [ ] Camera-IMU time offset t_d (section 4.3). Keep the plot.
- [ ] CPU time per frame for each downscale factor. Pick the factor from this.
- [ ] Baro noise on the bench at each oversampling setting.

Measure in a development flight:

- [ ] Hover throttle at engage, and how it moves with battery voltage.
- [ ] Baro height noise in prop wash, APB baro against FC blackbox `baroAlt`.
- [ ] Corner count and inlier ratio over the real ground at each level. Sets the "ready" thresholds.
- [ ] Image sharpness (no exposure control).
- [ ] Tilt step response in ANGLE mode. Sets the position loop gains.

Ask the organizers:

- [ ] May we read FC roll and pitch (`MSP_ATTITUDE`) during a hold, given yaw can be GPS-aided?
- [ ] May we change Betaflight settings (not firmware), and which ones?
- [ ] On the bench, may the aircraft sit on the floor first so we can zero the baro?
- [ ] May we use our own checkerboard and a flicker light for calibration before scoring?
- [ ] Is the ground under the flight levels flat and at take-off height?
- [ ] What is `msp_override_failsafe` set to, and what happens on RC link loss during a hold?
- [ ] Free space on `/data`.
