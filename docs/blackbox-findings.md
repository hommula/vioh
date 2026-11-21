# Blackbox findings: FC backup of 2026-11-21

Source: `dumps/fc-backup-2026-11-21/fc-backup-2026-11-21/blackbox_flash_16MB.bin` (read-only, not modified).
Decoded on 2026-11-21 with `blackbox_decode` (betaflight/blackbox-tools, commit 57c7695) and cross-checked with `orangebox` 0.5.0.

## The answer in short

- The chip holds two arm/disarm blips on the ground and **one real 30 s flight**: a manual hover in rate (acro) mode, well under 1 m high.
- **Hover throttle was rcCommand[3] = 1369 (std 7)**. That is about **1400 us raw RC throttle**. The configured `ap_hover_throttle = 1550` does **not** agree.
- **The engage switch, ANGLE mode and MSP override never appear.** Channel 10 is not logged at all, in any Betaflight blackbox log.
- **`hold_judge.py` does not work on these CSVs as-is.** It finds no engage, and it reads baro centimetres as metres.
- No failsafe, no desync, no dropped frames. All three logs ended by the arm switch.

## What to do (in order)

1. **Ask the scoring desk how they will find the engage on a Betaflight log.** Show them the two `hold_judge.py` problems in section 7. This decides points, so do it first.
2. **Start our throttle feed-forward near 1400 us raw, not 1550.** Confirm with one hover in ANGLE + MSP override.
3. **Treat stick-to-angle as a curve, not a straight line** (table in section 4). Confirm with the same test flight.
4. **Budget for baro noise.** In this hover the baro read from -66 to +124 cm while GPS altitude stayed within 0.8 m. The judges' vertical limit is 2 m.

Leave for later: out-of-ground-effect hover throttle, unfiltered vibration data, a check of the Blackbox Explorer CSV export. Each needs a new flight or the judges' answer.

## How to read the tags

Every number carries one tag.

| Tag | Meaning |
|---|---|
| [log] | Measured from the decoded log data |
| [header] | Read from the text header inside the log |
| [config] | Read from `dump_all.txt` / `diff_all.txt` |
| [source] | Read from stock Betaflight 2025.12.5 source code. The Orqa build may differ |
| [calc] | Computed by me from the other numbers. Not measured |

## What I ran, what I only read, what I assume

- **Ran:** both decoders on all 3 logs, my own statistics scripts (numpy), and `hold_judge.py` (Python 3.11.9) in 8 variations.
- **Only read:** stock Betaflight 2025.12.5 source, blackbox-tools source, and the Blackbox Explorer CSV export source.
- **Assume:** the Orqa firmware keeps the stock meaning of log fields, flag bits and the stick mapping. One fact supports this: flag bit 0 turns on at arming and off about 110 ms before disarm, as stock ARM would.
- **Assume:** 6S battery (23.3 V at rest [log]). Cell count is not in the log.
- **Not known:** payload and battery charge on this flight. The log is from this same FC (DeviceUID in [header] equals `mcu_id` in [config]).

---

## 1. What is in each log

All three were recorded within 43 s of each other. All were armed. All ended with disarm reason 4 = SWITCH [log event, name from source].

| | Log 1 | Log 2 | Log 3 |
|---|---|---|---|
| Start time, UTC [header] | 02:56:19.243 | 02:56:21.592 | 02:56:31.571 |
| FC uptime at start [log] | 54.9 s | 57.3 s | 67.2 s |
| Duration [log] | 0.98 s | 2.01 s | 30.66 s |
| Frames (orangebox) [log] | 984 | 2010 | 30642 |
| Armed [log] | yes | yes | yes |
| Throttle rcCommand[3] [log] | 1000 always | 1000 always | 1000 to 1385 |
| What it is | Arm blip on the ground | Arm blip on the ground | Real flight |
| GPS [log] | 0 satellites, no position | 0 satellites, no position | 12 to 16 satellites, home fix |

Logs 1 and 2: motors idle (motor output about 165 on a 158..2047 scale [log, header]). No stick input. Attitude does not move. Not flights.

Log 3 timeline [log], seconds from log start:

| Time | What happens |
|---|---|
| 0 to 2.1 | Idle on the ground |
| 2.1 to 7 | Slow throttle ramp, 1000 to 1345 |
| about 5.8 to 7 | Lift-off. Motors start to differ and attitude starts to move. This is my reading, not a logged event |
| 8.0 to 30.3 | Hover. Small roll inputs. Height roughly 0.5 m (see section 6) |
| 30.52 | Touchdown bump: 1.73 g peak, up to 53 deg/s |
| 30.54 | Arm switch off |
| 30.66 | Disarm, log ends |

**Log rate: 1 kHz.** Loop time 125 us, P interval 8, I interval 256 [header]. Median frame spacing 1001 us [log].

Fields in the header (same in all three logs) [header]:

- Main frames (42 fields): `loopIteration, time, axisP[0..2], axisI[0..2], axisD[0..1], axisF[0..2], rcCommand[0..3], setpoint[0..3], vbatLatest, amperageLatest, baroAlt, rssi, gyroADC[0..2], accSmooth[0..2], imuQuaternion[0..2], motor[0..3], eRPM[0..3]`
- GPS frames: `time, GPS_numSat, GPS_coord[0..1], GPS_altitude, GPS_speed, GPS_ground_course, GPS_velned[0..2]`
- GPS home frame: `GPS_home[0..2]`
- Slow frames: `flightModeFlags, stateFlags, failsafePhase, rxSignalReceived, rxFlightChannelsValid`

| Asked field | Present? |
|---|---|
| rcData (any channel) | **No.** Betaflight does not log raw RC channels |
| rcData / rcCommand for channel 10 (index 9) | **No.** rcCommand has only indexes 0..3: roll, pitch, yaw, throttle |
| baroAlt | Yes, in cm [source] |
| GPS_coord | Yes. Usable in log 3 only |
| GPS_numSat | Yes |
| flightModeFlags | Yes |
| setpoint | Yes. [0..2] are rates in deg/s. [3] is mixer throttle, 0..1000 [source] |
| motor | Yes, 4 motors, plus eRPM |
| accSmooth | Yes. After a 25 Hz low-pass (`acc_lpf_hz = 25` [config]) |
| gyroADC | Yes. Filtered gyro only. Unfiltered gyro is switched off [config] |
| Attitude | Only as `imuQuaternion[0..2]`. I converted it to roll/pitch/yaw |
| Debug fields, mag | No |

The log holds the real GPS position of the flight site. I did not copy it here.

## 2. Hover throttle

**Hover needed rcCommand[3] = 1369. The config value 1550 is about 150 us too high.**

How I chose the segments, log 3:

- **A, whole airborne window:** 8.0 to 30.3 s (22.3 s). After lift-off settled, before the landing descent.
- **B, steady subset of A:** samples where GPS vertical speed is under 15 cm/s, tilt under 3 deg, GPS ground speed under 0.25 m/s (14.7 s).

GPS altitude changed by only 0.2 m across window A [log], so the mean throttle is a fair hover value.

| Quantity [log] | A mean | A std | A range | B mean | B std |
|---|---|---|---|---|---|
| rcCommand[3] | 1368.6 | 6.8 | 1350 to 1385 | 1369.4 | 6.7 |
| setpoint[3] (0..1000) | 368.6 | 6.8 | 350 to 385 | 369.4 | 6.7 |
| Motor output, mean of 4 (158..2047) | 854 | 13 | 819 to 886 | 856 | 13 |
| Battery voltage, V | 22.60 | 0.13 | 22.23 to 23.00 | 22.59 | 0.13 |
| Current, A | 22.5 | 1.0 | 19.0 to 25.5 | 22.6 | 0.9 |

- One-second window means of rcCommand[3]: 1355 to 1379, std 5.3 [log].
- Per motor [log]: 855, 850, 852, 860. Well balanced. Highest single value in hover: 960 of 2047.
- Rotor speed [log + header `motor_poles` 14]: 122 Hz mean (about 7340 rpm), range 118 to 127 Hz.
- Electrical power: about 510 W [calc]. The current sensor calibration is not verified.

The same hover value in other units:

| Unit | Value | Tag |
|---|---|---|
| rcCommand[3] | 1369 | [log] |
| Mixer throttle | 36.9 % | [log] |
| Raw RC throttle to send with MSP_SET_RAW_RC | about 1400 us | [calc], from `min_check = 1050` [config] and the stock linear mapping [source] |
| Equivalent `ap_hover_throttle` | about 1400 | [calc]. It uses the same scale as raw RC [source] |
| Configured `ap_hover_throttle` | 1550 = 52.6 % mixer throttle | [header], [config], [calc] |

`ap_hover_throttle` is only used by the FC's own altitude/position hold. Our MSP override route does not use it. Our controller needs its own value.

**My opinion:** start near 1400 us raw. Three things would change my mind:

- A hover log with the competition payload. More weight needs more throttle.
- A hover above 2 m. This flight was near the ground, which can lower the throttle needed. I cannot size that from this log.
- A fuller battery. This hover was at 22.6 V under load. A full pack needs less throttle.

## 3. Engage switch and MSP override

**Neither appears in any log.**

- Channel 10 is not logged, so the switch cannot be seen directly [header].
- `flightModeFlags` is 1 while armed and 0 otherwise, in all three logs [log]. Bit 0 is ARM, bit 1 is ANGLE [source]. ANGLE was never on.
- The switch turns on ANGLE (`aux 2 1 5 1800 2100` [config]). ANGLE was never on, so the switch was never on. This is a deduction.
- The MSP OVERRIDE flag sits at bit 43 [source]. Only bits 0..31 are logged. **It will not be visible in future logs either.**

So there are no roll/pitch/throttle commands under override to show.

In a future log, expect this [source]: `rcCommand[0..3]` will carry our MSP values, after the deadband (10 [config]) and the throttle mapping. Bit 1 of `flightModeFlags` will mark the engage.

## 4. ANGLE mode: degrees per stick deflection

**Not measurable. Log 3 was flown in rate mode.** Roll angle followed the integral of the stick, not the stick itself [log].

What stock firmware does [source]: ANGLE mode takes the rate curve, scales it to -1..+1, and multiplies by `angle_limit`. The rate curve here is not linear (`rates_type = ACTUAL`, centre 70 deg/s, max 530 deg/s, expo 0 [header]). So the angle is not linear in the stick either.

Predicted table [calc], with `angle_limit = 30` and deadband 10 [config]:

| Raw RC offset from 1500 | Target tilt |
|---|---|
| 50 us | 0.5 deg |
| 100 us | 1.6 deg |
| 200 us | 5.5 deg |
| 300 us | 11.5 deg |
| 400 us | 19.7 deg |
| 500 us | 30.0 deg |

Formula [source]: `x = (offset - 10) / 490`, `tilt = 30 * (70*x + 460*x*|x|) / 530`.

What the log does confirm: the rate curve itself, for small sticks only. On roll, logged setpoint is 0.98 times the formula, correlation 0.98, for rcCommand between -30 and +36 [log]. Larger deflections were never flown.

Full stick giving 30 deg agrees with `angle_limit`. Half stick gives about 8 deg, not 15. A real ANGLE log would change my mind on any of this.

## 5. Vibration in hover

**The logged gyro is quiet. The accelerometer shows a clear line at rotor speed, about 125 Hz.** Both signals are filtered before logging, so real vibration is higher than these numbers.

Hover window 9.0 to 30.0 s, log 3. All [log].

| Axis | Gyro std, deg/s | Gyro above 100 Hz, deg/s | Acc std, g |
|---|---|---|---|
| Roll / X | 2.93 | 0.54 | 0.077 |
| Pitch / Y | 2.05 | 0.42 | 0.032 |
| Yaw / Z | 0.65 | 0.31 | 0.041 |

On the ground at idle, for comparison [log]: gyro 0.23 / 0.46 / 0.24 deg/s, acc 0.004 / 0.007 / 0.008 g.

Dominant frequencies [log]. The log rate allows up to 500 Hz:

- **Gyro:** most energy is below 20 Hz. That is stick motion and a slow wobble near 11 Hz (0.74 deg/s rms on roll, 0.65 on pitch). A small line sits at 485 to 494 Hz, close to 4 times rotor speed.
- **Acc:** 125 Hz, once per rotor turn. 0.058 g rms on X, 0.017 on Y, 0.015 on Z. Smaller lines at 360 to 372 Hz (3 times rotor speed, about 0.013 g) and 245 Hz (2 times).

Limits of these numbers:

- The gyro is logged after the RPM notch filters, which remove rotor harmonics 1 to 3 [header]. The lines that matter for a rolling shutter are hidden in the gyro trace.
- The accelerometer is logged after a 25 Hz low-pass [config]. A stock filter passes only about 9 % at 125 Hz [calc]. The real 125 Hz vibration on X could be near 0.7 g rms [calc]. This is an estimate, not a measurement.

For the camera: expect vibration lines near 122, 245 and 367 Hz in hover. They move with throttle and battery voltage.

Not needed yet: unfiltered gyro logging. It is off in the config (`blackbox_disable_gyrounfilt = ON`). Turn it on only if camera images show wobble.

## 6. Baro noise and vertical speed

**On the ground the baro is quiet: std 4 to 5 cm. In hover it is 6 times noisier: std 31 cm.**

Stationary, motors idling [log]:

| Window | Std | Peak to peak | Mean |
|---|---|---|---|
| Log 1, 1.0 s | 4.8 cm | 16 cm | -117 cm |
| Log 2, 2.0 s | 4.1 cm | 24 cm | -115 cm |
| Log 3, first 2.1 s | 5.1 cm | 25 cm | -105 cm |

- These windows are short. They do not show slow drift.
- The mean rose 13 cm in 12 s between logs 1 and 3 [log]. The FC had been on for only about a minute, so this is likely warm-up drift.
- The baro value changes about every 50 ms, in steps of about 8 cm [log].

In hover, 8.0 to 30.3 s [log]: std 31 cm, range -66 to +124 cm. Std around a 1 s moving average: 27 cm. GPS altitude over the same time stayed within a 0.8 m band.

The baro is spiky once the motors work. Single updates jumped by more than 1 m seven times in log 3, the largest by 1.4 m [log].

**The baro also shifts with throttle.** On the ground, before lift-off, the reading rose by about 1 m as throttle went from 1000 to 1226 [log]:

| Time | Throttle | Baro mean |
|---|---|---|
| 0 to 2.1 s | 1000 | -105 cm |
| 3.0 to 3.5 s | 1084 | -87 cm |
| 4.0 to 4.5 s | 1146 | -61 cm |
| 5.0 to 5.5 s | 1226 | -5 cm |
| Hover | 1369 | +9 cm |

So the baro "climb" of 1.1 m is mostly prop wash. GPS altitude rose about 0.5 m on average [log]. Real hover height: roughly half a metre, not well measured.

Vertical speed in flight [log]:

- GPS: -41 to +34 cm/s (positive is down), std 13 cm/s.
- Baro, smoothed over 0.5 s: -57 to +55 cm/s. Much of this is noise.

## 7. hold_judge.py on our logs

**It runs without error but cannot score a hold from these CSVs.** Four problems, all checked by running.

Plain run. Exact output:

```
$ hold_judge.py bbd/LOG.03.csv --verbose
columns used: {'time': ' time (us)', 'baro': ' baroAlt', 'switch': None, 'flags': ' flightModeFlags (flags)', 'lat': None, 'lon': None, 'sats': None}
log LOG.03.csv  (betaflight-csv)  GPS fixes 0  baro samples 30639  switch samples 0
no engage found (CH10 >= 1800). Use --engage-at, or check --channel / --bf-engage.
limits: horizontal 3.0 m, vertical 2.0 m, hold 30 s
#      start s   engaged    within    max H    max V   alt m   lvl  qual points  left limits by
```

With the GPS file. Exact output:

```
$ hold_judge.py bbd/LOG.03.csv --gps bbd/LOG.03.gps.csv --verbose
columns used: {'time': ' time (us)', 'baro': ' baroAlt', 'switch': None, 'flags': ' flightModeFlags (flags)', 'lat': None, 'lon': None, 'sats': None, 'gps_file': 'bbd/LOG.03.gps.csv', 'gps_time': 'time (us)', 'gps_lat': ' GPS_coord[0]', 'gps_lon': ' GPS_coord[1]'}
log LOG.03.csv  (betaflight-csv)  GPS fixes 267  baro samples 30639  switch samples 0
no engage found (CH10 >= 1800). Use --engage-at, or check --channel / --bf-engage.
limits: horizontal 3.0 m, vertical 2.0 m, hold 30 s
#      start s   engaged    within    max H    max V   alt m   lvl  qual points  left limits by
```

With `--bf-engage angle` added. Exact output (the `columns used` line is the same as above):

```
log LOG.03.csv  (betaflight-csv)  GPS fixes 267  baro samples 30639  switch samples 30639
limits: horizontal 3.0 m, vertical 2.0 m, hold 30 s
#      start s   engaged    within    max H    max V   alt m   lvl  qual points  left limits by
1        67.25     30.54      0.36     1.15   248.00     0.0     -    no      0  vertical 8.00 m
```

Logs 1 and 2 print the same as the plain run, with 955 and 1982 baro samples.

The pitfalls:

1. **No engage column.** `rcData[9]` does not exist. The tool always says "no engage found".
2. **Baro is read 100 times too large.** `blackbox_decode` names the column ` baroAlt`, with a leading space and no unit. The tool's unit test misses it and takes centimetres as metres. Above, a 2.48 m baro change became "248.00", and the hold "failed" after 0.36 s on an 8 cm change printed as "8.00 m". `--unit-height m` does not help; the decoder does not convert this field [ran].
3. **`--bf-engage angle` triggers on ARM, not ANGLE.** `blackbox_decode` labels bit 0 as `ANGLE_MODE`. In this firmware bit 0 is ARM [source]. Above, the "hold" is the whole armed time, starting on the ground at 67.25 s. This flight never used ANGLE. Real ANGLE (bit 1) would print as `HORIZON_MODE`.
4. **GPS needs `--gps LOG.03.gps.csv`.** Without it: 0 fixes. `--merge-gps` also works but is slow: 13 s instead of 0.3 s for this short log [ran].

Other things to know:

- Times are FC uptime, not time from log start. This log starts at 67.2 s. `--engage-at` must use that clock.
- Units that do work: time in us, GPS in degrees, and GPS in degrees times 1e7 is also handled (read in the tool).
- GPS must be connected and have a fix before arming on judged flights. Logs 1 and 2 have no position. At backup time the FC reported "GPS: NOT CONNECTED" (`status.txt`).
- `blackbox_decode` drops up to 29 frames (29 ms) after each flight-mode change. It kept 955 / 1982 / 30639 rows where `orangebox` kept 984 / 2010 / 30642 [ran]. The log itself has no gaps. Harmless for scoring.
- `blackbox_decode` does not parse the new header style `P interval:8`. Its "87 % iterations missing" message is wrong and can be ignored. Time stamps are correct (they match `orangebox`).

Proof that the tool works once the unit is fixed. I renamed the column to `baroAlt (cm)` in a scratch copy and gave the time by hand:

```
$ hold_judge.py patched/LOG.03.csv --gps patched/LOG.03.gps.csv --verbose --engage-at 76 --disengage-at 96
log LOG.03.csv  (betaflight-csv)  GPS fixes 267  baro samples 30639  switch samples 0
limits: horizontal 3.0 m, vertical 2.0 m, hold 30 s
#      start s   engaged    within    max H    max V   alt m   lvl  qual points  left limits by
1        76.00     20.00     20.00     1.32     1.65     0.6     -    no      5
```

Look at "max V 1.65". GPS altitude stayed within a 0.8 m band, yet baro noise used most of the 2 m limit. The tool takes one raw baro sample as the reference and stops the hold at the first sample outside the limit.

Also, the tool takes take-off altitude from the first baro sample, at idle. The throttle shift in section 6 adds about 1 m to "altitude relative to take-off". The level window is plus or minus 2 m.

Blackbox Explorer export (the route the judges' README names): **not run, only read.** From its source, the CSV starts with header lines before the column row, flags are plain numbers, and `baroAlt` has no leading space. So pitfall 2 would go away, the angle fallback would never match, and the tool may not find the column row at all. Someone should export once and try it.

## 8. Surprising or dangerous

From the log:

1. **No failsafe.** `failsafePhase` is 0 in every frame. RX signal valid throughout [log].
2. **No desync signs.** Every motor's eRPM follows its command. Largest 5 ms eRPM drop in flight: 1300 of about 51000 [log].
3. **The flight was in rate mode, not ANGLE,** and arming needs no ANGLE.
4. **Motors keep running 90 to 110 ms after the arm switch goes off.** Stock firmware waits for 4 RC frames in a row [source]. Seen in all three logs [log].
5. **Battery was partly used:** 23.3 V at rest, 22.6 V in hover. At the touchdown bump it dipped to 21.9 V with a 53 A peak [log].
6. **The pilot switched off 20 ms after the touchdown bump,** with throttle still at 1368 [log]. It reads as a planned landing, not a crash.

Read from the config, not from the log. I did not test these. Someone who knows the setup should confirm they are intended:

- `failsafe_delay = 200`. Stock units are 0.1 s, so 20 s. Stock default is 1.5 s.
- `failsafe_procedure = DROP` and `rxfail 3 h` (hold throttle during the delay).
- `msp_override_failsafe = OFF`.
- `small_angle = 180`: arming is allowed at any tilt.
- `vbat_min_cell_voltage = 250` (2.5 V per cell) and warning at 2.7 V. Low-battery warnings come very late.
- `status.txt` shows arming blocked by RXLOSS, CLI, DSHOT_TELEM. That is normal on USB with no battery.

## How to repeat

Scratch output is in a temp folder and may be deleted:
`C:\Users\hommula\AppData\Local\Temp\claude\a9fae5c573224634\ce9a7164-ef84-4c32-acbf-00bf7fd20a23\scratchpad\blackbox`

1. Cut the used part of the chip (bytes 0 to 1,146,292) into `LOG.bbl`. Logs start at byte 0, 38,912 and 110,592.
2. Build the decoder in the `vioh-dev` container: `make obj/blackbox_decode` in a clone of `betaflight/blackbox-tools`. The container has no `git`; clone on the host.
3. Decode: `blackbox_decode LOG.bbl`. Add `--unit-flags raw` to get flag numbers instead of wrong names.
4. Judge: `python hold_judge.py LOG.03.csv --gps LOG.03.gps.csv --verbose`.

`pip install orangebox` fails on a packaging error. Unzipping the wheel works.
