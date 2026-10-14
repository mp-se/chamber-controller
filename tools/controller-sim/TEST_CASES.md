# Test cases of the controller simulator

What `tools/controller-sim/run.sh test` and `run.sh compare` check, case by case, and what counts as a pass.
`README.md` explains how to run it and how to read a result; this file is the list of cases.

Everything here runs on the host, without a board, against `lib/brewpi-pid`. Numbers in the tables are the
current baseline (`expected/metrics.txt`), not hand written.

## 1. Unit tests

### 1.1 Filter against the exact solution (`filter_test.cpp` + `scripts/filter_check.py`)

| ID | Case | Pass |
| --- | --- | --- |
| F1-F6 | The cascaded IIR filter with coefficient b = 1, 2, 3, 4, 5, 6 is fed 3000 samples of a test signal and compared with an exact decimal (`Decimal`) evaluation of the same recurrence | largest difference < 1e-6 C (measured about 5e-10) |

### 1.2 Sensor filters (`sensor_test.cpp`)

| ID | Case | Pass |
| --- | --- | --- |
| S1 | First reading arrives after `init()`, memory poisoned with 0xA5: fast and slow filter | both equal the first reading (20) |
| S2 | 20 more readings of the same value | slow filter stays at 20 |
| S3 | Constant input | slope is 0 |
| S4 | Short disconnect: 40 + 40 failed reads, never 60 in a row, then a new value (25) | slow filter keeps its history (still 20, not restarted at 25) |
| S5 | Long disconnect: 70 failed reads in a row, then 25 | fast and slow filter restart at 25 |

### 1.3 Parsing and rounding (`format_test.cpp`)

| ID | Case | Pass |
| --- | --- | --- |
| P1 | `parseBytes` 16 upper case hex digits / 16 lower case | 8 bytes, accepted |
| P2 | Round trip with `printBytes` | same string |
| P3 | Odd length ('1') | rejected, buffer cleared |
| P4 | 48 hex digits into an 8 byte buffer | rejected, nothing written past 8 bytes |
| P5 | 18 digits (a trailing byte) | rejected |
| P6 | Non hex character / empty string / null | rejected |
| P7 | `roundTo` 19.55 to 1 decimal | 19.6 |
| P8 | `roundTo` -3.25 to 1 decimal | -3.3 (half away from zero) |
| P9 | `roundTo` 1.23456 to 2 and 3 decimals | 1.23 and 1.235 |
| P10 | `roundTo` with 0 and with 4 decimals | 3 decimals |
| P11 | `roundTo` a whole number, NAN, a tiny negative number | unchanged, 0, 0 |

### 1.4 Time arithmetic (`ticks_test.cpp`)

| ID | Case | Pass |
| --- | --- | --- |
| T1 | `secondsSince` 5 s ago | 5 |
| T2 | Never cooled since boot (timestamp 0), uptime 5 h / 18 h 12 m 21 s / 25 h | 18000 / 65541 / 90000 (no wrap of the old 16 bit counter at 65536 s) |
| T3 | Idle for 19 h 27 m, and for exactly 65536 s | 70020, 65536 |
| T4 | Under a second (1999 ms) | 1 |
| T5 | `millis()` rollover: 9095 ms across it, and a timestamp 1 ms before it | 9, 1 |
| T6 | `wait.seconds(2)` and `wait.seconds(70)` | `delay(2000)` and `delay(70000)` |

### 1.5 BrewGraph poll (`chamberreport_test.cpp`)

The part of the poll that has no HTTP and no globals (`src/chamberreport.cpp`), built against the real
ArduinoJson from `.pio/libdeps` (run the firmware build once with `pio`, or set `ARDUINOJSON`). Limits for the
decision are 1 ... 30 C, like the controller.

| ID | Case | Pass |
| --- | --- | --- |
| C1 | Request with both temperatures, mode b | `token`, `beer_temperature`, `fridge_temperature`, `temp_units` C, `current_mode` B, `current_target`, `rssi` |
| C2 | No token configured | `id` (chip id) is sent instead, no `token` |
| C3 | Beer sensor silent (fridge only), or beer only | the missing temperature is left out |
| C4 | No temperature at all | not built (the server would answer 422) |
| C5 | Mode b, f, o, and test mode t | `current_mode` B, F, O; t is left out |
| C6 | No target | `current_target` left out |
| C7 | 19.3 C | written as `19.3`, not `19.299999...` |
| C8 | Reply B / F with a target; lower case mode | mode B / F, target read; `b` accepted |
| C9 | Reply of an idle device: `{"target_temperature":null,"temp_units":"C","mode":"R"}` | mode R, target NAN |
| C10 | Target sent as a string | reply accepted, target NAN |
| C11 | Mode `O`, `BF`, missing, not a string | rejected |
| C12 | `{}` (the framework's answer to a failed request), HTML from a proxy, empty, null | rejected |
| C13 | Reply R, device not under remote control | nothing (the steady state: no config write) |
| C14 | Reply R while under remote control | restore |
| C15 | Reply B or F, not under remote control (also with the same values as the local setting) | apply, the device takes control |
| C16 | Reply equal to the device state under remote control (within 0.05 C) | nothing |
| C17 | 0.1 C different target, or a different mode with the same target, or the device has no target | apply |
| C18 | Target NAN, 0.5 (< 1), 31 (> 30) | ignored |
| C19 | Target exactly 1 and 30 | apply |

## 2. Closed loop scenarios

The controller (`TempControl`, `TempSensor`, PID, state machine) runs against a thermal model, one step per
second, boot at 5 s. One process per scenario, because the controller keeps state (PID integral, a static
counter in `updatePID`).

Model: fridge air `Tf += q + (Ta - Tf)/3000 + (Tb - Tf)/2500`, beer `Tb += (Tf - Tb)/1800 + fermentation`,
where `q` is -0.016 C/s while the compressor runs (about 1 C/min) and +0.008 C/s while the heater runs
(about 0.5 C/min). Sensors round to 1/16 C like a DS18B20; the noisy scenarios add +-0.03 C of deterministic
noise. Fridge and beer start at the start temperature. A lost probe gives no reading (like a BLE sensor
that is out of range).

Columns: room = ambient temperature, hours = simulated time, `coolSw`/`heatSw` = times the actuator was
switched on, reach = minutes until the controlled temperature is within 0.3 C of the target (after the step
for the step scenarios, `never` = not reached), meanDev = mean |controlled - target| over the second half of
the run, lostDev = largest beer deviation in the last hour without the beer probe. "Controlled" is what the
controller's probe measures: the fridge in fridge mode, the beer in beer mode, and with a single probe that
probe.

Prefixes: `nh_` no heater, `nc_` no cooler, `ble_` beer probe outage, `fp_` fridge probe only, `bp_` beer probe
only, no prefix = heater and cooler, both probes.

| Scenario | Device | Mode, target C | Room / start C | Hours | Event | What it shows | coolSw / heatSw | reach (min) | meanDev C | lostDev C |
| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |
| `fridge_cool` | heater + cooler | fridge, 10 | 22 / 20 | 6 | - | Cools the fridge from 20 to 10 and holds it inside the +-1 C idle band. | 26 / 0 | 38.7 | 0.88 | 0.00 |
| `beer_cool` | heater + cooler | beer, 12 | 22 / 20 | 24 | - | Cools the beer from 20 to 12 (fermentation heat, ambient 22) and holds it. | 94 / 0 | 76.8 | 0.05 | 0.00 |
| `beer_heat` | heater + cooler | beer, 18 | 10 / 10 | 24 | - | Heats the beer from 10 to 18 in a cold room (ambient 10) and holds it. | 0 / 91 | 101.1 | 0.03 | 0.00 |
| `beer_step` | heater + cooler | beer, 15 | 20 / 20 | 24 | target 19 at 8 h | Beer held at 15, target raised to 19 after 8 h: heats to the new target, then holds it. | 47 / 45 | 50.3 | 0.17 | 0.00 |
| `beer_cool_noisy` | heater + cooler | beer, 12 | 22 / 20 | 24 | noise +-0.03 | Same as beer_cool with +-0.03 C sensor noise: no chattering, same result. | 86 / 0 | 76.8 | 0.06 | 0.00 |
| `fridge_disconnect` | heater + cooler | fridge, 8 | 22 / 18 | 6 | fridge probe lost 120 min, 12 min | Fridge sensor gone for 12 minutes after 2 h: idles, then resumes control when it is back (filters restart at the new reading). | 24 / 0 | 40.6 | 1.11 | 0.00 |
| `beer_heat_noisy` | heater + cooler | beer, 20 | 8 / 12 | 24 | noise +-0.03 | Heats 12 to 20 with sensor noise. | 0 / 99 | 113.4 | 0.04 | 0.00 |
| `nh_fridge_cool` | cooler only | fridge, 10 | 22 / 20 | 6 | - | No heater: same as fridge_cool; never enters a heating state. | 26 / 0 | 38.7 | 0.88 | 0.00 |
| `nh_beer_cool` | cooler only | beer, 12 | 22 / 20 | 24 | - | No heater: cools and holds; never enters a heating state. | 98 / 0 | 76.8 | 0.04 | 0.00 |
| `nh_beer_cool_noisy` | cooler only | beer, 12 | 22 / 20 | 24 | noise +-0.03 | No heater, noisy sensors. | 92 / 0 | 76.8 | 0.05 | 0.00 |
| `nh_beer_warm` | cooler only | beer, 18 | 10 / 10 | 24 | - | Target 18 in a 10 C room, nothing to heat with: cannot be reached, the controller must idle and not use the missing heater. | 0 / 0 | never | 7.92 | 0.00 |
| `nh_beer_cold_room` | cooler only | beer, 12 | 8 / 14 | 24 | - | Cooled to 12 in an 8 C room, then drifts colder: cannot be corrected, must idle. | 1 / 0 | 24.9 | 3.92 | 0.00 |
| `nh_beer_step` | cooler only | beer, 15 | 20 / 20 | 24 | target 19 at 8 h | Target raised from 15 to 19 in a 20 C room: reached by drifting, no heating state. | 27 / 0 | 151.2 | 0.37 | 0.00 |
| `nh_fridge_disconnect` | cooler only | fridge, 8 | 22 / 18 | 6 | fridge probe lost 120 min, 12 min | No heater, 12 minute fridge sensor loss. | 24 / 0 | 40.6 | 1.11 | 0.00 |
| `ble_beer_cool` | heater + cooler | beer, 12 | 22 / 20 | 24 | beer probe lost from min 480, 360 min | Beer sensor (BLE) silent for 6 h from hour 8: the chamber is controlled on the chamber sensor and the beer stays near 12; PID takes over when the sensor is back. | 92 / 0 | 76.8 | 0.15 | 0.67 |
| `ble_beer_heat` | heater + cooler | beer, 18 | 10 / 10 | 24 | beer probe lost from min 480, 360 min | Same for heating (target 18, room 10). | 0 / 96 | 101.1 | 0.06 | 0.21 |
| `ble_beer_boot` | heater + cooler | beer, 12 | 22 / 20 | 24 | beer probe lost from min 0, 240 min | BLE sensor not heard during the first 4 h after boot: chamber sensor controls (fridge aimed at the beer target), then PID takes over. | 96 / 0 | 158.4 | 0.05 | 0.32 |
| `nh_ble_beer_cool` | cooler only | beer, 12 | 22 / 20 | 24 | beer probe lost from min 480, 360 min | BLE outage with a cooler only. | 94 / 0 | 76.8 | 0.21 | 0.99 |
| `nc_ble_beer_heat` | heater only | beer, 18 | 10 / 10 | 24 | beer probe lost from min 480, 360 min | BLE outage with a heater only. | 0 / 98 | 101.1 | 0.07 | 0.29 |
| `nc_fridge_heat` | heater only | fridge, 18 | 8 / 10 | 6 | - | No cooler: fridge from 10 to 18 in an 8 C room; never enters a cooling state. | 0 / 29 | 39.0 | 0.51 | 0.00 |
| `nc_beer_heat` | heater only | beer, 18 | 10 / 10 | 24 | - | No cooler: heats beer 10 to 18 and holds it. | 0 / 93 | 101.1 | 0.03 | 0.00 |
| `nc_beer_heat_noisy` | heater only | beer, 20 | 8 / 12 | 24 | noise +-0.03 | No cooler, noisy sensors, 12 to 20. | 0 / 99 | 113.4 | 0.04 | 0.00 |
| `nc_beer_cool` | heater only | beer, 12 | 22 / 20 | 24 | - | Target 12 in a 22 C room, nothing to cool with: cannot be reached, must idle. | 0 / 0 | never | 10.16 | 0.00 |
| `nc_beer_step` | heater only | beer, 15 | 8 / 10 | 24 | target 19 at 8 h | No cooler: target raised from 15 to 19 after 8 h, heats to it. | 0 / 101 | 76.8 | 0.05 | 0.00 |
| `nc_fridge_disconnect` | heater only | fridge, 18 | 8 / 12 | 6 | fridge probe lost 120 min, 12 min | No cooler, 12 minute fridge sensor loss. | 0 / 29 | 32.5 | 0.51 | 0.00 |
| `fp_fridge_cool` | fridge probe only | fridge, 10 | 22 / 20 | 6 | - | Only a fridge probe, fridge mode: works like the two probe case. | 26 / 0 | 38.7 | 0.88 | 0.00 |
| `fp_fridge_heat` | fridge probe only | fridge, 18 | 8 / 10 | 6 | - | Only a fridge probe, heating. | 0 / 29 | 39.0 | 0.51 | 0.00 |
| `fp_beer_cool` | fridge probe only | beer, 12 | 22 / 20 | 24 | - | Only a fridge probe, beer mode: the fridge probe is used as the beer probe. Reaches the target, +-1 C band of the fridge air applies. | 112 / 0 | 52.2 | 0.90 | 0.00 |
| `bp_beer_cool` | beer probe only | beer, 12 | 22 / 20 | 24 | - | Only a beer probe, beer mode: the beer probe is used for both roles. | 54 / 21 | 83.7 | 0.39 | 0.00 |
| `bp_beer_heat` | beer probe only | beer, 18 | 10 / 10 | 24 | - | Only a beer probe, heating. | 0 / 77 | 94.7 | 0.06 | 0.00 |
| `bp_fridge_cool` | beer probe only | fridge, 10 | 22 / 20 | 6 | - | Only a beer probe, fridge mode: the beer probe is used as the fridge probe. | 8 / 0 | 49.6 | 0.52 | 0.00 |

The three scenarios that cannot reach their target by physics (`nh_beer_warm`, `nh_beer_cold_room`,
`nc_beer_cool`) are there to show that the controller idles and does not use the actuator it does not have.

## 3. Checks on the scenarios and what passes

All checks run in `run.sh test` on the unperturbed run. In `run.sh compare` the actuator rule and the control
check also run on every perturbed run (7 start temperature offsets, -0.20 ... +0.20 C).

| ID | Check | Applies to | Pass |
| --- | --- | --- | --- |
| R1 | Regression against `expected/metrics.txt` (`check_expected.py`) | all 31 scenarios, 18 metrics each (558) | every metric within its tolerance (`metrics.py`): switches max(3, 10 %), duty cycle 1 point, reach max(5 min, 10 %) (15 min / 25 % for the step scenarios), meanDev max(0.03, 15 %), lostDev 0.5 C, estimators 5 % (min 0.05; step scenarios 0.15 heat, 2.0 cool), state times 12 % of the run |
| R2 | Actuator rule (`check_actuator_rules.py`) | `nh_` and `nc_` | `nh_`: 0 s in HEAT, WAIT_HEAT and HEAT_MIN, 0 heat switches, 0 % heat. `nc_`: the same for cooling |
| R3 | Control (`check_control.py`) | every scenario except the three that cannot reach their target | target reached; meanDev <= 0.5 C in beer mode, <= 1.0 C in fridge mode (1.5 C with a 12 minute sensor loss); `fp_beer_cool` <= 1.0 C (the probe is in the air, the fridge band applies); `ble_` lostDev <= 1.0 C; single probe scenarios at most 1.5 x the compressor switches of `beer_cool` |

R3 limits were fixed before the single probe and BLE outage runs were looked at. `fp_beer_cool` (0.90 C
against the 0.5 C first written down) is the one place where a limit was changed after seeing the result, and
the reason is in `check_control.py`. `nh_ble_beer_cool` (lostDev 0.99 against 1.0) passes with little margin.

## 4. Comparison with another revision (`run.sh compare [rev]`)

| ID | Case | Pass |
| --- | --- | --- |
| X1 | Summary metrics of all scenarios at 7 perturbed start temperatures, revision `rev` against the working tree | a metric passes if the unperturbed value is within tolerance of the reference, or inside the range the reference covers itself over its 7 runs (widened by the tolerance). Tolerances as in R1, but state times 3 % of the run |
| X2 | Replay: the sensor readings the reference saw are recorded and fed to both controllers with the loop open | while the controller state is identical: filtered temperatures within 0.005 C, slope 0.01 C/h, P, I, D 0.02, fridge setting 0.01. Every state divergence is listed with the margin to each threshold |

Against `60ff56d` (the fixed point code) the expected differences are: the four single probe scenarios and
the five BLE outage scenarios (the old code idles there), the fridge setting rounding of the old code, and the
integrator sign branch in `nh_beer_step`. All 22 other scenarios have no failed metric. See `README.md`.

## 5. Deliberate failure checks

To see that the checks can fail, the library was broken on a copy and `run.sh test` run against it
(`SIM_LIB=<copy>`). Each must exit 1.

| Change to the copy | Caught by | Result |
| --- | --- | --- |
| `Kp` 5 -> 0.5 | R1 | exit 1 |
| `Ticks::secondsSince` divides by 1024 | T-cases | exit 1 |
| `parseBytes` accepts trailing characters | P-cases (two failed) | exit 1 |
| heating states allowed without a heating actuator | R1, R2 | exit 1 |
| a missing probe no longer replaced by the other one | R3 | exit 1 |
| controller idles when the beer probe is silent | R1, R3 | exit 1 |
| `decideChamberAction`: limit check removed | C18 | exit 1 |
| `decideChamberAction`: R restores when not under remote control | C13 | exit 1 |
| `parseChamberReply`: accepts mode O | C11 | exit 1 |
| `buildChamberPayload`: sends the token as `id` | C1 | exit 1 |

## 6. Not covered by the simulator

- Real hardware: relays, compressor protection, OneWire timing (`requestTemperaturesByAddress` blocks), BLE
  reception. One thermal model only; it shows that the control logic behaves as before and in each
  configuration, not that the tuning fits a given chamber.
- Two probes where one fails for good (only the beer probe outage and the 12 minute fridge probe loss are run).
- Beer probe and fridge probe outage at the same time (the controller idles, by design).
- The HTTP part of the poll (`PidPush::pollChamber`), the 15 minute timer, and `applyRemoteCommand`/`applyChamberReply`
  (they use the config, the controller and the web server, not the host). `/api/remote` was refactored to use the
  same function and is not run either; both need the test device.
- The rule that a BLE beer sensor needs a chamber sensor (`configureTempControl`, the two mode handlers in
  `pidwebserver.cpp`): it is only built, not run. The UI side is covered by four tests in
  `ui/src/views/__tests__/DeviceHardwareView.test.js`.
- Filter coefficients from the constants (finding 1: only the placeholder default sensors get them). The sensors in
  the simulator run with the default b = 2, like the real ones, so the simulator matches the device, but a
  device with the constants applied would behave differently and is not tested.
