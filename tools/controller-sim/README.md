# controller-sim

The list of test cases and what counts as a pass is in [TEST_CASES.md](TEST_CASES.md).

Host tests and a closed loop simulation for `lib/brewpi-pid`. It compiles the library with the host C++
compiler against small stubs of Arduino, ArduinoJson, log and jsonfs (`stub/`), so no board is needed.

It was built to check the rewrite from fixed point to floating point and the 32 bit tick fix. It is kept so
that later changes to the controller can be checked the same way.

## Running

```
tools/controller-sim/run.sh test              # unit tests + closed loop regression check, exit code 0 = fine
tools/controller-sim/run.sh compare [rev]     # working tree against lib/brewpi-pid at a git revision (default HEAD)
tools/controller-sim/run.sh update-expected   # rewrite expected/metrics.txt from the working tree
```

Needs `c++` (C++17) and `python3`. Nothing is installed and nothing outside `tools/controller-sim/build/`
is written (ignored by git). Environment: `CXX`, `SIM_BUILD` (build dir), `SIM_LIB` (test another copy of
the library, used to check that the tests catch a broken library).

## What `test` checks

| Check | What it proves |
| --- | --- |
| filter against the exact solution | The cascaded IIR filter (b = 1..6) matches an exact decimal (`Decimal`) evaluation of the same recurrence to 1e-6 °C. Measured: about 5e-10. |
| `sensor_test` | Filters start at the first valid reading, a short disconnect keeps the history, more than 60 failed reads in a row restart the filters. Memory is poisoned before init. |
| `format_test` | `parseBytes` (sensor addresses) rejects odd length, too long, trailing characters, non hex, null; `roundTo` rounding. |
| `ticks_test` | `secondsSince` and `wait.seconds` across 16 bit second counter limits (65536 s), 25 h uptime and the `millis()` rollover. |
| `chamberreport_test` | The BrewGraph poll without HTTP: request body, reading the reply, deciding what to apply (needs the real ArduinoJson from `.pio/libdeps`). |
| closed loop vs `expected/metrics.txt` | 26 scenarios (24 h of simulated time each, 6 h for the fridge ones) on a simple thermal model: switch counts, duty cycles, time to reach the target, mean deviation, estimator values, time per state. 442 metrics. |
| actuator rules | `nh_` = no heater, `nc_` = no cooler: the controller never enters a heating (nh_) or cooling (nc_) state. This is what a device with that actuator disabled in the config does: `set...Actuator` is not called, the controller keeps its default actuator and `TempControl::updateState` skips those states. |
| control | Every configuration that can reach its target does (target reached, mean deviation within the limits in `scripts/check_control.py`). |

The scenarios cover the device configurations:

| Prefix | Device | Scenarios |
| --- | --- | --- |
| (none) | heater + cooler, fridge + beer probe | fridge_cool, beer_cool, beer_heat, beer_step, noisy variants, fridge_disconnect |
| `nh_` | cooler only | fridge_cool, beer_cool(_noisy), beer_step, fridge_disconnect, beer_warm and beer_cold_room (cannot be reached, must idle) |
| `nc_` | heater only | fridge_heat, beer_heat(_noisy), beer_step, fridge_disconnect, beer_cool (cannot be reached, must idle) |
| `fp_` | heater + cooler, fridge probe only | fridge_cool, fridge_heat, beer_cool (beer mode, the probe in the fridge air is used as the beer probe) |
| `bp_` | heater + cooler, beer probe only | beer_cool, beer_heat, fridge_cool (fridge mode, the beer probe is used as the fridge probe) |

With a single probe TempControl uses it for both roles (`beerControl()` / `fridgeControl()`), the display and
the getters only show the probes that exist. Before that change a device with a beer probe only, or a
fridge probe only in beer mode, stayed idle for ever (default sensors are never connected).
`fp_beer_cool` has the probe in the fridge air, so the +-1 C idle band of the fridge applies: mean deviation
0.90, the limit for it is 1.0 (the check first said 0.5, that was wrong for this placement, see
`check_control.py`). `bp_beer_cool` heats now and then (4 % duty cycle, 21 switches in 24 h) against a
compressor: the beer probe cannot see the fridge air, so the fridge side of the PID works on a delayed value.

Tolerances are in `scripts/metrics.py`. The time spent per state is judged with 12 % of the run in
`check_expected.py`, because in `beer_heat` it is bimodal: the split between IDLE and WAITING_TO_COOL depends
on a fridge overshoot that lands almost exactly on the idle limit.

If you change the control behaviour on purpose, look at what moved, then run `update-expected` and
review the diff of `expected/metrics.txt` in the commit.

## What `compare` does

Two different questions, kept apart on purpose:

1. **Summary metrics under perturbation.** Both libraries run every scenario at 7 start temperature offsets
   (-0.20 .. +0.20). A closed loop with a state machine is chaotic in the details (one threshold crossing a
   sample earlier shifts everything after it), so single runs are not comparable, the metrics over several
   runs are. A metric passes if it is within the tolerance of the reference, or within the reference's own
   spread over the 7 runs.
2. **Replay of identical sensor input.** The readings the reference saw are recorded and fed to both
   controllers with the loop open. Any difference is then numerical. `compare_replay.py` reports the maximum
   difference of the filtered temperatures, slope, P/I/D and fridge setting while the controller state is
   still identical, and for every state divergence the margin to each threshold the state machine compared.

## Rules that matter

- **One process per scenario.** The controller keeps state between runs (PID integral, a static counter in
  `updatePID`). `run.sh` sets `SIM_ONLY` per scenario. Running them in one process gives wrong numbers.
- Decide the acceptance criteria before looking at the result, and report misses instead of widening them
  afterwards.

## Reading a `compare` against the pre floating point revision

Known and expected when the reference is the fixed point library (`60ff56d`, the last commit before the rewrite):

- `fp_beer_cool`, `bp_beer_cool`, `bp_beer_heat`, `bp_fridge_cool` fail (33 metrics): the old code never ran there
  (idle for ever). All other 22 scenarios have no failed metric.
- `beer_heat t_IDLE`: passes only through the reference's own spread.
- Replay `fsP` (fridge setting) is 0.013 to 0.023 against the criterion of 0.01. The fixed point reference is
  quantised to 1/512 and P multiplies that by Kp = 5, so 0.02 to 0.03 is what the reference itself carries.
  Not a defect of the new code.
- Replay `nh_beer_step` (no heater), `i` and `fsP` 0.09: the integrator has two branches on its sign
  (`diffIntegral > 0`). The old code's rounding had put its integral 0.03 above the new one, so at t = 5130 s
  the old integral was +0.023 and the new -0.010. Old: the "fridge is 2 C from the setting" rule zeroed the
  update. New: the signs differ, the update is doubled (+0.38). The integral then stays 0.35 apart until the
  states diverge anyway. A threshold crossing like the state machine ones, only in the integrator. The summary
  metrics of all 7 perturbations of that scenario are within tolerance.
- Replay `fridge_disconnect` / `nh_fridge_disconnect` `fF` 2.5 °C at t = 7920 s: the sensor is back after a 12 minute disconnect. The
  new code restarts the filters at the new reading (after more than 60 failed reads), the reference blends
  it into the old history. The two agree to 0.001 °C at the end of the run.

## Limits

- The sensors run the filter coefficients of the control constants (fridge 1/4/3, beer 3/4/4), as on the device.
  Those are made for a beer that follows the fridge in hours: with `kBeerTau` = 1800 s the loop oscillates.
- One simple thermal model (fridge, beer, ambient, first order), no real sensor noise spectrum, no BLE or
  OneWire, no sensor timing. It shows that the numerics and the control logic behave as before, not that the
  tuning is good for a given chamber.
- Not run on hardware.
- The stubs only implement what the library uses. A library change that needs more of Arduino needs the stub
  extended.

## Layout

```
run.sh            entry point
src/              sim.cpp (closed loop, record/replay), filter_test, sensor_test, format_test, ticks_test
stub/             Arduino.h (virtual clock), ArduinoJson.h, log.hpp, jsonfs.hpp
scripts/          metrics.py (parse + tolerances), check_expected, compare_metrics, compare_replay,
                  filter_check (exact decimal check), variability
expected/         metrics.txt, the reference numbers for `test`
```
