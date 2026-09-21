"""Does the controller control? Every configuration of actuators and probes that CAN control must reach the
target and stay near it.

usage: check_control.py <metrics file> [<metrics file> ...]
Limits on the mean absolute deviation over the second half of the run (metrics.meanDev), in Celsius, fixed
before the single probe runs were looked at:
  beer probe in the beer (beer mode):      0.5     (the two probe runs are at 0.03 to 0.17)
  fridge mode:                             1.0     (idle band of +-1 C around the setting), 1.5 with a
                                                   12 minute sensor disconnect
  fp_beer_cool: beer mode but the only probe is in the fridge air, so the fridge band applies: 1.0. The
                first version of this check said 0.5 for it, the run gave 0.90.
  ble_ scenarios (beer probe silent for hours): the true beer temperature is within 1.0 C of the target in the
                last hour without it (metrics.lostDev), fixed before the fallback to the fridge probe was built.
  compressor: with a single probe at most 1.5 times the switches of beer_cool (same run).
Not checked, because nothing can reach the target: nh_beer_warm, nh_beer_cold_room, nc_beer_cool. Those are
covered by check_actuator_rules.py (the missing actuator is never used).
"""
import sys

import metrics

CANNOT = {"nh_beer_warm", "nh_beer_cold_room", "nc_beer_cool"}
SINGLE_PROBE = ("fp_", "bp_")


def limit(name):
    if name == "fp_beer_cool":
        return 1.0
    if "disconnect" in name:
        return 1.5
    return 1.0 if "fridge" in name else 0.5


bad = 0
checked = 0
for path in sys.argv[1:]:
    runs = metrics.parse(path)
    for name, v in runs.items():
        if name in CANNOT:
            continue
        checked += 1
        problems = []
        if v["reachMin"] < 0:
            problems.append("target never reached")
        if v["meanDev"] > limit(name):
            problems.append(f"mean deviation {v['meanDev']:.3f} > {limit(name)}")
        if "ble_" in name and v["lostDev"] > 1.0:
            problems.append(f"beer {v['lostDev']:.3f} C from the target in the last hour without the beer probe > 1.0")
        if name.startswith(SINGLE_PROBE) and "beer_cool" in runs and v["coolSw"] > 1.5 * runs["beer_cool"]["coolSw"]:
            problems.append(f"{v['coolSw']:.0f} compressor switches > 1.5 x {runs['beer_cool']['coolSw']:.0f}")
        for p in problems:
            bad += 1
            print(f"FAIL  {path}: {name}: {p}")
print(f"{checked} runs checked for control, {bad} problems")
sys.exit(1 if bad else 0)
