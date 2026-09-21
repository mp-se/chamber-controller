"""Hard rules for the scenarios of a device without heater (nh_) or without cooler (nc_).

usage: check_actuator_rules.py <metrics file> [<metrics file> ...]
Without a heating actuator TempControl::updateState skips the heating states, without a cooling actuator the
cooling states. So in every nh_ run the seconds in HEAT, WAIT_HEAT and HEAT_MIN, the heat switches and the heat
duty cycle must be 0, in every nc_ run the same for cooling.
"""
import sys

import metrics

RULES = {
    "nh_": ("heatSw", "heat%", "t_HEAT", "t_WAIT_HEAT", "t_HEAT_MIN"),
    "nc_": ("coolSw", "cool%", "t_COOL", "t_WAIT_COOL", "t_COOL_MIN"),
}

bad = 0
checked = 0
for path in sys.argv[1:]:
    for name, values in metrics.parse(path).items():
        for prefix, keys in RULES.items():
            if not name.startswith(prefix):
                continue
            checked += 1
            for key in keys:
                if values[key] != 0:
                    bad += 1
                    print(f"FAIL  {path}: {name} {key} = {values[key]}, must be 0")
print(f"{checked} runs without heater or cooler checked, {bad} broke the rule")
sys.exit(1 if bad else 0)
