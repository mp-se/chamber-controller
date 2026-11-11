"""Shared by the comparison scripts: parsing of the summary lines printed by `sim` and the tolerances.

A line looks like
  beer_cool  coolSw= 94 heatSw=  0 cool%= 22.0 heat%=  0.0 reachMin=  76.6 meanDev= 0.046  heatEst= ... coolEst= ...  states(s): 41533 0 0 ...
"""
import re

KEYS = ["coolSw", "heatSw", "cool%", "heat%", "reachMin", "meanDev", "lostDev", "heatEst", "coolEst"]
STATES = ["IDLE", "OFF", "HEAT", "COOL", "WAIT_COOL", "WAIT_HEAT", "WAIT_PEAK", "COOL_MIN", "HEAT_MIN"]
HOURS = {"fridge_cool": 6, "fridge_disconnect": 6, "nh_fridge_cool": 6, "nh_fridge_disconnect": 6, "nc_fridge_heat": 6, "nc_fridge_disconnect": 6, "fp_fridge_cool": 6, "fp_fridge_heat": 6, "bp_fridge_cool": 6}  # every other scenario runs 24 hours


def parse(path):
    """{scenario: {metric: value}}, the seconds spent in each state are called t_<STATE>."""
    runs = {}
    for line in open(path):
        m = re.match(r"^(\S+)\s+(coolSw=.*)$", line)
        if not m:
            continue
        name, rest = m.group(1), m.group(2)
        values = {}
        for key in KEYS:
            values[key] = float(re.search(re.escape(key) + r"=\s*(-?[0-9.]+)", rest).group(1))
        for i, seconds in enumerate(int(x) for x in rest.split("states(s):")[1].split()):
            values["t_" + STATES[i]] = seconds
        runs[name] = values
    return runs


def run_seconds(name):
    return HOURS.get(name, 24) * 3600


def tolerance(name, key, reference, state_fraction=0.03):
    """How far a metric may be from its reference and still count as the same behaviour.

    Chosen from how much the fixed point controller moves by itself when the start temperature is
    perturbed by +-0.2 C: switches +-1..3, duty cycle <= 0.3 points, mean deviation <= 0.003 C,
    beer_step time to target sd 6 min. State times get state_fraction of the run.
    """
    if key in ("coolSw", "heatSw"):
        return max(3, 0.10 * reference)
    if key in ("cool%", "heat%"):
        return 1.0
    if key == "reachMin":
        return max(15, 0.25 * reference) if name.endswith("beer_step") else max(5, 0.10 * reference)
    if key == "meanDev":
        return max(0.03, 0.15 * reference)
    if key == "lostDev":
        return 0.5
    if key == "heatEst":
        return 0.15 if name.endswith("beer_step") else max(0.05 * reference, 0.05)
    if key == "coolEst":
        return 2.0 if name.endswith("beer_step") else max(0.05 * reference, 0.05)
    if key.startswith("t_"):
        return state_fraction * run_seconds(name)
    raise KeyError(key)


def all_keys():
    return KEYS + ["t_" + s for s in STATES]
