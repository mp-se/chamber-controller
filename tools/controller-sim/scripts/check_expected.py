"""Regression check: do the summary metrics of a run still match expected/metrics.txt?

usage: check_expected.py <expected metrics file> <actual metrics file>
The tolerances are the ones of metrics.py. The time spent per controller state is judged with 12 % of
the run, because for some scenarios it is bimodal (beer_heat: the split between IDLE and
WAITING_TO_COOL depends on a fridge overshoot that lands almost exactly on the idle limit).
If a change of control behaviour is intended, regenerate the file with `run.sh update-expected`.
"""
import sys

import metrics

expected = metrics.parse(sys.argv[1])
actual = metrics.parse(sys.argv[2])
failures = 0

for name, ref in expected.items():
    if name not in actual:
        print(f"MISSING  {name}")
        failures += 1
        continue
    for key in metrics.all_keys():
        tol = metrics.tolerance(name, key, ref[key], state_fraction=0.12)
        diff = abs(actual[name][key] - ref[key])
        if diff > tol:
            failures += 1
            print(f"FAIL  {name:18s} {key:10s} expected {ref[key]:10.3f}  actual {actual[name][key]:10.3f}  tolerance {tol:.3f}")

checked = len(expected) * len(metrics.all_keys())
print(f"{checked} metrics checked in {len(expected)} scenarios, {failures} outside the tolerance")
sys.exit(1 if failures else 0)
