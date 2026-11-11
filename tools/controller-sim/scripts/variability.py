"""How much do the summary metrics of one controller move when the scenario is perturbed slightly?

usage: variability.py <metrics file> [<metrics file> ...]   (each file = `sim - <perturbation>` for every scenario)
Prints per scenario and metric: mean, min, max and the population standard deviation.
"""
import statistics
import sys

import metrics

runs = [metrics.parse(p) for p in sys.argv[1:]]
print(f"{len(runs)} runs")
for name in runs[0]:
    print(name)
    for key in metrics.all_keys():
        v = [r[name][key] for r in runs]
        print(f"   {key:11s} mean {statistics.mean(v):10.4f}  min {min(v):10.4f}  max {max(v):10.4f}  sd {statistics.pstdev(v):9.4f}")
