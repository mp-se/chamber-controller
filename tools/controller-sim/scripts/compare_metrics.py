"""Compare the summary metrics of a reference controller and the current one.

usage: compare_metrics.py <reference dir> <current dir>
Each dir holds m_<perturbation>.txt files (output of `sim - <perturbation>` for every scenario),
m_0.00.txt is the unperturbed run.

A metric passes if EITHER
  A. the unperturbed current value is within the tolerance of the unperturbed reference value, OR
  B. it lies inside the range the REFERENCE covers itself over its perturbed runs, widened by the
     tolerance (a metric the reference cannot reproduce with itself cannot be required of the
     current controller).
Both are reported, so a pass by B alone is visible.
"""
import glob
import os
import sys

import metrics


def load(directory):
    files = sorted(glob.glob(os.path.join(directory, "m_*.txt")))
    return metrics.parse(os.path.join(directory, "m_0.00.txt")), [metrics.parse(f) for f in files]


ref_base, ref_runs = load(sys.argv[1])
cur_base, cur_runs = load(sys.argv[2])

fail = 0
only_b = []
rows = 0
for name in ref_base:
    for key in metrics.all_keys():
        ref = ref_base[name][key]
        cur = cur_base[name][key]
        tol = metrics.tolerance(name, key, ref)
        a = abs(cur - ref) <= tol
        lo = min(r[name][key] for r in ref_runs)
        hi = max(r[name][key] for r in ref_runs)
        b = lo - tol <= cur <= hi + tol
        rows += 1
        if not (a or b):
            fail += 1
            print(f"FAIL  {name:18s} {key:10s} reference {ref:10.3f} (range {lo:.3f}..{hi:.3f})  current {cur:10.3f}  tol {tol:.3f}")
        elif not a:
            only_b.append((name, key, ref, lo, hi, cur, tol))

print()
print(f"{rows} metrics checked ({len(ref_base)} scenarios), {fail} failed")
print(f"{rows - fail - len(only_b)} pass on the tolerance alone, {len(only_b)} pass only because the reference varies that much itself:")
for name, key, ref, lo, hi, cur, tol in only_b:
    print(f"   {name:18s} {key:10s} reference {ref:10.3f} (own range {lo:.3f}..{hi:.3f})  current {cur:10.3f}  tol {tol:.3f}")
sys.exit(1 if fail else 0)
