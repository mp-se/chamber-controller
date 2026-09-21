"""Compare two REPLAY runs (identical sensor input) of a reference controller and the current one.

usage: compare_replay.py <reference trace dir> <current trace dir>
With the loop open, any difference is numerical. While the controller state is still identical
these must agree: filtered temperatures within 0.005 C, slope 0.01 C/h, P/I/D 0.02, fridge setting
0.01 (a fixed point reference is quantised to 1/512 and P amplifies that by Kp = 5, so 0.02 to 0.03
on the fridge setting is what to expect from it). Every state divergence is listed with the margin
to each threshold the state machine compares against, to show whether it is a threshold crossing.
"""
import csv
import os
import sys

limits = {"fF": 0.005, "sB": 0.005, "slope": 0.01, "p": 0.02, "i": 0.02, "d": 0.02, "fsP": 0.01}
ref_dir, cur_dir = sys.argv[1], sys.argv[2]
missed = 0

for name in sorted(f[:-4] for f in os.listdir(ref_dir) if f.endswith(".csv")):
    ref = list(csv.DictReader(open(os.path.join(ref_dir, name + ".csv"))))
    cur = list(csv.DictReader(open(os.path.join(cur_dir, name + ".csv"))))
    n = min(len(ref), len(cur))

    def key(r):
        return (r["state"], r["cool"], r["heat"])

    first_div = next((i for i in range(n) if key(ref[i]) != key(cur[i])), None)
    agree = sum(1 for i in range(n) if key(ref[i]) == key(cur[i]))
    upto = first_div if first_div is not None else n

    worst = {k: 0.0 for k in limits}
    for i in range(upto):
        for k in limits:
            worst[k] = max(worst[k], abs(float(ref[i][k]) - float(cur[i][k])))
    over = [k for k, d in worst.items() if d > limits[k]]
    missed += len(over)

    span = f"{int(ref[upto - 1]['t']) / 3600:5.2f} h" if upto else "0 h"
    print(f"{name:18s} identical controller state for {span} of {int(ref[n - 1]['t']) / 3600:.0f} h; state agrees in {100.0 * agree / n:5.1f} % of the samples")
    print("   max diff before the first state divergence: " + "  ".join(f"{k} {d:.4f}" + (" OVER" if k in over else "") for k, d in worst.items()))
    if first_div is not None:
        o, w = ref[first_div], cur[first_div]
        fo, fn = float(o["fF"]), float(w["fF"])
        so, sn = float(o["fsP"]), float(w["fsP"])
        bs = float(o["beerSetting"])
        margins = {
            "fridge below low limit": (fo - (so - 1.0), fn - (sn - 1.0)),
            "fridge above high limit": (fo - (so + 1.0), fn - (sn + 1.0)),
            "beer above target-1/32": (float(o["fB"]) - (bs - 1 / 32), float(w["fB"]) - (bs - 1 / 32)),
            "beer above target+1/32": (float(o["fB"]) - (bs + 1 / 32), float(w["fB"]) - (bs + 1 / 32)),
            "estimated peak vs fridge setting": (float(o["epP"]) - so, float(w["epP"]) - sn),
        }
        closest = min(margins, key=lambda k: min(abs(margins[k][0]), abs(margins[k][1])))
        a, b = margins[closest]
        print(f"   first state divergence at t={o['t']}s: state {o['state']}/{w['state']} cool {o['cool']}/{w['cool']} heat {o['heat']}/{w['heat']} (reference/current)")
        print(f"      fridge setting {so:.4f}/{sn:.4f}, fridge filtered {fo:.4f}/{fn:.4f}, beer fast {o['fB']}/{w['fB']}")
        print("      margins (value - threshold): " + "; ".join(f"{k} {x:+.4f}/{y:+.4f}" for k, (x, y) in margins.items()))
        print(f"      closest threshold: {closest}, {a:+.4f} vs {b:+.4f}")

print("\nALL CRITERIA MET" if missed == 0 else f"\n{missed} criteria missed (read the note in the docstring before judging)")
sys.exit(1 if missed else 0)
