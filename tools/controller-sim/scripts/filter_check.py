"""The filter output must match the mathematically exact filter.

usage: filter_check.py <output of `filter_test steps`>
The coefficients are powers of two and the inputs are multiples of 1/16, so 80 digit decimal
arithmetic stands in for exact. Checks the first 3000 samples of the step response for b = 1..6.
Limit 1e-6 C. (The old fixed point filter was off by 6e-7 at b=2, 5e-5 at b=4 and 8e-4 at b=6.)
"""
import sys
from decimal import Decimal, getcontext

getcontext().prec = 80
SAMPLES = 3000
LIMIT = 1e-6

actual = {}
for line in open(sys.argv[1]):
    b, q, i, v = line.split()
    if q == "0" and int(i) < SAMPLES:
        actual[(int(b), int(i))] = float(v)


def exact(b):
    seq = [Decimal(20)] * 300 + [Decimal(25)] * (SAMPLES - 300)
    ka = Decimal(1) / (Decimal(2) ** (2 * b + 4))
    kb = Decimal(1) / (Decimal(2) ** b)
    x = [[seq[0]] * 3 for _ in range(3)]
    y = [[seq[0]] * 3 for _ in range(3)]
    out = []
    for i in range(SAMPLES):
        v = seq[i]
        for s in range(3):
            x[s] = [v, x[s][0], x[s][1]]
            y[s] = [None, y[s][0], y[s][1]]
            y[s][0] = 2 * y[s][1] - y[s][2] - kb * (y[s][1] - y[s][2]) + ka * (x[s][0] + 2 * x[s][1] + x[s][2]) - 4 * ka * y[s][2]
            v = y[s][0]
        out.append(float(v))
    return out


failures = 0
for b in range(1, 7):
    ex = exact(b)
    worst = max(abs(actual[(b, i)] - ex[i]) for i in range(SAMPLES))
    ok = worst <= LIMIT
    failures += not ok
    print(f"  b={b}: max |filter - exact| over {SAMPLES} samples = {worst:.3e} C  {'PASS' if ok else 'FAIL'}")
print("ALL PASSED" if failures == 0 else f"{failures} FAILED")
sys.exit(1 if failures else 0)
