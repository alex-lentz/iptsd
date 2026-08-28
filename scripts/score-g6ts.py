#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-2.0-or-later
"""
Scores iptsd-g6ts-replay JSON output against the Windows processor pen reports
of the same capture phase (processor-pen-reports-P4-P8.csv from the
SP11 Windows capture evidence corpus).

The two time bases differ by a constant per-phase offset (the corpus starts at
the first HEAT record while the Windows trace starts earlier), so the scorer
derives the offset from the first report of each phase and aligns every cycle
to the nearest Windows report within +-8 ms (half the ~66 Hz cycle).

Usage: score-g6ts.py <phase> <replay.json> <processor-reports.csv>
"""

import bisect
import csv
import json
import statistics
import sys

X_MAX, Y_MAX = 27388.0, 18258.0
HIMETRIC_PER_MM = 100.06
MATCH_WINDOW_MS = 8.0


def main() -> int:
    if len(sys.argv) != 4:
        print(__doc__)
        return 2

    phase, replay_path, reports_path = sys.argv[1:4]

    with open(reports_path, encoding="utf-8-sig") as handle:
        rows = list(csv.DictReader(handle))
    win = [r for r in rows if r["Phase"] == phase]
    win_t = [float(r["ActionElapsedMilliseconds"]) for r in win]

    with open(replay_path) as handle:
        ours = [json.loads(l) for l in handle if l.startswith('{"cycle"')]
    if not ours or not win:
        print("no data to score")
        return 1

    offset = win_t[0] - min(o["t_ms"] for o in ours)

    errs = []
    prox_agree = 0
    used = 0
    for cycle in ours:
        t = cycle["t_ms"] + offset
        j = bisect.bisect_left(win_t, t - MATCH_WINDOW_MS)
        best, best_d = None, 1e9
        while j < len(win) and win_t[j] <= t + MATCH_WINDOW_MS:
            d = abs(win_t[j] - t)
            if d < best_d:
                best, best_d = win[j], d
            j += 1
        if best is None:
            continue
        used += 1

        in_range = best["InRange"] not in ("0", "")
        if in_range != bool(cycle["proximity"]):
            prox_agree += 1
        if not in_range or not cycle["proximity"]:
            continue

        dx = float(best["X"]) - cycle["x"] * X_MAX
        dy = float(best["Y"]) - cycle["y"] * Y_MAX
        errs.append((dx * dx + dy * dy) ** 0.5)

    print(f"== {phase}: windows={len(win)} ours={len(ours)} matched={used} "
          f"offset={offset:.1f}ms")
    if errs:
        errs.sort()
        q = lambda p: errs[int(p * (len(errs) - 1))]
        print(f"   position error vs Windows: median "
              f"{statistics.median(errs):.1f} HIM = "
              f"{statistics.median(errs) / HIMETRIC_PER_MM:.2f} mm | "
              f"p95 {q(0.95) / HIMETRIC_PER_MM:.2f} mm | "
              f"max {errs[-1] / HIMETRIC_PER_MM:.2f} mm")
    print(f"   proximity disagreements: {prox_agree}/{used}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
