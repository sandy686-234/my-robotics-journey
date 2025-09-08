#!/usr/bin/env python3
import csv, sys, math, pathlib
import matplotlib.pyplot as plt
from collections import defaultdict

def load_trace(path):
    rows = []
    with open(path, newline="") as f:
        r = csv.DictReader(f)
        for row in r:
            rows.append(row)
    return rows

def to_float(s):
    return float(s) if s not in ("", None) else None

def analyze(rows):
    # 任务级别统计
    starts, finishes, deadlines, misses = {}, {}, {}, set()
    poses = defaultdict(list)

    for row in rows:
        t = to_float(row["t"])
        ev = row["event"]
        task = row["task"]
        x = to_float(row["x"]); y = to_float(row["y"])
        ddl = to_float(row["deadline"]) if row["deadline"] else None

        if task != "":
            tid = int(task)
            if ddl is not None:
                deadlines[tid] = ddl
            if ev == "start":
                starts[tid] = t
            elif ev == "finish":
                finishes[tid] = t
            elif ev == "miss":
                misses.add(tid)

        if ev == "pose" and task != "":
            poses[int(task)].append((t, x, y))

    # 指标
    report = []
    for tid in sorted(set(list(starts.keys()) + list(finishes.keys()) + list(deadlines.keys()) + list(misses))):
        st = starts.get(tid)
        fn = finishes.get(tid)
        ddl = deadlines.get(tid)
        miss = tid in misses
        dur = (fn - st) if (fn is not None and st is not None) else None
        lateness = None
        if ddl is not None:
            if fn is not None:
                lateness = fn - ddl     # >0 表示迟到
            elif miss:
                lateness = math.nan
        report.append((tid, st, fn, ddl, dur, miss, lateness))

    return report, poses

def print_report(report):
    print("tid | start | finish | deadline | duration | miss | lateness")
    for tid, st, fn, ddl, dur, miss, late in report:
        print(f"{tid:>3} | {st!s:>6} | {fn!s:>6} | {ddl!s:>8} | {dur!s:>8} | {miss!s:>4} | {late!s}")

def plot_gantt(report, out_png):
    # 简单甘特图
    import matplotlib.pyplot as plt
    fig, ax = plt.subplots(figsize=(7, 2 + 0.5*len(report)))
    yticks, ylabels = [], []
    for i,(tid, st, fn, ddl, _, miss, _) in enumerate(report):
        y = i
        yticks.append(y); ylabels.append(f"T{tid}")
        if st is not None and fn is not None:
            ax.broken_barh([(st, fn-st)], (y-0.3, 0.6), alpha=0.7, label="run" if i==0 else None)
        if ddl is not None:
            ax.vlines(ddl, y-0.4, y+0.4, colors="r", linestyles="--")
    ax.set_xlabel("time (s)")
    ax.set_yticks(yticks); ax.set_yticklabels(ylabels)
    ax.grid(True, axis="x", alpha=0.3)
    fig.tight_layout()
    fig.savefig(out_png, dpi=180)
    print(f"[OK] saved {out_png}")

def main():
    path = pathlib.Path(sys.argv[1] if len(sys.argv)>1 else "traces/trace_latest.csv")
    rows = load_trace(path)
    report, poses = analyze(rows)
    print_report(report)
    out_png = path.with_suffix(".gantt.png")
    plot_gantt(report, out_png)

if __name__ == "__main__":
    main()

