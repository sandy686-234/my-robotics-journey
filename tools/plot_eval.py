#!/usr/bin/env python3
import csv, pathlib
import matplotlib.pyplot as plt

rows = list(csv.DictReader(open("runs_summary.csv", newline="")))
if not rows:
    print("runs_summary.csv is empty"); exit(0)

# --- ① 每个 run 的最小裕量柱状图 ---
labels, slacks = [], []
for r in rows:
    ms = r.get("min_slack_s", "")
    if ms not in ("", None):
        labels.append(r["run"])
        slacks.append(float(ms))
if slacks:
    plt.figure(figsize=(9,3))
    plt.bar(range(len(slacks)), slacks)
    plt.xticks(range(len(labels)), labels, rotation=45, ha="right", fontsize=8)
    plt.ylabel("min_slack (s)")
    plt.title("Min slack by run")
    plt.tight_layout()
    plt.savefig("eval_min_slack_by_run.png", dpi=150)

# --- ② 读取 params.txt（若有）做参数-裕量散点图 ---
def read_params(run):
    p = pathlib.Path("runs")/run/"params.txt"
    vals={}
    if p.exists():
        for line in p.read_text().splitlines():
            if "=" in line:
                k,v=line.split("=",1)
                try: vals[k.strip()] = float(v.strip())
                except: pass
    return vals

wcet_x, wcet_y = [], []
vmin_x, vmin_y = [], []
for r in rows:
    ms = r.get("min_slack_s","")
    if ms in ("", None): continue
    ms = float(ms)
    ps = read_params(r["run"])
    if "DELTA_WCET" in ps:
        wcet_x.append(ps["DELTA_WCET"]); wcet_y.append(ms)
    if "VMIN" in ps:
        vmin_x.append(ps["VMIN"]); vmin_y.append(ms)

if wcet_x:
    plt.figure()
    plt.scatter(wcet_x, wcet_y)
    plt.xlabel("DELTA_WCET")
    plt.ylabel("min_slack (s)")
    plt.grid(True, alpha=0.3)
    plt.tight_layout()
    plt.savefig("eval_wcet_vs_slack.png", dpi=150)

if vmin_x:
    plt.figure()
    plt.scatter(vmin_x, vmin_y)
    plt.xlabel("VMIN (m/s)")
    plt.ylabel("min_slack (s)")
    plt.grid(True, alpha=0.3)
    plt.tight_layout()
    plt.savefig("eval_vmin_vs_slack.png", dpi=150)

print("Saved:",
      "eval_min_slack_by_run.png",
      ("eval_wcet_vs_slack.png" if wcet_x else "(no wcet plot: missing params)"),
      ("eval_vmin_vs_slack.png" if vmin_x else "(no vmin plot: missing params)"))
