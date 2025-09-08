import csv, sys, os
import matplotlib.pyplot as plt
import datetime

def load_trace(path):
    with open(path, newline="") as f:
        return list(csv.DictReader(f))

def main():
    if len(sys.argv) < 2:
        print("Usage: python3 scripts/analyze_trace.py <trace.csv>")
        sys.exit(1)

    path = sys.argv[1]
    rows = load_trace(path)

    # 提取任务执行信息
    tasks = {}
    for r in rows:
        ev, tid = r["event"], r["task"]
        if not tid: continue
        tid = int(tid)
        if tid not in tasks:
            tasks[tid] = {"start": None, "finish": None, "deadline": float(r["deadline"]), "miss": False}
        if ev == "start":
            tasks[tid]["start"] = float(r["t"])
        elif ev == "finish":
            tasks[tid]["finish"] = float(r["t"])
        elif ev == "miss":
            tasks[tid]["miss"] = True

    # 打印表格
    print("tid | start | finish | deadline | duration | miss | lateness")
    for tid, info in tasks.items():
        dur = (info["finish"] - info["start"]) if (info["start"] and info["finish"]) else None
        late = (info["finish"] - info["deadline"]) if info["finish"] else None
        print(f"{tid:2d} | {info['start']} | {info['finish']} | {info['deadline']} | {dur} | {info['miss']} | {late}")

    # 自动生成文件名
    timestamp = datetime.datetime.now().strftime("%Y-%m-%d_%H%M")
    base = os.path.splitext(os.path.basename(path))[0]
    out_gantt = f"traces/{base}_{timestamp}.gantt.png"
    out_summary = f"traces/{base}_{timestamp}.summary.csv"

    os.makedirs("traces", exist_ok=True)

    # 画 Gantt 图
    fig, ax = plt.subplots()
    for tid, info in tasks.items():
        if info["start"] and info["finish"]:
            ax.barh(tid, info["finish"] - info["start"], left=info["start"])
        ax.axvline(info["deadline"], color="r", linestyle="--")
    ax.set_xlabel("time (s)")
    ax.set_ylabel("task id")
    plt.savefig(out_gantt)
    print(f"[OK] saved {out_gantt}")

    # 导出 summary CSV
    with open(out_summary, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["tid","start","finish","deadline","duration","miss","lateness"])
        for tid, info in tasks.items():
            dur = (info["finish"] - info["start"]) if (info["start"] and info["finish"]) else ""
            late = (info["finish"] - info["deadline"]) if info["finish"] else ""
            w.writerow([tid, info["start"], info["finish"], info["deadline"], dur, info["miss"], late])
    print(f"[OK] saved {out_summary}")

if __name__ == "__main__":
    main()

