import argparse, csv, os, math, json
import matplotlib.pyplot as plt

# ======= 可改参数（与你的实验保持一致）=======
DANGER_A = (3.0, 5.0, -0.5, 0.5)                 # 危险区 A: (xmin, xmax, ymin, ymax)
VMAX_TABLE = [(10.0, 0.4), (20.0, 0.2), (1e9, 0.3)]  # 速度上限分段
GOAL_WINDOW = 5.0                                    # F_[0,5](in_goal) 窗口
# =============================================

def read_trace(path):
    import csv
    with open(path, newline="") as f:
        return list(csv.DictReader(f))

def to_f(x):
    try: return float(x)
    except: return None

def vmax_of(t):
    for t_up, v in VMAX_TABLE:
        if t < t_up: return v
    return VMAX_TABLE[-1][1]

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("trace", help="runs/<ts>/trace.csv")
    args = ap.parse_args()
    trace = read_trace(args.trace)
    outdir = os.path.dirname(os.path.abspath(args.trace)) or "."

    # 提取列
    t = [to_f(r.get("t")) for r in trace]
    x = [to_f(r.get("x")) for r in trace]
    y = [to_f(r.get("y")) for r in trace]
    ev= [r.get("event","") for r in trace]
    task=[r.get("task") for r in trace]
    speed = [to_f(r.get("speed")) for r in trace]
    vmax  = [to_f(r.get("vmax"))  for r in trace]

    # 速度缺失则差分估计
    if all(v is None for v in speed):
        speed = [None]
        for i in range(1, len(trace)):
            if None in (t[i-1], t[i], x[i-1], x[i], y[i-1], y[i]): speed.append(None); continue
            dt = max(1e-6, t[i]-t[i-1])
            dx, dy = x[i]-x[i-1], y[i]-y[i-1]
            speed.append((dx**2+dy**2)**0.5 / dt)

    # vmax 缺失则按表生成
    if all(v is None for v in vmax):
        vmax = [vmax_of(tt if tt is not None else 0.0) for tt in t]

    # ϕ1: G(¬in_A) 近似鲁棒度（在 A 外 +0.05；在 A 内 -0.05）
    xmin,xmax,ymin,ymax = DANGER_A
    phi1 = []
    for xi, yi in zip(x, y):
        if None in (xi, yi): phi1.append(None); continue
        inside = (xmin <= xi <= xmax) and (ymin <= yi <= ymax)
        phi1.append(0.05 if not inside else -0.05)

    # ϕ3: G(speed ≤ vmax) 鲁棒度 = vmax - speed
    phi3 = []
    for s, vm in zip(speed, vmax):
        if None in (s, vm): phi3.append(None); continue
        phi3.append(vm - s)

    # ϕ2（事件近似）：每个 task 是否在 start 后 5s 内 finish
    phi2_by_task = {}
    try:
        tids = sorted({int(k) for k in task if k not in ("", None)})
    except:
        tids = []
    for tid in tids:
        ts = [to_f(r.get("t")) for r in trace if r.get("task") == str(tid) and r.get("event")=="start"]
        fs = [to_f(r.get("t")) for r in trace if r.get("task") == str(tid) and r.get("event")=="finish"]
        sat = None
        if ts:
            s = ts[0]
            f = [ff for ff in fs if ff is not None and ff >= s]
            sat = (len(f)>0 and (f[0]-s) <= GOAL_WINDOW)
        phi2_by_task[tid] = {"satisfied": bool(sat) if sat is not None else None}

    # --------- 图 1: 速度 vs 上限 ----------
    plt.figure()
    tt = [tt for tt in t if tt is not None]
    plt.plot(t, speed, label="speed")
    plt.plot(t, vmax,  label="vmax")
    plt.xlabel("time (s)"); plt.ylabel("m/s"); plt.legend()
    plt.title("Speed vs time-varying limit")
    p1 = os.path.join(outdir, "speed_vs_limit.png"); plt.savefig(p1, dpi=160, bbox_inches="tight"); plt.close()

    # --------- 图 2: 轨迹 + 危险区 ----------
    plt.figure()
    xs = [xi for xi,yi in zip(x,y) if xi is not None and yi is not None]
    ys = [yi for xi,yi in zip(x,y) if xi is not None and yi is not None]
    if xs: plt.plot(xs, ys)
    plt.plot([xmin,xmax,xmax,xmin,xmin],[ymin,ymin,ymax,ymax,ymin])  # A 区矩形
    plt.xlabel("x (m)"); plt.ylabel("y (m)"); plt.axis("equal")
    plt.title("Trajectory (danger A shown)")
    p2 = os.path.join(outdir, "trajectory.png"); plt.savefig(p2, dpi=160, bbox_inches="tight"); plt.close()

    # --------- 图 3: 鲁棒度曲线 ----------
    plt.figure()
    plt.plot(t, phi1, label="phi1: G(not in A)")
    plt.plot(t, phi3, label="phi3: G(speed ≤ vmax)")
    plt.axhline(0.0, linestyle="--")
    plt.xlabel("time (s)"); plt.ylabel("robustness (approx.)"); plt.legend()
    plt.title("Robustness over time (approx.)")
    p3 = os.path.join(outdir, "robustness_estimated.png"); plt.savefig(p3, dpi=160, bbox_inches="tight"); plt.close()

    # --------- 摘要 JSON ----------
    def _nanmin(arr):
        arr = [v for v in arr if isinstance(v,(int,float))]
        return (min(arr) if arr else None)
    summary = {
        "rows": len(trace),
        "deadline_misses": sum(1 for r in trace if r.get("event")=="miss"),
        "phi1_min_rob_est": _nanmin(phi1),
        "phi1_G_est_satisfied": (_nanmin(phi1) is not None and _nanmin(phi1) >= 0),
        "phi3_min_rob_est": _nanmin(phi3),
        "phi3_G_est_satisfied": (_nanmin(phi3) is not None and _nanmin(phi3) >= 0),
        "phi2_F_0_5_per_task": phi2_by_task,
        "figures": [p1,p2,p3],
    }
    with open(os.path.join(outdir, "verification_summary.json"), "w") as f:
        json.dump(summary, f, indent=2)
    print("Wrote:", *summary["figures"], os.path.join(outdir, "verification_summary.json"), sep="\n - ")

if __name__ == "__main__":
    main()


