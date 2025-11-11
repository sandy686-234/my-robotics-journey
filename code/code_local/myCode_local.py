#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import math, time, csv, json, subprocess, re, glob, os
from pathlib import Path
try:
    import HAL, Frequency  # Robotics Academy / ROS2 env provides this
except ImportError:
    # Local fallbacks so the script can run without the RA env
    import hal_stub as HAL
    import frequency_stub as Frequency

# ================== runtime role ==================
AGENT_NAME = "B"   # <- set "A" or "B" for this process

# ========== core config ==========
HZ = 20                                  # control loop frequency (Hz)
WAYPOINTS = [
    (2.0,  0.0, 17.0),
    (4.0, -1.0, 34.0),
    (6.0,  0.5, 54.0),
]
POS_TOL = 0.15                           # success radius (m)
V_MAX, W_MAX = 0.7, 0.9                  # speed caps (runtime)
TRACE_PATH = "/tmp/trace_constrained.csv"  # will be overridden into runs/<ts> after gate
POSE_LOG_PERIOD = 1.0                    # seconds

# --- STL/Constraints config ---
DANGER_A = (3.0, 5.0, -0.5, 0.5)         # (xmin, xmax, ymin, ymax)
VMAX_TABLE = [(10.0, 0.4), (20.0, 0.2), (1e9, 0.3)]   # (t_upper, vmax)
GOAL_WINDOW = 5.0

# --- Shared corridor (resource) config (matches gate) ---
CORRIDOR = (3.0, 5.0, -0.2, 0.2)   # corridor rectangle
EPS_MUTEX = 0.10                   # epsilon time margin in SMT (sec)

# --- Formal Gate/WCET config ---
DELTA_WCET = 0.10        # WCET safety margin (+10%)
GLOBAL_BLOCKING = 0.0    # worst-case blocking B (seconds)
RUNS_DIR = "runs"        # artifacts root

# ==========================================================
# Plan loader (from gate_two_agents.py)
def latest_plan_path(runs_dir="runs"):
    cand = sorted(Path(runs_dir).glob("*/plan.json"))
    return str(cand[-1]) if cand else None

def load_plan(plan_path=None):
    """Load the latest plan; fall back to no-plan mode if missing."""
    if plan_path is None:
        plan_path = latest_plan_path()
    if not plan_path or not Path(plan_path).exists():
        print("[PLAN] not found → run without corridor plan")
        return {"release_times": {}, "order": [], "corridor": CORRIDOR}
    data = json.loads(Path(plan_path).read_text())
    print(f"[PLAN] loaded {plan_path}")
    cor = tuple(data.get("corridor", CORRIDOR))
    return {
        "release_times": data.get("release_times", {}),
        "order": data.get("order", []),
        "corridor": cor if len(cor)==4 else CORRIDOR,
    }

PLAN = load_plan()  # dict with release_times, order, corridor

# ==========================================================
# Set loop rate if provided by the framework
if hasattr(Frequency, "setHz"):
    Frequency.setHz(HZ)

# ----------------- EDF task model -----------------
class Task:
    def __init__(self, tid, x, y, ddl_abs):
        self.id = tid
        self.x = x
        self.y = y
        self.deadline = ddl_abs
        self.released = 0.0
        self.start = None
        self.finish = None
        self.missed = False
        self.state = "READY"      # READY / RUNNING / DONE / MISS

def now():
    return time.monotonic()

def pose_xyth():
    """Return (x, y, theta). Prefer getPose2d(); fallback to getPose3d()."""
    if hasattr(HAL, "getPose2d"):
        p = HAL.getPose2d()
        try:
            x, y, th = p
        except Exception:
            x = getattr(p, "x", 0.0)
            y = getattr(p, "y", 0.0)
            th = getattr(p, "theta", getattr(p, "yaw", 0.0))
        return float(x), float(y), float(th)
    if hasattr(HAL, "getPose3d"):
        p = HAL.getPose3d()
        try:
            x, y, z, roll, pitch, yaw = p
            return float(x), float(y), float(yaw)
        except Exception:
            x = getattr(p, "x", 0.0)
            y = getattr(p, "y", 0.0)
            th = getattr(p, "yaw", getattr(p, "theta", 0.0))
            return float(x), float(y), float(th)
    return 0.0, 0.0, 0.0

def dist(a, b, c, d):
    return math.hypot(a - c, b - d)

def normalize_angle(a):
    """wrap to [-pi, pi)"""
    return (a + math.pi) % (2.0 * math.pi) - math.pi

def point_heading_ctrl(px, py, th, gx, gy):
    """Very simple P controller towards a waypoint (gx, gy)."""
    dx, dy = gx - px, gy - py
    target = math.atan2(dy, dx)
    e = normalize_angle(target - th)
    v = V_MAX * (1.0 - min(abs(e) / math.pi, 1.0))   # slow when heading error big
    w = 2.0 * e                                      # heading P term
    v = max(-V_MAX, min(V_MAX, v))
    w = max(-W_MAX, min(W_MAX, w))
    return v, w

# --------- STL runtime monitors (windowed robustness) ---------
class OnlineMonitor:
    def __init__(self, horizon_sec, dt):
        import collections, math as _m
        self.maxlen = max(1, int(_m.ceil(horizon_sec / max(dt, 1e-6))))
        self.buf = collections.deque(maxlen=self.maxlen)
    def clear(self):
        self.buf.clear()
    def step(self, value):
        self.buf.append(float(value))
    def rob_G(self):
        if not self.buf: return float('inf')
        return min(self.buf)
    def rob_F(self):
        if not self.buf: return float('-inf')
        return max(self.buf)

def vmax_of(t_since_start):
    for t_up, v in VMAX_TABLE:
        if t_since_start < t_up:
            return v
    return VMAX_TABLE[-1][1] if VMAX_TABLE else V_MAX

def margin_not_in_A(px, py):
    # Positive margin if outside A, negative if inside.
    xmin, xmax, ymin, ymax = DANGER_A
    inside = (xmin <= px <= xmax) and (ymin <= py <= ymax)
    return 0.05 if not inside else -0.05

def margin_in_goal(px, py, gx, gy, radius):
    # Positive if inside circle of given radius around goal.
    return radius - dist(px, py, gx, gy)

def in_rect(px, py, rect):
    xmin, xmax, ymin, ymax = rect
    return (xmin <= px <= xmax) and (ymin <= py <= ymax)

def corridor_flag(px, py):
    """Return 1 if inside shared corridor, else 0."""
    return 1 if in_rect(px, py, PLAN["corridor"]) else 0

def seg_intersects_rect(p, q, rect):
    """Axis-aligned rectangle overlap (conservative AABB)."""
    (x1,y1), (x2,y2) = p, q
    xmin,xmax,ymin,ymax = rect
    if max(x1,x2) < xmin or min(x1,x2) > xmax: return False
    if max(y1,y2) < ymin or min(y1,y2) > ymax: return False
    return True

def this_leg_uses_corridor(px, py, gx, gy, rect):
    return seg_intersects_rect((px,py), (gx,gy), rect)

def leg_name(agent, idx):
    return f"{agent}_leg{idx}"

def export_header(w):
    w.writerow([
        "t","event","task","x","y","deadline","note",
        "phi1_bool","phi1_rob",
        "phi2_bool","phi2_rob",
        "phi3_bool","phi3_rob",
        "phi_all_bool","phi_all_rob",
        "speed","vmax"
    ])

def export_event(w, t, ev, task, x=None, y=None, note="",
                 phi1_rob=None, phi2_rob=None, phi3_rob=None, speed=None, vmax=None):
    def b(v):
        if v is None: return ""
        return 1 if v >= 0 else 0
    phi_all_rob = None
    phi_all_bool = ""
    if phi1_rob is not None and phi2_rob is not None and phi3_rob is not None:
        phi_all_rob = min(phi1_rob, phi2_rob, phi3_rob)
        phi_all_bool = 1 if (phi1_rob>=0 and phi2_rob>=0 and phi3_rob>=0) else 0

    w.writerow([
        f"{t:.3f}",
        ev,
        (task.id if task else ""),
        (f"{x:.3f}" if x is not None else ""),
        (f"{y:.3f}" if y is not None else ""),
        (f"{task.deadline:.3f}" if task else ""),
        note,
        (b(phi1_rob) if phi1_rob is not None else ""),
        (f"{phi1_rob:.4f}" if phi1_rob is not None else ""),
        (b(phi2_rob) if phi2_rob is not None else ""),
        (f"{phi2_rob:.4f}" if phi2_rob is not None else ""),
        (b(phi3_rob) if phi3_rob is not None else ""),
        (f"{phi3_rob:.4f}" if phi3_rob is not None else ""),
        (phi_all_bool if phi_all_rob is not None else ""),
        (f"{phi_all_rob:.4f}" if phi_all_rob is not None else ""),
        (f"{speed:.3f}" if speed is not None else ""),
        (f"{vmax:.3f}" if vmax is not None else ""),
    ])

# ----------------- Gate/WCET (kept for single-robot) -----------------
EPS = 1e-9

def wcet_leg(ax, ay, bx, by, v_max_cap, w_max_cap, delta):
    """Conservative WCET: straight run + worst turn + margin."""
    d = math.hypot(bx-ax, by-ay)
    base = d/max(v_max_cap, 1e-9) + math.pi/max(w_max_cap, 1e-9)
    return base * (1.0 + delta)

def worst_v_cap_from_table(vmax_table):
    """For WCET we must assume worst (smallest) speed cap ever applies."""
    if not vmax_table: return V_MAX
    return min(v for _, v in vmax_table)

def build_jobs_from_waypoints(waypoints_rel, start_xy, v_cap, w_cap, delta, release_abs):
    """Return list of jobs {name, release, deadline, wcet} using relative deadlines."""
    pts = [start_xy] + [(x, y) for (x, y, _) in waypoints_rel]
    jobs = []
    for i, (wp, prev) in enumerate(zip(waypoints_rel, pts)):
        x, y, ddl_rel = wp
        C = wcet_leg(prev[0], prev[1], x, y, v_cap, w_cap, delta)
        jobs.append({
            "name": f"leg{i}",
            "release": float(release_abs),
            "deadline": float(release_abs + ddl_rel),
            "wcet": float(C)
        })
    return jobs

def pdc_windows_finite(jobs):
    R = sorted(set(round(j["release"], 9) for j in jobs))
    D = sorted(set(round(j["deadline"], 9) for j in jobs))
    for t1 in R:
        for t2 in D:
            if t2 >= t1 - EPS:
                yield (t1, t2)

def pdc_check_finite(jobs, blocking=0.0):
    """Processor-demand criterion for finite job sets (necessary & sufficient)."""
    cert = {"feasible": True, "windows": [], "violations": [], "blocking": blocking}
    for (t1, t2) in pdc_windows_finite(jobs):
        lhs = sum(j["wcet"] for j in jobs
                  if j["release"] >= t1 - EPS and j["deadline"] <= t2 + EPS) + blocking
        rhs = t2 - t1
        margin = rhs - lhs
        row = {"t1": t1, "t2": t2, "lhs": lhs, "rhs": rhs, "margin": margin}
        cert["windows"].append(row)
        if margin < -EPS:
            cert["feasible"] = False
            cert["violations"].append(row)
    return cert

def minimal_repair_hint(cert):
    if cert["feasible"] or not cert["violations"]:
        return {"type": "none", "hint": "already feasible"}
    v = min(cert["violations"], key=lambda r: r["margin"])
    delta = -(v["margin"])
    return {
        "type": "deadline_or_wcet",
        "window": {"t1": v["t1"], "t2": v["t2"]},
        "needed_relax": delta,
        "suggestions": [
            {"action": "relax_deadline", "job": "any job with d<=t2 in window", "delta_d": delta},
            {"action": "shrink_wcet", "job": "any job fully in window", "delta_C": delta}
        ]
    }

def export_smt2(cert, path, with_unsat_core=True):
    lines = []
    lines.append("(set-logic QF_LRA)")
    if with_unsat_core:
        lines.append("(set-option :produce-unsat-cores true)")
    for i, w in enumerate(cert["windows"]):
        nm = f"w{i}"
        lines.append(f"; window {i}: [{w['t1']}, {w['t2']}]  lhs={w['lhs']:.6f}  rhs={w['rhs']:.6f}")
        # '!' here is SMT attribute syntax (naming assertion), not logical not
        lines.append(f"(assert (! (<= {w['lhs']:.9f} {w['rhs']:.9f}) :named {nm}))")
    lines.append("(check-sat)")
    if with_unsat_core:
        lines.append("(get-unsat-core)")
    Path(path).write_text("\n".join(lines))
    return path

def try_run_z3(smt2_path):
    try:
        out = subprocess.check_output(["z3", "-smt2", str(smt2_path)], text=True)
        return True, out.strip()
    except (FileNotFoundError, subprocess.CalledProcessError) as e:
        return None, str(e)

# ----------------- Build tasks -----------------
t0 = now()
tasks = [Task(i, x, y, t0 + ddl_rel) for i, (x, y, ddl_rel) in enumerate(WAYPOINTS)]
for tk in tasks:
    tk.released = t0

# ----------------- Formal Gate before starting control -----------------
V_CAP_FOR_WCET = min(V_MAX, worst_v_cap_from_table(VMAX_TABLE))  # conservative linear speed
W_CAP_FOR_WCET = W_MAX

jobs = build_jobs_from_waypoints(
    waypoints_rel=WAYPOINTS,
    start_xy=(0.0, 0.0),
    v_cap=V_CAP_FOR_WCET,
    w_cap=W_CAP_FOR_WCET,
    delta=DELTA_WCET,
    release_abs=t0
)

run_ts = time.strftime("%Y%m%d-%H%M%S")
run_dir = Path(RUNS_DIR) / run_ts
run_dir.mkdir(parents=True, exist_ok=True)

# Save jobs.json
Path(run_dir/"jobs.json").write_text(json.dumps({
    "jobs": jobs,
    "meta": {
        "t0": t0, "delta_wcet": DELTA_WCET,
        "v_cap_for_wcet": V_CAP_FOR_WCET, "w_cap_for_wcet": W_CAP_FOR_WCET,
        "vmax_table": VMAX_TABLE, "source": "inline-gate"
    }
}, indent=2))

# PDC check (finite jobs)
pdc_cert = pdc_check_finite(jobs, blocking=GLOBAL_BLOCKING)
Path(run_dir/"pdc_certificate.json").write_text(json.dumps(pdc_cert, indent=2))

# SMT export + optional Z3 call (kept for single-robot certificate)
smt2_path = run_dir/"pdc.smt2"
export_smt2(pdc_cert, smt2_path, with_unsat_core=True)
z3_ok, z3_out = try_run_z3(smt2_path)
Path(run_dir/"z3_result.txt").write_text(z3_out if z3_out else "")

# Gate decision
if not pdc_cert["feasible"]:
    print("[GATE] FAIL (PDC infeasible). Minimal repair hint:", minimal_repair_hint(pdc_cert), flush=True)
    raise SystemExit(0)

print("[GATE] PASS. Artifacts in", run_dir, flush=True)

# Open trace file INSIDE run_dir so all artifacts live together
TRACE_PATH = str(run_dir / "trace_constrained.csv")
trace_fp = open(TRACE_PATH, "w", newline="")
trace = csv.writer(trace_fp)
export_header(trace)

# ----------------- Runtime state -----------------
current = None
current_leg_idx = None
_prev_pose = None  # (x, y, t)

DT = 1.0 / HZ
mon_not_inA = OnlineMonitor(horizon_sec=2.0, dt=DT)          # ϕ1: G(¬in_A)
mon_inGoal  = OnlineMonitor(horizon_sec=GOAL_WINDOW, dt=DT)  # ϕ2: F_[0,5](in_goal)
mon_vlimit  = OnlineMonitor(horizon_sec=0.25, dt=DT)         # ϕ3: G(speed ≤ vmax)

def pick_task_EDF():
    ready = [tk for tk in tasks if tk.state in ("READY", "RUNNING")]
    if not ready:
        return None
    return min(ready, key=lambda z: z.deadline)

def predict_enter_A(px, py, v, w, dt):
    """One-step kinematic prediction (Euler)."""
    th = pose_xyth()[2]
    nx = px + v * math.cos(th) * dt
    ny = py + v * math.sin(th) * dt
    xmin, xmax, ymin, ymax = DANGER_A
    return (xmin <= nx <= xmax) and (ymin <= ny <= ymax)

# ----------------- Main loop -----------------
try:
    export_event(trace, 0.0, "start", None, note="formal-gate passed; plan-aware runtime (EDF+Fence+STL)")
    last_pose_log = t0
    while True:
        t = now()
        x, y, th = pose_xyth()

        # speed estimate
        if _prev_pose is None:
            spd = 0.0
        else:
            px, py, pt = _prev_pose
            dt_s = max(1e-6, t - pt)
            spd = dist(x, y, px, py) / dt_s
        _prev_pose = (x, y, t)

        # limits & margins
        t_since = t - t0
        vmax_now = vmax_of(t_since)
        mon_not_inA.step(margin_not_in_A(x, y))
        if current is not None:
            mon_inGoal.step(margin_in_goal(x, y, current.x, current.y, POS_TOL))
        else:
            mon_inGoal.step(-1e6)  # until a goal is active
        mon_vlimit.step(vmax_now - spd)

        # deadline miss detection
        for tk in tasks:
            if tk.state in ("READY", "RUNNING") and t > tk.deadline:
                tk.state = "MISS"
                tk.missed = True
                export_event(trace, t - t0, "miss", tk, x, y, note="deadline",
                             phi1_rob=mon_not_inA.rob_G(),
                             phi2_rob=mon_inGoal.rob_F(),
                             phi3_rob=mon_vlimit.rob_G(),
                             speed=spd, vmax=vmax_now)

        # EDF pick
        nxt = pick_task_EDF()
        if nxt is None:
            HAL.setV(0.0); HAL.setW(0.0)
            export_event(trace, t - t0, "idle", None, x, y,
                         phi1_rob=mon_not_inA.rob_G(),
                         phi2_rob=mon_inGoal.rob_F(),
                         phi3_rob=mon_vlimit.rob_G(),
                         speed=spd, vmax=vmax_now)
            if all(tk.state in ("DONE", "MISS") for tk in tasks):
                break
            if hasattr(Frequency, "tick"):
                Frequency.tick()
            continue

        # (re)start
        if nxt is not current:
            current = nxt
            current_leg_idx = current.id
            if current.start is None:
                current.start = t
                mon_inGoal.clear()  # reset F-window
                export_event(trace, t - t0, "start", current, x, y,
                             note="new goal window",
                             phi1_rob=mon_not_inA.rob_G(),
                             phi2_rob=mon_inGoal.rob_F(),
                             phi3_rob=mon_vlimit.rob_G(),
                             speed=spd, vmax=vmax_now)

        # ---- Plan-based corridor guard (hold until planned release) ----
        wait_note = ""
        if current is not None and current_leg_idx is not None:
            will_use_corr = this_leg_uses_corridor(
                x, y, current.x, current.y, PLAN["corridor"]
            )
            if will_use_corr:
                my_leg = f"{AGENT_NAME}_leg{current_leg_idx}"
                rel_t  = PLAN["release_times"].get(my_leg, None)  # planned start time for corridor leg
                t_since = t - t0
                if (rel_t is not None) and (t_since + 1e-6 < rel_t):
                    wait_note = f"plan-hold {my_leg} until {rel_t:.2f}s"
                    HAL.setV(0.0); HAL.setW(0.0)
                    export_event(trace, t - t0, "pose", current, x, y,
                                 note=wait_note,
                                 phi1_rob=mon_not_inA.rob_G(),
                                 phi2_rob=mon_inGoal.rob_F(),
                                 phi3_rob=mon_vlimit.rob_G(),
                                 speed=spd, vmax=vmax_now)
                    last_pose_log = t
                    if hasattr(Frequency, "tick"):
                        Frequency.tick()
                    continue

        # control with caps + HCD fence before entering A
        v_cmd, w_cmd = point_heading_ctrl(x, y, th, current.x, current.y)
        v_cmd = max(-vmax_now, min(vmax_now, v_cmd))
        if predict_enter_A(x, y, v_cmd, w_cmd, 1.0 / HZ):
            note = "HCD: prevent entering A"
            v_out, w_out = 0.0, 0.0
        else:
            note = ""
            v_out, w_out = v_cmd, w_cmd
        HAL.setV(v_out)
        HAL.setW(w_out)

        # finish detection
        if dist(x, y, current.x, current.y) <= POS_TOL:
            current.finish = t
            current.state = "DONE"
            export_event(trace, t - t0, "finish", current, x, y,
                note=f"dur={(current.finish - current.start):.2f}s",
                phi1_rob=mon_not_inA.rob_G(),
                phi2_rob=mon_inGoal.rob_F(),
                phi3_rob=mon_vlimit.rob_G(),
                speed=spd, vmax=vmax_now)
            current = None
            HAL.setV(0.0); HAL.setW(0.0)

        # periodic pose logging
        if (t - last_pose_log) >= POSE_LOG_PERIOD:
            export_event(trace, t - t0, "pose", (current or tasks[0]), x, y,
                note=note if current else "",
                phi1_rob=mon_not_inA.rob_G(),
                phi2_rob=mon_inGoal.rob_F(),
                phi3_rob=mon_vlimit.rob_G(),
                speed=spd, vmax=vmax_now)
            last_pose_log = t

        if hasattr(Frequency, "tick"):
            Frequency.tick()

finally:
    HAL.setV(0.0); HAL.setW(0.0)
    export_event(trace, now() - t0, "end", None, note="all done",
                 phi1_rob=mon_not_inA.rob_G(),
                 phi2_rob=mon_inGoal.rob_F(),
                 phi3_rob=mon_vlimit.rob_G())
    trace_fp.flush()
    trace_fp.close()
    print(f"[INFO] run artifacts in {run_dir}", flush=True)
    print(f"[INFO] trace saved to {TRACE_PATH}", flush=True)



