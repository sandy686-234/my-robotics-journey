import math, time, csv
import HAL, Frequency

# ========== config ==========
HZ = 20                                 # control loop frequency (Hz)
WAYPOINTS = [                           # (x, y, deadline_relative_to_t0_in_seconds)
    (2.0,  0.0, 10.0),
    (4.0, -1.0, 20.0),
    (6.0,  0.5, 30.0),
]
POS_TOL = 0.15                          # success radius (m)
V_MAX, W_MAX = 0.7, 0.9                 # speed limits (caps)
TRACE_PATH = "/tmp/trace_constrained.csv"   # writeable in GUI container
POSE_LOG_PERIOD = 1.0                   # seconds

# --- STL/Constraints config ---
DANGER_A = (3.0, 5.0, -0.5, 0.5)        # (xmin, xmax, ymin, ymax)
VMAX_TABLE = [(10.0, 0.4), (20.0, 0.2), (1e9, 0.3)]   # (t_upper, vmax)
GOAL_WINDOW = 5.0
# ============================

if hasattr(Frequency, "setHz"):
    Frequency.setHz(HZ)

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
    # Prefer 2D pose
    if hasattr(HAL, "getPose2d"):
        p = HAL.getPose2d()
        try:
            x, y, th = p  # iterable
        except Exception:
            x = getattr(p, "x", 0.0)
            y = getattr(p, "y", 0.0)
            th = getattr(p, "theta", getattr(p, "yaw", 0.0))
        return float(x), float(y), float(th)

    # Fallback 3D pose
    if hasattr(HAL, "getPose3d"):
        p = HAL.getPose3d()
        try:
            x, y, z, roll, pitch, yaw = p  # iterable
            return float(x), float(y), float(yaw)
        except Exception:
            x = getattr(p, "x", 0.0)
            y = getattr(p, "y", 0.0)
            th = getattr(p, "yaw", getattr(p, "theta", 0.0))
            return float(x), float(y), float(th)

    # Last resort
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

# --------- STL monitors (minimal windowed robustness) ---------
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
    # Positive if inside circle of radius around goal.
    return radius - dist(px, py, gx, gy)

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

# Build tasks
t0 = now()
tasks = [Task(i, x, y, t0 + ddl_rel) for i, (x, y, ddl_rel) in enumerate(WAYPOINTS)]
for tk in tasks:
    tk.released = t0

# Open trace file
trace_fp = open(TRACE_PATH, "w", newline="")
trace = csv.writer(trace_fp)
export_header(trace)

current = None
_prev_pose = None  # (x, y, t)

# Runtime STL monitors
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
    """One-step kinematic prediction for differential drive (Euler)."""
    th = pose_xyth()[2]  # current heading
    nx = px + v * math.cos(th) * dt
    ny = py + v * math.sin(th) * dt
    xmin, xmax, ymin, ymax = DANGER_A
    return (xmin <= nx <= xmax) and (ymin <= ny <= ymax)

try:
    export_event(trace, 0.0, "start", None, note="constrained+STL runtime")
    last_pose_log = t0
    while True:
        t = now()
        x, y, th = pose_xyth()

        # speed
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
            mon_inGoal.step(-1e6)
        mon_vlimit.step(vmax_now - spd)

        # deadline miss
        for tk in tasks:
            if tk.state in ("READY", "RUNNING") and t > tk.deadline:
                tk.state = "MISS"
                tk.missed = True
                export_event(trace, t - t0, "miss", tk, x, y, note="deadline",
                             phi1_rob=mon_not_inA.rob_G(),
                             phi2_rob=mon_inGoal.rob_F(),
                             phi3_rob=mon_vlimit.rob_G(),
                             speed=spd, vmax=vmax_now)

        # pick EDF
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
            if current.start is None:
                current.start = t
                mon_inGoal.clear()  # reset F-window
                export_event(trace, t - t0, "start", current, x, y,
                             note="new goal window",
                             phi1_rob=mon_not_inA.rob_G(),
                             phi2_rob=mon_inGoal.rob_F(),
                             phi3_rob=mon_vlimit.rob_G(),
                             speed=spd, vmax=vmax_now)

        # control
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

        # finish
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
                note=note,
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
    print(f"[INFO] trace saved to {TRACE_PATH}", flush=True)

