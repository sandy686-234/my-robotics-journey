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
V_MAX, W_MAX = 0.7, 0.9                 # speed limits
TRACE_PATH = "/tmp/trace.csv"           # GUI container can write here
POSE_LOG_PERIOD = 1.0                   # seconds
# ============================

# Set loop rate if provided by the framework
if hasattr(Frequency, "setHz"):
    Frequency.setHz(HZ)

class Task:
    def __init__(self, tid, x, y, ddl_abs):
        self.id = tid
        self.x = x
        self.y = y
        self.deadline = ddl_abs   # absolute deadline (seconds since time.monotonic() epoch)
        self.released = 0.0       # absolute release time
        self.start = None
        self.finish = None
        self.missed = False
        self.state = "READY"      # READY / RUNNING / DONE / MISS

def now():
    return time.monotonic()

def pose_xyth():
    """
    Return (x, y, theta).
    Prefer getPose2d(); fallback to getPose3d().
    Compatible with tuple/list or object-with-attributes.
    """
    # Prefer 2D pose
    if hasattr(HAL, "getPose2d"):
        p = HAL.getPose2d()
        try:
            x, y, th = p  # iterable
        except Exception:
            x = getattr(p, "x", 0.0)
            y = getattr(p, "y", 0.0)
            th = getattr(p, "theta", getattr(p, "yaw", 0.0))
        return x, y, th

    # Fallback 3D pose
    if hasattr(HAL, "getPose3d"):
        p = HAL.getPose3d()
        try:
            x, y, z, roll, pitch, yaw = p  # iterable
            return x, y, yaw
        except Exception:
            x = getattr(p, "x", 0.0)
            y = getattr(p, "y", 0.0)
            th = getattr(p, "yaw", getattr(p, "theta", 0.0))
            return x, y, th

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
    v = V_MAX * (1.0 - min(abs(e) / math.pi, 1.0))   # slow down when heading error is big
    w = 2.0 * e                                      # heading P term
    v = max(-V_MAX, min(V_MAX, v))
    w = max(-W_MAX, min(W_MAX, w))
    return v, w

def export_header(w):
    w.writerow(["t", "event", "task", "x", "y", "deadline", "note"])

def export_event(w, t, ev, task, x=None, y=None, note=""):
    w.writerow([
        f"{t:.3f}",
        ev,
        (task.id if task else ""),
        (f"{x:.3f}" if x is not None else ""),
        (f"{y:.3f}" if y is not None else ""),
        (f"{task.deadline:.3f}" if task else ""),
        note
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

tick = 0
current = None
last_pose_log = t0

def pick_task_EDF():
    ready = [tk for tk in tasks if tk.state in ("READY", "RUNNING")]
    if not ready:
        return None
    return min(ready, key=lambda z: z.deadline)  # earliest deadline first

try:
    while True:
        t = now()

        # deadline miss check
        for tk in tasks:
            if tk.state in ("READY", "RUNNING") and t > tk.deadline:
                tk.state = "MISS"
                tk.missed = True
                export_event(trace, t - t0, "miss", tk, note="deadline")

        # pick next according to EDF
        nxt = pick_task_EDF()
        if nxt is None:
            HAL.setV(0.0)
            HAL.setW(0.0)
            export_event(trace, t - t0, "idle", None)
            if all(tk.state in ("DONE", "MISS") for tk in tasks):
                break
            tick += 1
            if hasattr(Frequency, "tick"):
                Frequency.tick()
            continue

        # (re)start event
        if nxt is not current:
            current = nxt
            if current.start is None:
                current.start = t
                export_event(trace, t - t0, "start", current)

        # control towards current waypoint
        x, y, th = pose_xyth()
        v, w = point_heading_ctrl(x, y, th, current.x, current.y)
        HAL.setV(v)
        HAL.setW(w)

        # finish condition
        if dist(x, y, current.x, current.y) <= POS_TOL:
            current.finish = t
            current.state = "DONE"
            export_event(
                trace, t - t0, "finish", current, x, y,
                note=f"dur={(current.finish - current.start):.2f}s"
            )
            current = None
            HAL.setV(0.0)
            HAL.setW(0.0)

        # periodic pose logging (each POSE_LOG_PERIOD seconds)
        if (t - last_pose_log) >= POSE_LOG_PERIOD:
            # 若 current 为空，随便给一个任务对象用于“deadline”字段展示
            holder = current if current else tasks[0]
            export_event(trace, t - t0, "pose", holder, x, y)
            last_pose_log = t

        tick += 1
        if hasattr(Frequency, "tick"):
            Frequency.tick()

finally:
    # finalize
    HAL.setV(0.0)
    HAL.setW(0.0)
    export_event(trace, now() - t0, "end", None, note="all done")
    trace_fp.flush()
    trace_fp.close()
    print(f"[INFO] trace saved to {TRACE_PATH}", flush=True)




