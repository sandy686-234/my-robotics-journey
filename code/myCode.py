import math, time, sys, csv
import HAL, Frequency

# --------- config ----------
HZ = 20                 # control loop frequency
WAYPOINTS = [           # (x,y,deadline_s)
    (2.0,  0.0, 10.0),
    (4.0, -1.0, 20.0),
    (6.0,  0.5, 30.0),
]
POS_TOL = 0.15          # success radius (m)
V_MAX, W_MAX = 0.7, 0.9 # speed limits
TRACE_PATH = "/tmp/trace.csv"
# ---------------------------

Frequency.setHz(HZ) if hasattr(Frequency, "setHz") else None

class Task:
    def __init__(self, tid, x, y, ddl):
        self.id = tid
        self.x = x; self.y = y
        self.deadline = ddl    # absolute seconds since t0
        self.released = 0.0    # absolute release time
        self.start = None
        self.finish = None
        self.missed = False
        self.state = "READY"   # READY/RUNNING/DONE/MISS

def now():
    return time.monotonic()

def pose_xyth():
    x, y, z, roll, pitch, yaw = HAL.getPose3d()
    return x, y, yaw

def dist(a, b, c, d):
    return math.hypot(a-c, b-d)

def point_heading_ctrl(px, py, th, gx, gy):
    # simple P controller to waypoint
    dx, dy = gx - px, gy - py
    target = math.atan2(dy, dx)
    e = (target - th + math.pi) % (2*math.pi) - math.pi
    v = V_MAX * (1.0 - min(abs(e)/math.pi, 1.0))
    w = 2.0 * e
    return max(-V_MAX, min(V_MAX, v)), max(-W_MAX, min(W_MAX, w))

def export_header(w):
    w.writerow(["t","event","task","x","y","deadline","note"])

def export_event(w, t, ev, task, x=None, y=None, note=""):
    w.writerow([f"{t:.3f}", ev, task.id if task else "", 
                f"{x:.3f}" if x is not None else "", 
                f"{y:.3f}" if y is not None else "", 
                f"{task.deadline:.3f}" if task else "", note])

# build tasks (release at t0)
tasks = [Task(i, x, y, ddl) for i,(x,y,ddl) in enumerate(WAYPOINTS)]
t0 = now()
for tk in tasks:
    tk.released = t0

# open trace
trace_fp = open(TRACE_PATH, "w", newline="")
trace = csv.writer(trace_fp)
export_header(trace)

tick = 0
current = None

def pick_task_EDF(t):
    ready = [tk for tk in tasks if tk.state in ("READY","RUNNING")]
    if not ready:
        return None
    # earliest absolute deadline
    return min(ready, key=lambda z: z.deadline)

while True:
    t = now()
    # check deadline miss
    for tk in tasks:
        if tk.state in ("READY","RUNNING") and t > tk.deadline:
            tk.state = "MISS"; tk.missed = True
            export_event(trace, t - t0, "miss", tk, note="deadline")

    # select task
    nxt = pick_task_EDF(t)
    if nxt is None:
        HAL.setV(0.0); HAL.setW(0.0)
        export_event(trace, t - t0, "idle", None)
        # stop if all done/missed
        if all(tk.state in ("DONE","MISS") for tk in tasks):
            break
        tick += 1; Frequency.tick(); continue

    # start event
    if nxt is not current:
        current = nxt
        if current.start is None:
            current.start = t
            export_event(trace, t - t0, "start", current)

    # control to waypoint
    x, y, th = pose_xyth()
    v, w = point_heading_ctrl(x, y, th, current.x, current.y)
    HAL.setV(v); HAL.setW(w)

    # finish condition
    if dist(x, y, current.x, current.y) <= POS_TOL:
        current.finish = t
        current.state = "DONE"
        export_event(trace, t - t0, "finish", current, x, y, note=f"dur={(current.finish-current.start):.2f}s")
        current = None
        HAL.setV(0.0); HAL.setW(0.0)

    # periodic telemetry (optional)
    if tick % HZ == 0:
        export_event(trace, t - t0, "pose", current if current else tasks[0], x, y)

    tick += 1
    Frequency.tick()

# finalize
HAL.setV(0.0); HAL.setW(0.0)
export_event(trace, now() - t0, "end", None, note="all done")
trace_fp.flush(); trace_fp.close()

print(f"[INFO] trace saved to {TRACE_PATH}", flush=True)


