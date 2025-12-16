#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import math, time, json, subprocess, re
from pathlib import Path

# ===================== CONFIG (no YAML) =====================
# global speed caps (for WCET bounding)
V_MAX = 0.7
W_MAX = 1.2
VMAX_TABLE = [(10.0, 0.4), (20.0, 0.21), (1e9, 0.3)]  # (t_upper, vmax)

# WCET / gate
DELTA_WCET = 0.10         # safety margin (+10%)
GLOBAL_BLOCKING = 0.0     # optional B
EPS_MUTEX    = 0.10       # corridor mutex slack (sec)

# shared corridor rectangle: (xmin, xmax, ymin, ymax)
CORRIDOR = (3.0, 5.0, -0.2, 0.2)

# agents + waypoints (x, y, relative_deadline)
AGENTS = {
    "A": {
        "start": (0.0, 0.0),
        "waypoints": [(2.0, 0.0, 32.0), (4.0, -1.0, 54.0), (6.0, 0.5, 76.0)],
    },
    "B": {
        "start": (0.0, 0.5),
        "waypoints": [(2.0, 0.6, 33.0), (4.2, 0.0, 66.0), (6.0, 0.4, 88.0)],
    },
}

RUNS_DIR = "runs"
EPS = 1e-9

# 仅用于调试：是否把 PDC 窗口也写成 SMT 断言（通常不需要，交由 Python 判定）
INCLUDE_PDC_ASSERTS = False
# ===========================================================


# ---------- helpers ----------
def worst_v_cap(vmax_table, default_v):
    """Smallest cap over the whole mission (conservative)."""
    return min(v for _, v in vmax_table) if vmax_table else default_v

def wcet_leg(ax, ay, bx, by, v_cap, w_cap, delta):
    """Conservative WCET = straight + worst turn + margin."""
    d = math.hypot(bx - ax, by - ay)
    base = d/max(v_cap, 1e-9) + math.pi/max(w_cap, 1e-9)
    return base * (1.0 + delta)

def segment_intersects_rect(p, q, rect):
    """Axis-aligned rectangle intersection (AABB), conservative."""
    xmin, xmax, ymin, ymax = rect
    (x1, y1), (x2, y2) = p, q
    if max(x1, x2) < xmin or min(x1, x2) > xmax: return False
    if max(y1, y2) < ymin or min(y1, y2) > ymax: return False
    return True

def build_jobs(agent_name, start_xy, wps, v_cap, w_cap, delta, t0, corridor_rect):
    """Return per-leg jobs with absolute release/deadline time and WCET.
       Also mark whether that leg uses the corridor."""
    pts = [start_xy] + [(x, y) for (x, y, _) in wps]
    jobs = []
    for i, ((gx, gy, ddl), (px, py)) in enumerate(zip(wps, pts)):
        C = wcet_leg(px, py, gx, gy, v_cap, w_cap, delta)
        uses_corr = segment_intersects_rect((px, py), (gx, gy), corridor_rect)
        jobs.append({
            "agent": agent_name,
            "name": f"{agent_name}_leg{i}",
            "release": float(t0),
            "deadline": float(t0 + ddl),
            "wcet": float(C),
            "uses_corridor": bool(uses_corr),
        })
    return jobs


# ---------- PDC (finite job set) ----------
def pdc_windows_finite(jobs):
    R = sorted({round(j["release"], 9) for j in jobs})
    D = sorted({round(j["deadline"], 9) for j in jobs})
    for t1 in R:
        for t2 in D:
            if t2 >= t1 - EPS:
                yield (t1, t2)

def pdc_check_finite(jobs, blocking=0.0):
    """Processor-Demand Criterion (necessary & sufficient for finite job sets)."""
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
    """Return a human-friendly relaxation hint from the worst violating window."""
    if cert["feasible"] or not cert["violations"]:
        return {"type": "none", "hint": "already feasible"}
    v = min(cert["violations"], key=lambda r: r["margin"])
    delta = -(v["margin"])
    return {
        "type": "deadline_or_wcet",
        "window": {"t1": v["t1"], "t2": v["t2"]},
        "needed_relax": delta,
        "suggestions": [
            {"action": "relax_deadline", "job": "any d<=t2 in window", "delta_d": delta},
            {"action": "shrink_wcet",    "job": "any job fully in window", "delta_C": delta},
        ],
    }


# ---------- SMT (QF_LRA) with shared-corridor mutex ----------
def export_smt2_multi(cert, jobs, eps_mutex, path):
    """
    Generate multi.smt2 that:
      - (optionally) lists PDC windows (named; not asserted by default)
      - enables models; declares e_<agent>_<leg> for corridor-using legs
      - encodes release/deadline/WCET bounds
      - adds pairwise mutual exclusion for corridor legs across agents
      - queries get-value for e_* and human-readable who-first flags
    """
    def f(x): return f"{x:.6f}"

    lines = []
    lines.append("(set-logic QF_LRA)")
    lines.append("(set-option :produce-unsat-cores true)")
    lines.append("(set-option :produce-models true)")
    lines.append("(set-option :pp.decimal true)") 

    # PDC windows for traceability
    for i, w in enumerate(cert["windows"]):
        lines.append(f"; window {i}: [{w['t1']}, {w['t2']}] lhs={w['lhs']:.6f} rhs={w['rhs']:.6f}")
        if INCLUDE_PDC_ASSERTS:
            lines.append(f"(assert (<= {w['lhs']:.9f} {w['rhs']:.9f}))")

    # corridor users only
    A_corr = [j for j in jobs if j["agent"] == "A" and j["uses_corridor"]]
    B_corr = [j for j in jobs if j["agent"] == "B" and j["uses_corridor"]]
    both = A_corr + B_corr

    def vname(j): return f"e_{j['name']}"

    # declare variables
    for j in both:
        lines.append(f"(declare-fun {vname(j)} () Real)")

    # release/deadline bounds (e_r in [release, deadline - C - eps])
    for j in both:
        e = vname(j)
        r, d, C = j["release"], j["deadline"], j["wcet"]
        lines.append(f"(assert (<= {f(r)} {e}))")
        lines.append(f"(assert (<= (+ {e} {f(C)} {f(eps_mutex)}) {f(d)}))")

    # non-overlap for each A/B corridor pair
    before_flags = []
    for a in A_corr:
        for b in B_corr:
            ea, eb = vname(a), vname(b)
            Ca, Cb = a["wcet"], b["wcet"]
            nm = f"mutex_{a['name']}_{b['name']}"
            lines.append(
                f"(assert (! (or "
                f"(<= (+ {ea} {f(Ca)} {f(eps_mutex)}) {eb}) "
                f"(<= (+ {eb} {f(Cb)} {f(eps_mutex)}) {ea})) :named {nm}))"
            )
            nm_ab = f"{a['name']}_before_{b['name']}"
            nm_ba = f"{b['name']}_before_{a['name']}"
            lines.append(f"(define-fun {nm_ab} () Bool (<= (+ {ea} {f(Ca)} {f(eps_mutex)}) {eb}))")
            lines.append(f"(define-fun {nm_ba} () Bool (<= (+ {eb} {f(Cb)} {f(eps_mutex)}) {ea}))")
            before_flags += [nm_ab, nm_ba]

    # solve + get values
    lines.append("(check-sat)")
    if both:
        varlist = " ".join(vname(j) for j in both)
        lines.append(f"(get-value ({varlist}))")
    if before_flags:
        lines.append(f"(get-value ({' '.join(before_flags)}))")

    Path(path).write_text("\n".join(lines))
    return path

def try_run_z3(smt2_path):
    try:
        out = subprocess.check_output(["z3", "-smt2", str(smt2_path)], text=True)
        return True, out.strip()
    except (FileNotFoundError, subprocess.CalledProcessError) as e:
        return None, str(e)


# ---------- parse z3 (get-value ...) ----------
_val_re = re.compile(r"\(\s*([^\s\)]+)\s+([^\s\)]+)\s*\)")
def parse_get_values(z3_out: str):
    """Parse (get-value ...) output into dict(name -> value/bool)."""
    vals = {}
    for name, val in _val_re.findall(z3_out):
        if val in ("true", "false"):
            vals[name] = (val == "true")
        else:
            try:
                vals[name] = float(val)
            except ValueError:
                vals[name] = val
    return vals


# ---------- write plan.json (with t0_mono) ----------
def write_plan_json(run_dir: Path, jobs, vals, corridor, t0_mono):
    """Write plan.json with gate's monotonic epoch, corridor schedule and order."""
    # pick corridor legs
    corr_names = {j["name"] for j in jobs if j.get("uses_corridor", False)}

    # e_* -> planned corridor-enter time (same monotonic epoch as gate t0_mono)
    release_times = {}
    for name, v in vals.items():
        if name.startswith("e_") and isinstance(v, (int, float)):
            leg = name[2:]  # e_A_leg1 -> A_leg1
            if (not corr_names) or (leg in corr_names):
                release_times[leg] = float(v)

    # human-readable “who first”
    order = []
    A_corr = [j["name"] for j in jobs if j["agent"]=="A" and j["uses_corridor"]]
    B_corr = [j["name"] for j in jobs if j["agent"]=="B" and j["uses_corridor"]]
    for a in A_corr:
        for b in B_corr:
            if vals.get(f"{a}_before_{b}", False):
                order.append({"a": a, "b": b, "first": "A"})
            elif vals.get(f"{b}_before_{a}", False):
                order.append({"a": a, "b": b, "first": "B"})

    plan = {
        "t0_mono": float(t0_mono),       # gate's monotonic epoch (seconds)
        "t0_wall": float(time.time()),   # wall clock for logging
        "corridor": corridor,
        "release_times": release_times,
        "order": order
    }
    (run_dir/"plan.json").write_text(json.dumps(plan, indent=2))
    print("[PLAN] saved", run_dir/"plan.json")


# ----------------- main -----------------
def main():
    # same monotonic epoch for all absolute times in this script
    t0 = time.monotonic()
    v_cap = worst_v_cap(VMAX_TABLE, V_MAX)
    w_cap = W_MAX

    # build jobs for both agents
    jobs = []
    for agent in ("A", "B"):
        a = AGENTS[agent]
        jobs += build_jobs(agent, a["start"], a["waypoints"],
                           v_cap, w_cap, DELTA_WCET, t0, CORRIDOR)

    # save jobs.json
    run_dir = Path(RUNS_DIR) / time.strftime("%Y%m%d-%H%M%S")
    run_dir.mkdir(parents=True, exist_ok=True)
    (run_dir / "jobs.json").write_text(json.dumps({"jobs": jobs}, indent=2))

    # PDC certificate
    pdc_cert = pdc_check_finite(jobs, blocking=GLOBAL_BLOCKING)
    (run_dir / "pdc_certificate.json").write_text(json.dumps(pdc_cert, indent=2))

    # SMT (with corridor mutex + models)
    smt2_path = run_dir / "multi.smt2"
    export_smt2_multi(pdc_cert, jobs, EPS_MUTEX, smt2_path)
    z3_ok, z3_out = try_run_z3(smt2_path)
    (run_dir / "z3_result.txt").write_text(z3_out if z3_out else "")

    if not pdc_cert["feasible"]:
        print("[GATE] FAIL (PDC infeasible). Hint:", minimal_repair_hint(pdc_cert))
        return

    print("[GATE] PASS (PDC). Artifacts:", run_dir)
    if not z3_ok:
        print("[Z3] not run / error")
        return

    # first line: sat/unsat
    first = z3_out.splitlines()[0] if z3_out else ""
    print("[Z3]", first)

    if first.strip() == "sat":
        vals = parse_get_values(z3_out)

        # pretty-print corridor enter times
        e_items = sorted([(k, v) for k, v in vals.items() if k.startswith("e_")],
                         key=lambda kv: kv[0])
        if e_items:
            print("  [enter times]")
            for k, v in e_items:
                try:
                    print(f"   - {k} = {float(v):.6f}")
                except Exception:
                    print(f"   - {k} = {v}")

        # pretty-print who-first flags
        bool_items = sorted([(k, v) for k, v in vals.items() if isinstance(v, bool)],
                            key=lambda kv: kv[0])
        if bool_items:
            print("  [who-first flags]")
            for k, v in bool_items:
                print(f"   - {k} : {v}")

        # write plan.json for runtime nodes (with gate's t0_mono)
        write_plan_json(run_dir, jobs, vals, CORRIDOR, t0_mono=t0)


if __name__ == "__main__":
    main()


