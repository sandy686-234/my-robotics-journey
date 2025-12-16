#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
gate_heterogeneous_scheduler.py

Heterogeneous Multi-Robot Task Allocation and Scheduling via SMT

Key Features:
- Task pool with capability requirements
- Heterogeneous robots with different capabilities and speeds
- SMT jointly decides: task assignment + ordering + timing
- Minimizes makespan while respecting constraints
"""

import math
import time
import json
import subprocess
from pathlib import Path

# ===================== CONFIGURATION =====================

# ========== Task Pool ==========
TASK_POOL = {
    "pickup_shelfA": {
        "location": (2.0, 0.0),
        "duration": 15.0,
        "deadline": 60.0,
        "requires_capability": "heavy_lift",
        "uses_resources": ["corridor_1"],
        "priority": 1,
        "description": "Pick heavy item from Shelf A"
    },
    "pickup_shelfB": {
        "location": (2.5, 1.0),
        "duration": 18.0,
        "deadline": 90.0,
        "requires_capability": "heavy_lift",
        "uses_resources": ["corridor_2"],
        "priority": 1,
        "description": "Pick heavy item from Shelf B"
    },
    "delivery_packing": {
        "location": (4.0, -1.0),
        "duration": 20.0,
        "deadline": 100.0,
        "requires_capability": "transport",
        "uses_resources": ["corridor_1", "elevator"],
        "priority": 2,
        "description": "Deliver to packing station"
    },
    "delivery_dock": {
        "location": (6.0, -1.0),
        "duration": 22.0,
        "deadline": 140.0,
        "requires_capability": "transport",
        "uses_resources": ["corridor_1", "loading_zone"],
        "priority": 1,
        "description": "Deliver to loading dock"
    },
    "quality_inspect": {
        "location": (5.0, 0.5),
        "duration": 25.0,
        "deadline": 130.0,
        "requires_capability": "sensor",
        "uses_resources": [],
        "priority": 2,
        "description": "Quality inspection"
    },
    "battery_charge": {
        "location": (1.5, 0.5),
        "duration": 30.0,
        "deadline": 150.0,
        "requires_capability": None,  # All robots can charge
        "uses_resources": ["charging_station"],
        "priority": 3,
        "description": "Battery charging"
    },
}

# ========== Robot Fleet ==========
ROBOTS = {
    "Forklift": {
        "id": "A",
        "start": (0.0, 0.0),
        "capabilities": ["heavy_lift", "transport"],
        "speed": 0.5,  # m/s
        "max_tasks": 3,
        "description": "Heavy-duty forklift robot"
    },
    "Courier": {
        "id": "B",
        "start": (0.0, 0.5),
        "capabilities": ["transport"],
        "speed": 0.7,  # m/s (faster)
        "max_tasks": 4,
        "description": "Fast courier robot"
    },
    "Inspector": {
        "id": "C",
        "start": (0.0, 1.0),
        "capabilities": ["sensor", "transport"],
        "speed": 0.4,  # m/s (slower but has sensors)
        "max_tasks": 3,
        "description": "Inspection robot with sensors"
    },
}

# ========== Shared Resources ==========
RESOURCES = {
    "corridor_1": {
        "type": "mutex",
        "rect": (3.0, 5.0, -0.5, 0.5),
        "traversal_time": 15.0,
    },
    "corridor_2": {
        "type": "mutex",
        "rect": (2.0, 3.0, 0.8, 1.2),
        "traversal_time": 12.0,
    },
    "elevator": {
        "type": "mutex",
        "rect": (4.0, 4.5, -1.5, -0.5),
        "traversal_time": 10.0,
    },
    "charging_station": {
        "type": "capacity",
        "limit": 2,
        "rect": (1.0, 2.0, 0.0, 1.0),
    },
    "loading_zone": {
        "type": "mutex",
        "rect": (5.5, 6.5, -1.5, -0.5),
        "traversal_time": 8.0,
    },
}

EPSILON = 0.1  # Safety margin
RUNS_DIR = "runs"
SOLVER_TIMEOUT = 120  # seconds

# ===================== HELPER FUNCTIONS =====================

def compute_travel_time(loc1, loc2, speed):
    """Compute travel time between two locations"""
    distance = math.hypot(loc2[0] - loc1[0], loc2[1] - loc1[1])
    return distance / max(speed, 0.01)


def can_robot_do_task(robot_data, task_data):
    """Check if robot has capability for task"""
    req_cap = task_data.get("requires_capability")
    if req_cap is None:
        return True  # Task has no special requirement
    return req_cap in robot_data["capabilities"]


# ===================== SMT ENCODING =====================

def export_smt2_heterogeneous(task_pool, robots, resources, output_path):
    """
    Generate SMT2 encoding for heterogeneous task allocation
    
    Decision Variables:
    - s_<task>: start time (Real)
    - assign_<task>_<robot>: assignment (0/1 Int)
    - order_<task1>_<task2>_<robot>: ordering (0/1 Int)
    
    Constraints:
    - Each task assigned to exactly one robot
    - Assignment respects capability requirements
    - Tasks on same robot don't overlap (with travel time)
    - Deadlines satisfied
    - Resource mutex constraints
    
    Objective:
    - Minimize makespan
    """
    
    lines = []
    lines.append("; Heterogeneous Multi-Robot Task Allocation via SMT")
    lines.append("(set-logic QF_LIRA)")  # Linear Integer + Real Arithmetic
    lines.append("(set-option :produce-models true)")
    lines.append("(set-option :produce-unsat-cores true)")
    lines.append("(set-option :pp.decimal true)")
    lines.append("")
    
    task_ids = list(task_pool.keys())
    robot_ids = [r["id"] for r in robots.values()]
    
    # ========== DECISION VARIABLES ==========
    
    lines.append("; === Decision Variables ===")
    lines.append("")
    
    # Start times
    lines.append("; Start times for each task")
    for task_id in task_ids:
        lines.append(f"(declare-const s_{task_id} Real)")
        lines.append(f"(assert (>= s_{task_id} 0.0))")
    lines.append("")
    
    # Assignment variables
    lines.append("; Assignment variables: assign_<task>_<robot> = 1 if assigned")
    for task_id in task_ids:
        for robot_id in robot_ids:
            var_name = f"assign_{task_id}_{robot_id}"
            lines.append(f"(declare-const {var_name} Int)")
            lines.append(f"(assert (or (= {var_name} 0) (= {var_name} 1)))")
    lines.append("")
    
    # Ordering variables (for tasks on same robot)
    lines.append("; Ordering variables: order_<t1>_<t2>_<robot> = 1 if t1 before t2")
    for robot_id in robot_ids:
        for i, t1 in enumerate(task_ids):
            for t2 in task_ids[i+1:]:
                var_name = f"order_{t1}_{t2}_{robot_id}"
                lines.append(f"(declare-const {var_name} Int)")
                lines.append(f"(assert (or (= {var_name} 0) (= {var_name} 1)))")
    lines.append("")
    
    # Makespan
    lines.append("; Makespan variable")
    lines.append("(declare-const makespan Real)")
    lines.append("(assert (>= makespan 0.0))")
    lines.append("")
    
    # ========== CONSTRAINTS ==========
    
    lines.append("; === Constraints ===")
    lines.append("")
    
    # Constraint 1: Each task assigned to exactly one robot
    lines.append("; C1: Each task assigned to exactly one robot")
    for task_id in task_ids:
        assign_vars = [f"assign_{task_id}_{rid}" for rid in robot_ids]
        sum_expr = " ".join(assign_vars)
        lines.append(f"(assert (= (+ {sum_expr}) 1))")
    lines.append("")
    
    # Constraint 2: Capability matching
    lines.append("; C2: Capability constraints")
    for task_id, task_data in task_pool.items():
        req_cap = task_data.get("requires_capability")
        if req_cap:
            for robot_name, robot_data in robots.items():
                robot_id = robot_data["id"]
                if req_cap not in robot_data["capabilities"]:
                    # This robot cannot do this task
                    lines.append(f"(assert (= assign_{task_id}_{robot_id} 0))")
    lines.append("")
    
    # Constraint 3: Max tasks per robot (optional)
    lines.append("; C3: Maximum tasks per robot")
    for robot_name, robot_data in robots.items():
        robot_id = robot_data["id"]
        max_tasks = robot_data.get("max_tasks", 999)
        assign_vars = [f"assign_{tid}_{robot_id}" for tid in task_ids]
        sum_expr = " ".join(assign_vars)
        lines.append(f"(assert (<= (+ {sum_expr}) {max_tasks}))")
    lines.append("")
    
    # Constraint 4: Intra-robot task sequencing
    lines.append("; C4: Tasks on same robot must not overlap (with travel time)")
    for robot_name, robot_data in robots.items():
        robot_id = robot_data["id"]
        robot_speed = robot_data["speed"]
        robot_start = robot_data["start"]
        
        for i, t1 in enumerate(task_ids):
            for t2 in task_ids[i+1:]:
                t1_data = task_pool[t1]
                t2_data = task_pool[t2]
                
                # Compute travel time from t1 to t2
                travel_12 = compute_travel_time(
                    t1_data["location"], 
                    t2_data["location"], 
                    robot_speed
                )
                
                travel_21 = compute_travel_time(
                    t2_data["location"], 
                    t1_data["location"], 
                    robot_speed
                )
                
                # If both assigned to this robot AND t1 before t2
                # Then s_t2 >= s_t1 + duration_t1 + travel_12
                lines.append(
                    f"(assert (=> "
                    f"(and (= assign_{t1}_{robot_id} 1) "
                    f"(= assign_{t2}_{robot_id} 1) "
                    f"(= order_{t1}_{t2}_{robot_id} 1)) "
                    f"(>= s_{t2} (+ s_{t1} {t1_data['duration']:.2f} {travel_12:.2f}))))"
                )
                
                # If both assigned AND t2 before t1
                lines.append(
                    f"(assert (=> "
                    f"(and (= assign_{t1}_{robot_id} 1) "
                    f"(= assign_{t2}_{robot_id} 1) "
                    f"(= order_{t1}_{t2}_{robot_id} 0)) "
                    f"(>= s_{t1} (+ s_{t2} {t2_data['duration']:.2f} {travel_21:.2f}))))"
                )
        
        # Initial travel from start position
        lines.append(f"; Initial travel from start for robot {robot_id}")
        for task_id, task_data in task_pool.items():
            initial_travel = compute_travel_time(
                robot_start,
                task_data["location"],
                robot_speed
            )
            # If this is the first task for this robot, add initial travel
            # (Simplified: just add to all tasks on this robot)
            lines.append(
                f"(assert (=> (= assign_{task_id}_{robot_id} 1) "
                f"(>= s_{task_id} {initial_travel:.2f})))"
            )
    lines.append("")
    
    # Constraint 5: Deadlines
    lines.append("; C5: Deadline constraints")
    for task_id, task_data in task_pool.items():
        deadline = task_data["deadline"]
        duration = task_data["duration"]
        lines.append(
            f"(assert (<= (+ s_{task_id} {duration:.2f}) {deadline:.2f}))"
        )
    lines.append("")
    
    # Constraint 6: Resource mutex
    lines.append("; C6: Resource mutual exclusion")
    for res_name, res_data in resources.items():
        if res_data["type"] != "mutex":
            continue
        
        # Find tasks using this resource
        users = [tid for tid, tdata in task_pool.items() 
                 if res_name in tdata.get("uses_resources", [])]
        
        if len(users) < 2:
            continue
        
        lines.append(f"; Resource: {res_name}")
        traversal = res_data.get("traversal_time", 10.0)
        
        for i, t1 in enumerate(users):
            for t2 in users[i+1:]:
                d1 = task_pool[t1]["duration"]
                d2 = task_pool[t2]["duration"]
                
                # Mutex: either t1 finishes before t2 starts, or vice versa
                lines.append(
                    f"(assert (or "
                    f"(<= (+ s_{t1} {d1:.2f} {EPSILON:.2f}) s_{t2}) "
                    f"(<= (+ s_{t2} {d2:.2f} {EPSILON:.2f}) s_{t1})))"
                )
    lines.append("")
    
    # Constraint 7: Makespan definition
    lines.append("; C7: Makespan = max completion time")
    for task_id, task_data in task_pool.items():
        duration = task_data["duration"]
        lines.append(f"(assert (>= makespan (+ s_{task_id} {duration:.2f})))")
    lines.append("")
    
    # ========== OBJECTIVE ==========
    
    lines.append("; === Objective ===")
    lines.append("(minimize makespan)")
    lines.append("")
    
    # ========== SOLVE ==========
    
    lines.append("(check-sat)")
    lines.append("(get-model)")
    
    # Write to file
    Path(output_path).write_text("\n".join(lines))
    print(f"[SMT] Generated {len(lines)} lines of SMT2 encoding")
    return output_path


# ===================== SOLVER ======================

def run_z3_solver(smt2_path, timeout=120):
    """Run Z3 solver"""
    try:
        result = subprocess.run(
            ["z3", f"-T:{timeout}", "-smt2", str(smt2_path)],
            capture_output=True,
            text=True,
            timeout=timeout + 10,
        )
        return True, result.stdout
    except FileNotFoundError:
        return None, "Z3 not found in PATH"
    except subprocess.TimeoutExpired:
        return False, "Solver timeout"
    except Exception as e:
        return False, str(e)


# ===================== RESULT PARSING =====================

def parse_z3_result(z3_output, task_pool, robots):
    """Parse Z3 output to extract task assignments and schedule"""
    import re
    from fractions import Fraction

    lines = z3_output.strip().split("\n")

    if not lines or lines[0].strip() != "sat":
        return None

    values = {}

    def _parse_num(token: str):
        token = token.strip()
        # Z3 may output rationals as '(/ a b)' or plain 'a/b'
        m = re.match(r'\(\s*/\s*([^\s\)]+)\s+([^\s\)]+)\s*\)', token)
        if m:
            try:
                return float(Fraction(m.group(1))) / float(Fraction(m.group(2)))
            except Exception:
                return None
        # plain a/b
        if "/" in token:
            try:
                return float(Fraction(token))
            except Exception:
                pass
        # plain float/int
        try:
            return float(token)
        except Exception:
            return None

    # 1) parse define-fun blocks: (define-fun NAME () Real <value>)
    for m in re.finditer(r'\(define-fun\s+([^\s\)]+)\s*\(\)\s*\w+\s*([^\)]+)\)', z3_output, re.DOTALL):
        name = m.group(1)
        val_str = m.group(2).strip()
        parsed = _parse_num(val_str)
        values[name] = parsed if parsed is not None else val_str

    # 2) also accept simple pairs (get-value style): ((name val) ...)
    after_sat = z3_output.split("sat", 1)[-1]
    pairs = re.findall(r'\(\s*([a-zA-Z0-9_]+)\s+([^\)\s]+)\s*\)', after_sat)
    for name, val in pairs:
        if name in values:
            continue
        parsed = _parse_num(val)
        values[name] = parsed if parsed is not None else val
    
    # Extract makespan
    makespan = values.get("makespan", None)
    
    # Extract task assignments
    task_assignments = {}
    for task_id in task_pool.keys():
        for robot_name, robot_data in robots.items():
            robot_id = robot_data["id"]
            assign_var = f"assign_{task_id}_{robot_id}"
            if values.get(assign_var, 0) == 1:
                task_assignments[task_id] = robot_id
                break
    
    # Extract start times
    start_times = {}
    for task_id in task_pool.keys():
        start_var = f"s_{task_id}"
        start_times[task_id] = values.get(start_var, 0.0)
    
    # Build schedule per robot
    schedule = {}
    for robot_name, robot_data in robots.items():
        robot_id = robot_data["id"]
        robot_tasks = []
        
        for task_id, assigned_robot in task_assignments.items():
            if assigned_robot == robot_id:
                task_data = task_pool[task_id]
                robot_tasks.append({
                    "task_id": task_id,
                    "description": task_data["description"],
                    "start_time": start_times[task_id],
                    "duration": task_data["duration"],
                    "end_time": start_times[task_id] + task_data["duration"],
                    "location": task_data["location"],
                })
        
        # Sort by start time
        robot_tasks.sort(key=lambda t: t["start_time"])
        schedule[robot_name] = robot_tasks
    
    return {
        "makespan": makespan,
        "task_assignments": task_assignments,
        "start_times": start_times,
        "schedule": schedule,
    }


# ===================== EXPORT =====================

def export_schedule_json(result, task_pool, robots, resources, run_dir):
    """Export schedule to JSON file"""
    
    output = {
        "timestamp": time.time(),
        "makespan": result["makespan"],
        "task_assignments": result["task_assignments"],
        "schedule": result["schedule"],
        "task_pool": task_pool,
        "robots": {name: {
            "id": data["id"],
            "capabilities": data["capabilities"],
            "speed": data["speed"]
        } for name, data in robots.items()},
        "resources": resources,
    }
    
    output_path = run_dir / "schedule_heterogeneous.json"
    output_path.write_text(json.dumps(output, indent=2))
    print(f"[EXPORT] Saved schedule to {output_path}")
    
    return output


def print_schedule_summary(result, robots):
    """Print human-readable schedule summary"""
    print("\n" + "="*60)
    print("HETEROGENEOUS TASK ALLOCATION RESULT")
    print("="*60)
    
    # Safely handle makespan
    makespan_val = result.get('makespan')
    if makespan_val is not None:
        try:
            # Handle fractions or strings
            if isinstance(makespan_val, str):
                if "/" in makespan_val:
                    parts = makespan_val.split("/")
                    makespan_val = float(parts[0]) / float(parts[1])
                else:
                    makespan_val = float(makespan_val)
            else:
                makespan_val = float(makespan_val)
            print(f"Optimal Makespan: {makespan_val:.2f}s")
        except (ValueError, TypeError) as e:
            print(f"Optimal Makespan: {makespan_val} (parse error: {e})")
    else:
        print("Optimal Makespan: N/A")
    
    print("")
    
    for robot_name, tasks in result["schedule"].items():
        robot_data = robots[robot_name]
        print(f"Robot {robot_name} ({robot_data['id']}) - {robot_data['description']}:")
        print(f"  Capabilities: {', '.join(robot_data['capabilities'])}")
        print(f"  Speed: {robot_data['speed']} m/s")
        print(f"  Assigned Tasks ({len(tasks)}):")
        
        if not tasks:
            print("    (No tasks assigned)")
        else:
            for task in tasks:
                print(f"    • {task['task_id']}: {task['description']}")
                
                # Safely handle times
                try:
                    start = float(task['start_time'])
                    end = float(task['end_time'])
                    print(f"      Time: [{start:.1f}s - {end:.1f}s]")
                except (ValueError, TypeError):
                    print(f"      Time: [{task['start_time']} - {task['end_time']}]")
                
                print(f"      Location: {task['location']}")
        print("")


# ===================== MAIN =====================

def main():
    print("="*60)
    print("Heterogeneous Multi-Robot Task Scheduler (SMT)")
    print("="*60)
    print("")
    
    # Validate configuration
    print("[CONFIG] Validating task pool and robot capabilities...")
    
    # Check if all tasks can be done by at least one robot
    for task_id, task_data in TASK_POOL.items():
        req_cap = task_data.get("requires_capability")
        if req_cap:
            capable_robots = [
                name for name, rdata in ROBOTS.items()
                if req_cap in rdata["capabilities"]
            ]
            if not capable_robots:
                print(f"[ERROR] No robot can do task '{task_id}' "
                      f"(requires '{req_cap}')")
                return
            print(f"  Task '{task_id}' can be done by: {', '.join(capable_robots)}")
    
    print(f"\n[CONFIG] Task pool: {len(TASK_POOL)} tasks")
    print(f"[CONFIG] Robot fleet: {len(ROBOTS)} robots")
    print(f"[CONFIG] Shared resources: {len(RESOURCES)}")
    print("")
    
    # Create run directory
    run_dir = Path(RUNS_DIR) / time.strftime("%Y%m%d-%H%M%S")
    run_dir.mkdir(parents=True, exist_ok=True)
    
    # Export configuration
    config = {
        "task_pool": TASK_POOL,
        "robots": ROBOTS,
        "resources": RESOURCES,
    }
    (run_dir / "config.json").write_text(json.dumps(config, indent=2))
    
    # Generate SMT encoding
    print("[SMT] Generating SMT2 encoding...")
    smt2_path = run_dir / "heterogeneous.smt2"
    export_smt2_heterogeneous(TASK_POOL, ROBOTS, RESOURCES, smt2_path)
    print(f"[SMT] Saved to {smt2_path}")
    print("")
    
    # Solve
    print(f"[Z3] Running optimizer (timeout={SOLVER_TIMEOUT}s)...")
    solve_start = time.time()
    z3_ok, z3_output = run_z3_solver(smt2_path, timeout=SOLVER_TIMEOUT)
    solve_time = time.time() - solve_start
    
    # Save Z3 output
    (run_dir / "z3_result.txt").write_text(z3_output if z3_output else "")
    
    if not z3_ok:
        print(f"[Z3] ERROR: {z3_output}")
        print("\n[RESULT] FAIL - Solver error")
        return
    
    first_line = z3_output.strip().split("\n")[0] if z3_output else ""
    print(f"[Z3] Result: {first_line} (solved in {solve_time:.3f}s)")
    print("")
    
    if first_line.strip() != "sat":
        print("[RESULT] UNSAT - No feasible schedule found")
        print("\nPossible reasons:")
        print("  • Deadlines too tight")
        print("  • Resource conflicts unresolvable")
        print("  • No robot has required capabilities")
        print("\nSuggestions:")
        print("  • Relax deadlines")
        print("  • Add more robots")
        print("  • Reduce task durations")
        return
    
    # Parse result
    print("[PARSE] Extracting schedule from model...")
    result = parse_z3_result(z3_output, TASK_POOL, ROBOTS)
    
    if not result:
        print("[ERROR] Failed to parse Z3 output")
        return
    
    # Export schedule
    export_schedule_json(result, TASK_POOL, ROBOTS, RESOURCES, run_dir)
    
    # Print summary
    print_schedule_summary(result, ROBOTS)
    
    # Statistics
    print("="*60)
    print("STATISTICS")
    print("="*60)
    print(f"Solver time: {solve_time:.3f}s")
    # Safely print makespan (may be None or rational string)
    makespan_val = result.get('makespan')
    if makespan_val is not None:
        try:
            if isinstance(makespan_val, str) and "/" in makespan_val:
                p, q = makespan_val.split("/", 1)
                makespan_val = float(p) / float(q)
            else:
                makespan_val = float(makespan_val)
            print(f"Makespan: {makespan_val:.2f}s")
        except (ValueError, TypeError):
            print(f"Makespan: {makespan_val} (unparsed)")
    else:
        print("Makespan: N/A")

    print(f"Tasks assigned: {len(result['task_assignments'])}/{len(TASK_POOL)}")
    
    # Load distribution
    print("\nTask distribution:")
    for robot_name, tasks in result["schedule"].items():
        print(f"  {robot_name}: {len(tasks)} tasks")
    
    print("\n[SUCCESS] All artifacts saved to:", run_dir)
    print("\nKey files:")
    print(f"  • {run_dir}/heterogeneous.smt2 - SMT encoding")
    print(f"  • {run_dir}/z3_result.txt - Solver output")
    print(f"  • {run_dir}/schedule_heterogeneous.json - Executable schedule")
    print("")


if __name__ == "__main__":
    main()

