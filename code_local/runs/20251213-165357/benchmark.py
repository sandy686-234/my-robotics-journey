"""
Multi-Robot Scheduling Comparison: VeriROS-Exec vs Baselines
Compares 4 methods: SMT (VeriROS), MILP, Naive EDF, Priority-based
"""

import time
import numpy as np
from typing import List, Dict, Tuple, Optional
from dataclasses import dataclass
from z3 import *
try:
    from gurobipy import Model, GRB, quicksum
    GUROBI_AVAILABLE = True
except ImportError:
    GUROBI_AVAILABLE = False
    print("Warning: Gurobi not available. MILP baseline will be skipped.")


@dataclass
class Task:
    """Task definition"""
    id: str
    requirements: List[str]  # Required capabilities
    duration: float  # seconds
    deadline: float  # seconds
    resources: List[str]  # Shared resources needed


@dataclass
class Robot:
    """Robot definition"""
    id: str
    capabilities: List[str]
    speed: float  # m/s
    

@dataclass
class Resource:
    """Shared resource (corridor, elevator, etc.)"""
    id: str
    capacity: int = 1  # Usually 1 for mutex


class ProblemInstance:
    """3 Robots, 6 Tasks, 5 Shared Resources scenario"""
    def __init__(self):
        # Robot fleet
        self.robots = [
            Robot("forklift", ["lift", "transport"], 0.5),
            Robot("courier", ["transport"], 0.7),
            Robot("inspector", ["sensor", "transport"], 0.4)
        ]
        
        # Task pool
        self.tasks = [
            Task("pickup_shelfA", ["lift"], 20.0, 100.0, ["corridor1"]),
            Task("pickup_shelfB", ["lift"], 18.0, 110.0, ["corridor1"]),
            Task("delivery_packing", ["transport"], 15.0, 120.0, ["corridor1", "elevator1"]),
            Task("delivery_dock", ["transport"], 22.0, 130.0, ["corridor1", "elevator1"]),
            Task("quality_inspect", ["sensor"], 25.0, 140.0, ["lab_area"]),
            Task("battery_charge", [], 30.0, 200.0, ["charging_station"])
        ]
        
        # Shared resources
        self.resources = [
            Resource("corridor1"),
            Resource("elevator1"),
            Resource("lab_area"),
            Resource("charging_station"),
            Resource("docking_area")
        ]
        
        # Travel times between tasks (simplified)
        self.travel_time = 5.0  # Average travel time


# ============================================================================
# METHOD 1: VeriROS-Exec (SMT Optimization)
# ============================================================================

class VeriROSSMTScheduler:
    """SMT-based optimal scheduler using Z3"""
    
    def __init__(self, problem: ProblemInstance):
        self.problem = problem
        self.solver = Optimize()
        self.start_times = {}
        self.assignments = {}
        self.orderings = {}
        
    def solve(self) -> Tuple[float, int, float, float]:
        """
        Returns: (makespan, conflicts, deadline_met_rate, solver_time)
        """
        start = time.time()
        
        tasks = self.problem.tasks
        robots = self.problem.robots
        n_tasks = len(tasks)
        n_robots = len(robots)
        
        # Decision variables
        # Start times for each task
        for i, task in enumerate(tasks):
            self.start_times[i] = Real(f's_{task.id}')
            self.solver.add(self.start_times[i] >= 0)
        
        # Assignment variables: assign[t][r] = 1 if task t assigned to robot r
        for i in range(n_tasks):
            for j in range(n_robots):
                self.assignments[(i, j)] = Bool(f'assign_{i}_{j}')
        
        # Ordering variables: order[t1][t2][r] = 1 if t1 before t2 on robot r
        for i in range(n_tasks):
            for k in range(i+1, n_tasks):
                for j in range(n_robots):
                    self.orderings[(i, k, j)] = Bool(f'order_{i}_{k}_{j}')
        
        # C1: Each task assigned to exactly one robot
        for i in range(n_tasks):
            self.solver.add(
                Sum([If(self.assignments[(i, j)], 1, 0) 
                     for j in range(n_robots)]) == 1
            )
        
        # C2: Capability constraints
        for i, task in enumerate(tasks):
            for j, robot in enumerate(robots):
                if task.requirements:
                    # Check if robot has all required capabilities
                    has_caps = all(cap in robot.capabilities 
                                  for cap in task.requirements)
                    if not has_caps:
                        self.solver.add(Not(self.assignments[(i, j)]))
        
        # C3: Sequencing - tasks on same robot don't overlap
        for i in range(n_tasks):
            for k in range(i+1, n_tasks):
                for j in range(n_robots):
                    # If both tasks on same robot, they must be ordered
                    both_on_j = And(self.assignments[(i, j)], 
                                   self.assignments[(k, j)])
                    
                    # If i before k
                    self.solver.add(
                        Implies(
                            And(both_on_j, self.orderings[(i, k, j)]),
                            self.start_times[k] >= 
                            self.start_times[i] + tasks[i].duration + 
                            self.problem.travel_time
                        )
                    )
                    
                    # If k before i
                    self.solver.add(
                        Implies(
                            And(both_on_j, Not(self.orderings[(i, k, j)])),
                            self.start_times[i] >= 
                            self.start_times[k] + tasks[k].duration + 
                            self.problem.travel_time
                        )
                    )
                    
                    # Must pick an order if on same robot
                    self.solver.add(
                        Implies(both_on_j, 
                               Or(self.orderings[(i, k, j)],
                                  Not(self.orderings[(i, k, j)])))
                    )
        
        # C4: Resource mutex constraints
        for res in self.problem.resources:
            # Find tasks that use this resource
            task_indices = [i for i, t in enumerate(tasks) 
                          if res.id in t.resources]
            
            # Any two tasks using this resource must not overlap
            for i in task_indices:
                for k in task_indices:
                    if i < k:
                        # Either i finishes before k starts, or vice versa
                        self.solver.add(
                            Or(
                                self.start_times[i] + tasks[i].duration <= 
                                self.start_times[k],
                                self.start_times[k] + tasks[k].duration <= 
                                self.start_times[i]
                            )
                        )
        
        # C5: Deadline constraints
        for i, task in enumerate(tasks):
            self.solver.add(
                self.start_times[i] + task.duration <= task.deadline
            )
        
        # Optimization: Minimize makespan
        makespan = Real('makespan')
        for i, task in enumerate(tasks):
            self.solver.add(makespan >= self.start_times[i] + task.duration)
        
        self.solver.minimize(makespan)
        
        # Solve
        result = self.solver.check()
        solve_time = time.time() - start
        
        if result == sat:
            model = self.solver.model()
            makespan_val = float(model[makespan].as_fraction())
            
            # Check deadlines
            deadlines_met = 0
            for i, task in enumerate(tasks):
                start_val = float(model[self.start_times[i]].as_fraction())
                if start_val + task.duration <= task.deadline:
                    deadlines_met += 1
            
            return makespan_val, 0, deadlines_met / n_tasks, solve_time
        else:
            return float('inf'), 0, 0.0, solve_time


# ============================================================================
# METHOD 2: MILP (Gurobi)
# ============================================================================

class MILPScheduler:
    """MILP-based scheduler using Gurobi"""
    
    def __init__(self, problem: ProblemInstance):
        self.problem = problem
        
    def solve(self) -> Tuple[float, int, float, float]:
        if not GUROBI_AVAILABLE:
            return float('inf'), 0, 0.0, 0.0
        
        start = time.time()
        
        tasks = self.problem.tasks
        robots = self.problem.robots
        n_tasks = len(tasks)
        n_robots = len(robots)
        
        m = Model("MILP_Scheduler")
        m.setParam('OutputFlag', 0)  # Suppress output
        
        # Variables
        s = {}  # Start times
        assign = {}  # Assignment
        order = {}  # Ordering
        makespan = m.addVar(lb=0, name='makespan')
        
        for i in range(n_tasks):
            s[i] = m.addVar(lb=0, name=f's_{i}')
        
        for i in range(n_tasks):
            for j in range(n_robots):
                assign[i, j] = m.addVar(vtype=GRB.BINARY, name=f'a_{i}_{j}')
        
        for i in range(n_tasks):
            for k in range(i+1, n_tasks):
                for j in range(n_robots):
                    order[i, k, j] = m.addVar(vtype=GRB.BINARY, 
                                             name=f'o_{i}_{k}_{j}')
        
        m.update()
        
        # Constraints
        # Each task to one robot
        for i in range(n_tasks):
            m.addConstr(quicksum(assign[i, j] for j in range(n_robots)) == 1)
        
        # Capability constraints
        for i, task in enumerate(tasks):
            for j, robot in enumerate(robots):
                if task.requirements:
                    has_caps = all(cap in robot.capabilities 
                                  for cap in task.requirements)
                    if not has_caps:
                        m.addConstr(assign[i, j] == 0)
        
        # Sequencing
        M = 1000  # Big M
        for i in range(n_tasks):
            for k in range(i+1, n_tasks):
                for j in range(n_robots):
                    both = m.addVar(vtype=GRB.BINARY)
                    m.addConstr(both <= assign[i, j])
                    m.addConstr(both <= assign[k, j])
                    m.addConstr(both >= assign[i, j] + assign[k, j] - 1)
                    
                    # If both on robot j and i before k
                    m.addConstr(
                        s[k] >= s[i] + tasks[i].duration + 
                        self.problem.travel_time - M * (2 - both - order[i, k, j])
                    )
                    
                    # If both on robot j and k before i
                    m.addConstr(
                        s[i] >= s[k] + tasks[k].duration + 
                        self.problem.travel_time - M * (1 + both - order[i, k, j])
                    )
        
        # Resource mutex
        for res in self.problem.resources:
            task_indices = [i for i, t in enumerate(tasks) 
                          if res.id in t.resources]
            for i in task_indices:
                for k in task_indices:
                    if i < k:
                        y = m.addVar(vtype=GRB.BINARY)
                        m.addConstr(
                            s[i] + tasks[i].duration <= s[k] + M * y
                        )
                        m.addConstr(
                            s[k] + tasks[k].duration <= s[i] + M * (1 - y)
                        )
        
        # Deadlines
        for i, task in enumerate(tasks):
            m.addConstr(s[i] + task.duration <= task.deadline)
        
        # Makespan
        for i in range(n_tasks):
            m.addConstr(makespan >= s[i] + tasks[i].duration)
        
        # Objective
        m.setObjective(makespan, GRB.MINIMIZE)
        
        # Solve
        m.optimize()
        solve_time = time.time() - start
        
        if m.status == GRB.OPTIMAL:
            makespan_val = makespan.X
            deadlines_met = sum(1 for i in range(n_tasks) 
                              if s[i].X + tasks[i].duration <= tasks[i].deadline)
            return makespan_val, 0, deadlines_met / n_tasks, solve_time
        else:
            return float('inf'), 0, 0.0, solve_time


# ============================================================================
# METHOD 3: Naive EDF (Earliest Deadline First)
# ============================================================================

class NaiveEDFScheduler:
    """Online EDF scheduler - greedy, no look-ahead"""
    
    def __init__(self, problem: ProblemInstance):
        self.problem = problem
        
    def solve(self) -> Tuple[float, int, float, float]:
        start_time = time.time()
        
        tasks = sorted(self.problem.tasks, key=lambda t: t.deadline)
        robots = self.problem.robots
        
        # Track when each robot is free
        robot_available = {r.id: 0.0 for r in robots}
        # Track resource usage
        resource_usage = {res.id: [] for res in self.problem.resources}
        
        schedule = []
        conflicts = 0
        
        for task in tasks:
            # Find first capable robot
            capable_robots = [r for r in robots 
                            if all(cap in r.capabilities 
                                  for cap in task.requirements)]
            
            if not capable_robots:
                continue
            
            # Pick robot that's available earliest
            robot = min(capable_robots, key=lambda r: robot_available[r.id])
            
            # Schedule at earliest available time
            start = robot_available[robot.id]
            end = start + task.duration
            
            # Check resource conflicts (simplified - just count overlaps)
            for res_id in task.resources:
                for (res_start, res_end) in resource_usage[res_id]:
                    if not (end <= res_start or start >= res_end):
                        conflicts += 1
                resource_usage[res_id].append((start, end))
            
            schedule.append((task, robot, start, end))
            robot_available[robot.id] = end + self.problem.travel_time
        
        makespan = max(end for _, _, _, end in schedule)
        deadlines_met = sum(1 for task, _, _, end in schedule 
                          if end <= task.deadline)
        
        solve_time = time.time() - start_time
        return makespan, conflicts, deadlines_met / len(tasks), solve_time


# ============================================================================
# METHOD 4: Priority-Based (Fixed priorities)
# ============================================================================

class PriorityScheduler:
    """Fixed priority scheduler based on task requirements"""
    
    def __init__(self, problem: ProblemInstance):
        self.problem = problem
        
    def task_priority(self, task: Task) -> int:
        """Higher priority for specialized tasks"""
        if "lift" in task.requirements:
            return 3
        elif "sensor" in task.requirements:
            return 2
        elif "transport" in task.requirements:
            return 1
        else:
            return 0
    
    def solve(self) -> Tuple[float, int, float, float]:
        start_time = time.time()
        
        # Sort by priority, then deadline
        tasks = sorted(self.problem.tasks, 
                      key=lambda t: (-self.task_priority(t), t.deadline))
        robots = self.problem.robots
        
        robot_available = {r.id: 0.0 for r in robots}
        resource_usage = {res.id: [] for res in self.problem.resources}
        
        schedule = []
        conflicts = 0
        
        for task in tasks:
            capable_robots = [r for r in robots 
                            if all(cap in r.capabilities 
                                  for cap in task.requirements)]
            
            if not capable_robots:
                continue
            
            robot = min(capable_robots, key=lambda r: robot_available[r.id])
            start = robot_available[robot.id]
            end = start + task.duration
            
            # Check conflicts
            for res_id in task.resources:
                for (res_start, res_end) in resource_usage[res_id]:
                    if not (end <= res_start or start >= res_end):
                        conflicts += 1
                resource_usage[res_id].append((start, end))
            
            schedule.append((task, robot, start, end))
            robot_available[robot.id] = end + self.problem.travel_time
        
        makespan = max(end for _, _, _, end in schedule)
        deadlines_met = sum(1 for task, _, _, end in schedule 
                          if end <= task.deadline)
        
        solve_time = time.time() - start_time
        return makespan, conflicts, deadlines_met / len(tasks), solve_time


# ============================================================================
# BENCHMARK RUNNER
# ============================================================================

def run_benchmark(n_runs: int = 20):
    """Run comparison benchmark"""
    
    print("="*80)
    print("Multi-Robot Scheduling Benchmark")
    print("Scenario: 3 Robots, 6 Tasks, 5 Shared Resources")
    print("="*80)
    
    problem = ProblemInstance()
    
    results = {
        'VeriROS (SMT Opt.)': [],
        'MILP (Gurobi)': [],
        'Naive EDF (Online)': [],
        'Priority (Fixed)': []
    }
    
    # Run multiple times for statistics
    for run in range(n_runs):
        print(f"\nRun {run+1}/{n_runs}...")
        
        # VeriROS SMT
        scheduler = VeriROSSMTScheduler(problem)
        result = scheduler.solve()
        results['VeriROS (SMT Opt.)'].append(result)
        
        # MILP
        if GUROBI_AVAILABLE:
            scheduler = MILPScheduler(problem)
            result = scheduler.solve()
            results['MILP (Gurobi)'].append(result)
        
        # Naive EDF
        scheduler = NaiveEDFScheduler(problem)
        result = scheduler.solve()
        results['Naive EDF (Online)'].append(result)
        
        # Priority
        scheduler = PriorityScheduler(problem)
        result = scheduler.solve()
        results['Priority (Fixed)'].append(result)
    
    # Print results
    print("\n" + "="*80)
    print("RESULTS (Average over {} runs)".format(n_runs))
    print("="*80)
    print(f"{'Method':<25} {'Makespan (s)':<15} {'Conflicts':<12} {'Deadlines Met':<18} {'Solver Time':<15}")
    print("-"*80)
    
    for method, data in results.items():
        if not data:
            continue
        
        makespans = [d[0] for d in data]
        conflicts = [d[1] for d in data]
        deadlines = [d[2] for d in data]
        times = [d[3] for d in data]
        
        avg_makespan = np.mean(makespans)
        avg_conflicts = np.mean(conflicts)
        avg_deadlines = np.mean(deadlines)
        avg_time = np.mean(times)
        
        time_str = f"{avg_time:.3f}s" if avg_time > 0.001 else "N/A (Runtime)"
        
        print(f"{method:<25} {avg_makespan:<15.1f} {avg_conflicts:<12.1f} "
              f"{avg_deadlines*100:.1f}% ({avg_deadlines*6:.1f}/6)  {time_str:<15}")
    
    # Print improvements
    print("\n" + "="*80)
    print("IMPROVEMENTS (VeriROS vs. Baselines):")
    print("="*80)
    
    veri_makespan = np.mean([d[0] for d in results['VeriROS (SMT Opt.)']])
    edf_makespan = np.mean([d[0] for d in results['Naive EDF (Online)']])
    
    edf_improvement = (edf_makespan - veri_makespan) / edf_makespan * 100
    print(f"▶ vs. Naive EDF: {edf_improvement:.1f}% faster, eliminated all conflicts")
    
    if results['MILP (Gurobi)']:
        milp_makespan = np.mean([d[0] for d in results['MILP (Gurobi)']])
        milp_time = np.mean([d[3] for d in results['MILP (Gurobi)']])
        veri_time = np.mean([d[3] for d in results['VeriROS (SMT Opt.)']])
        
        makespan_improvement = (milp_makespan - veri_makespan) / milp_makespan * 100
        time_speedup = milp_time / veri_time
        
        print(f"▶ vs. MILP: {makespan_improvement:.1f}% faster makespan, "
              f"{time_speedup:.0f}× faster solving")


if __name__ == "__main__":
    print("Starting benchmark...")
    print("Note: This will take a few seconds to complete.\n")
    
    # Run with fewer iterations for quick test
    run_benchmark(n_runs=5)
    
    print("\n" + "="*80)
    print("Benchmark complete!")
    print("="*80)