#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
visualize_heterogeneous.py

Visualize heterogeneous task allocation results
"""

import json
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
from pathlib import Path
import sys
import argparse
import glob

def load_schedule(json_path):
    """Load schedule from JSON"""
    with open(json_path) as f:
        return json.load(f)


def plot_gantt_chart(schedule_data, output_path):
    """Plot Gantt chart showing task allocation"""
    
    schedule = schedule_data.get("schedule", {})
    makespan = schedule_data.get("makespan", None)

    fig, ax = plt.subplots(figsize=(14, 6))
    
    # Colors for different task types
    task_colors = {
        "pickup": "#FF6B6B",
        "delivery": "#4ECDC4",
        "inspect": "#45B7D1",
        "charge": "#FFA07A",
    }
    
    robot_names = list(schedule.keys())
    y_positions = {name: i for i, name in enumerate(robot_names)}
    
    # Plot tasks
    # keep a computed max end time in case makespan is missing
    _computed_max_end = 0.0
    for robot_name, tasks in schedule.items():
        y = y_positions[robot_name]

        for task in tasks:
            task_id = task["task_id"]
            start = task["start_time"]
            duration = task["duration"]
            
            # Determine color based on task type
            color = "#95E1D3"  # default
            for task_type, c in task_colors.items():
                if task_type in task_id:
                    color = c
                    break
            
            # Draw task rectangle
            rect = mpatches.Rectangle(
                (start, y - 0.4),
                duration,
                0.8,
                facecolor=color,
                edgecolor='black',
                linewidth=1.5,
                alpha=0.8
            )
            ax.add_patch(rect)
            
            # Add task label
            ax.text(
                start + duration/2,
                y,
                task_id.replace("_", "\n"),
                ha='center',
                va='center',
                fontsize=8,
                fontweight='bold'
            )

            # track maximum end time in case makespan is None
            try:
                end_time = float(start) + float(duration)
                _computed_max_end = max(_computed_max_end, end_time)
            except Exception:
                pass
    
    # Styling
    if makespan is None:
        makespan_val = _computed_max_end
    else:
        try:
            makespan_val = float(makespan)
        except Exception:
            makespan_val = _computed_max_end

    ax.set_xlim(0, makespan_val + 5)
    ax.set_ylim(-0.5, len(robot_names) - 0.5)
    ax.set_yticks(range(len(robot_names)))
    ax.set_yticklabels(robot_names)
    ax.set_xlabel('Time (s)', fontsize=12, fontweight='bold')
    ax.set_ylabel('Robot', fontsize=12, fontweight='bold')
    ax.set_title('Heterogeneous Task Allocation - Gantt Chart', 
                 fontsize=14, fontweight='bold')
    ax.grid(True, axis='x', alpha=0.3, linestyle='--')

    # Add makespan line
    ax.axvline(makespan_val, color='red', linestyle='--', linewidth=2,
               label=f'Makespan: {makespan_val:.1f}s' if makespan_val is not None else 'Makespan: N/A')
    ax.legend(loc='upper right')
    
    plt.tight_layout()
    plt.savefig(output_path, dpi=300, bbox_inches='tight')
    print(f"[VIS] Saved Gantt chart to {output_path}")
    plt.close()


def plot_task_assignment_matrix(schedule_data, output_path):
    """Plot task assignment matrix"""
    
    # Support two schema names: older 'schedule' or newer 'schedules'
    schedule = schedule_data.get("schedules", schedule_data.get("schedule", {}))

    # task_pool may not be present in the exported schedule.json; build if missing
    if "task_pool" in schedule_data:
        task_pool = schedule_data["task_pool"]
    else:
        # derive task ids from schedule entries
        task_ids_set = set()
        for tasks in schedule.values():
            for t in tasks:
                task_ids_set.add(t.get("task_id"))
        task_pool = {tid: {} for tid in sorted([tid for tid in task_ids_set if tid is not None])}

    robots_data = schedule_data.get("robots", {})
    
    fig, ax = plt.subplots(figsize=(10, 6))
    
    robot_names = list(schedule.keys())
    task_ids = list(task_pool.keys())
    
    # Create assignment matrix
    matrix = []
    for task_id in task_ids:
        row = []
        for robot_name in robot_names:
            assigned = any(t["task_id"] == task_id for t in schedule[robot_name])
            row.append(1 if assigned else 0)
        matrix.append(row)
    
    # Plot heatmap
    im = ax.imshow(matrix, cmap='YlGn', aspect='auto')
    
    # Set ticks
    ax.set_xticks(range(len(robot_names)))
    ax.set_yticks(range(len(task_ids)))
    ax.set_xticklabels(robot_names)
    ax.set_yticklabels(task_ids)
    
    # Add text annotations
    for i, task_id in enumerate(task_ids):
        for j, robot_name in enumerate(robot_names):
            text = "✓" if matrix[i][j] == 1 else ""
            ax.text(j, i, text, ha="center", va="center",
                    color="black", fontsize=16, fontweight='bold')
    
    ax.set_xlabel('Robot', fontsize=12, fontweight='bold')
    ax.set_ylabel('Task', fontsize=12, fontweight='bold')
    ax.set_title('Task Assignment Matrix', fontsize=14, fontweight='bold')

    plt.tight_layout()
    plt.savefig(output_path, dpi=300, bbox_inches='tight')
    print(f"[VIS] Saved assignment matrix to {output_path}")
    plt.close()
def main():
    parser = argparse.ArgumentParser(description="Visualize heterogeneous schedule JSON")
    parser.add_argument("path", nargs="?", help="Path to schedule JSON")
    parser.add_argument("--latest", action="store_true", help="Use latest run's schedule JSON in runs/")
    args = parser.parse_args()

    if args.latest:
        # find newest schedule file in runs/
        # prefer schedule_heterogeneous.json, fallback to schedule.json
        run_dirs = sorted(glob.glob("runs/*/"), reverse=True)
        json_path = None
        for rd in run_dirs:
            cand1 = Path(rd) / "schedule_heterogeneous.json"
            cand2 = Path(rd) / "schedule.json"
            if cand1.exists():
                json_path = cand1
                break
            if cand2.exists():
                json_path = cand2
                break
        if json_path is None:
            print("No schedule JSON found under runs/ (tried schedule_heterogeneous.json and schedule.json)")
            sys.exit(1)
    else:
        if not args.path:
            parser.print_usage()
            sys.exit(1)
        json_path = Path(args.path)

    if not json_path.exists():
        print(f"Error: {json_path} not found")
        sys.exit(1)

    print(f"[LOAD] Loading schedule from {json_path}")
    schedule_data = load_schedule(json_path)

    output_dir = json_path.parent

    # Generate visualizations
    plot_gantt_chart(
        schedule_data,
        output_dir / "gantt_heterogeneous.png"
    )

    plot_task_assignment_matrix(
        schedule_data,
        output_dir / "assignment_matrix.png"
    )

    print("\n[DONE] All visualizations generated!")


if __name__ == "__main__":
    main()