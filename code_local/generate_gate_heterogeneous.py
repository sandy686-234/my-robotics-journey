#!/usr/bin/env python3
import json
import glob
from pathlib import Path

# Find newest schedule_heterogeneous.json or schedule.json
run_dirs = sorted(glob.glob('runs/*/'), reverse=True)
json_path = None
for rd in run_dirs:
    cand = Path(rd) / 'schedule_heterogeneous.json'
    if cand.exists():
        json_path = cand
        break
    cand2 = Path(rd) / 'schedule.json'
    if cand2.exists():
        json_path = cand2
        break

if json_path is None:
    print('No schedule JSON found in runs/; run scheduler first')
    raise SystemExit(1)

print('Using schedule:', json_path)
data = json.loads(json_path.read_text())

# Support both shapes
if 'schedule' in data:
    sched = data['schedule']
else:
    sched = data.get('schedules', {})

task_pool = data.get('task_pool', {})
robots = data.get('robots', {})
# robots here may map names->info; ensure mapping name->id exists

# build mapping from robot name to id
robot_name_to_id = {}
for name, info in robots.items():
    rid = info.get('id') if isinstance(info, dict) else None
    if rid is not None:
        robot_name_to_id[name] = rid

# If robots mapping is empty, try to infer from schedule keys using single-letter ids
if not robot_name_to_id:
    # attempt to map Forklift->A, Courier->B etc by checking schedule keys
    # fallback: use keys as-is
    for i, name in enumerate(sched.keys()):
        robot_name_to_id[name] = name

# Build gate-style schedules keyed by robot id
gate_schedules = {}
for robot_name, tasks in sched.items():
    rid = robot_name_to_id.get(robot_name, robot_name)
    out_tasks = []
    for t in tasks:
        task_id = t.get('task_id')
        start = t.get('start_time')
        duration = t.get('duration')
        end = t.get('end_time', (start + duration) if start is not None and duration is not None else None)
        goal = None
        deadline = None
        if task_pool and task_id in task_pool:
            goal = task_pool[task_id].get('location')
            deadline = task_pool[task_id].get('deadline')
        # also check task entry for location
        if not goal and 'location' in t:
            goal = t.get('location')
        out_tasks.append({
            'task_id': task_id,
            'start_time': start,
            'duration': duration,
            'end_time': end,
            'goal': goal,
            'deadline': deadline,
        })
    gate_schedules[rid] = out_tasks

out = {
    't0_mono': data.get('timestamp', 0),
    'makespan': data.get('makespan'),
    'schedules': gate_schedules,
    'resources': data.get('resources', {})
}

out_path = json_path.parent / 'gate_heterogeneous.json'
out_path.write_text(json.dumps(out, indent=2))
print('Wrote', out_path)
