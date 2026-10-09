"""Summarize a bounded maze run from controller.log and timestamped telemetry.csv."""

import csv
import json
import math
from pathlib import Path
import re
import sys

root = Path(sys.argv[1])
rows = list(csv.DictReader((root / 'telemetry.csv').open()))
odom = [{k: float(v) for k, v in r.items() if k != 'kind'} for r in rows if r['kind'] == 'odom']
cmd = [{k: float(v) for k, v in r.items() if k != 'kind'} for r in rows if r['kind'] == 'cmd']
log = (root / 'controller.log').read_text()
starts = re.findall(
    r'\[INFO\] \[([\d.]+)\].*Turn (\d+)/\d+: start=([-\d.]+) delta=([-\d.]+) target=([-\d.]+)', log
)
ends = re.findall(
    r'\[INFO\] \[([\d.]+)\].*Reached turn (\d+): target=([-\d.]+) actual=([-\d.]+) error=([-\d.]+)',
    log,
)
settles = re.findall(r'\[INFO\] \[([\d.]+)\].*Turn (\d+) settled:', log)
settles = list({item[1]: item for item in settles}.values())
assert len(starts) == len(ends) == len(settles) == 4
metrics = []
for start, end, settle in zip(starts, ends, settles):
    t0, t1, ts = float(start[0]), float(end[0]), float(settle[0])
    target, direction = float(start[4]), math.copysign(1, float(start[3]))
    samples = [r for r in odom if t0 <= r['wall_time'] <= t1]
    stopped = [r for r in samples if r['wall_time'] >= ts]
    overshoot = max(0.0, max(direction * (r['yaw'] - target) for r in samples))
    metrics.append(
        {
            'turn': int(start[1]),
            'target_rad': target,
            'final_error_rad': float(end[4]),
            'peak_overshoot_rad': overshoot,
            'dwell_wall_seconds': t1 - ts,
            'dwell_sampled_ros_seconds': stopped[-1]['time'] - stopped[0]['time'],
            'dwell_max_measured_wz': max(abs(r['wz']) for r in stopped),
        }
    )
assert all(abs(m['final_error_rad']) <= 0.01 for m in metrics)
assert all(m['dwell_sampled_ros_seconds'] >= 1.85 for m in metrics)
assert all(r['vx'] == 0 and r['vy'] == 0 for r in cmd)
assert cmd[-1]['wz'] == 0
assert int((root / 'exit_code.txt').read_text()) == 0
summary = {
    'turns': metrics,
    'max_command_rad_s': max(abs(r['wz']) for r in cmd),
    'final_measured_rad_s': odom[-1]['wz'],
    'max_xy_displacement_m': max(
        math.hypot(r['x'] - odom[0]['x'], r['y'] - odom[0]['y']) for r in odom
    ),
    'linear_commands_zero': True,
    'final_command_zero': True,
    'exit_code': 0,
    'limits': 'Odometry and command evidence only; no independent collision or absolute heading ground truth.',
}
(root / 'summary.json').write_text(json.dumps(summary, indent=2) + '\n')
print(json.dumps(summary, indent=2))
