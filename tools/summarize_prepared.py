"""Check preparation handoff and angular-only tracking from one recorded sequence."""

import csv
import json
import math
from pathlib import Path
import re
import sys

root = Path(sys.argv[1])
log = (root / 'controller.log').read_text()
rows = list(csv.DictReader((root / 'telemetry.csv').open()))
handoff = float(
    re.search(r'\[([\d.]+)\] \[task3_sequence\]: Preparation process exited;', log).group(1)
)
origin = re.search(r'\[turn_controller\]: Initialization complete: x=([-\d.]+) y=([-\d.]+)', log)
x0, y0 = map(float, origin.groups())
commands = [r for r in rows if r['kind'] == 'cmd' and float(r['wall_time']) >= handoff]
odom = [r for r in rows if r['kind'] == 'odom' and float(r['wall_time']) >= handoff]
errors = [float(e) for e in re.findall(r'Reached turn \d+: .*error=([-\d.]+) rad', log)]
assert log.index('Preparation accepted finish: state=WAITING') < log.index(
    'Preparation process exited;'
)
assert log.index('Preparation process exited;') < log.index('[turn_controller]: Task3:')
assert len(errors) == 4 and max(map(abs, errors)) <= 0.01
assert all(float(r['vx']) == 0 and float(r['vy']) == 0 for r in commands)
assert float(commands[-1]['wz']) == 0
assert int((root / 'exit_code.txt').read_text()) == 0
summary = {
    'sequential_handoff': True,
    'turn_origin_xy': [x0, y0],
    'turn_errors_rad': errors,
    'turn_linear_commands_zero': True,
    'turn_max_xy_drift_m': max(math.hypot(float(r['x']) - x0, float(r['y']) - y0) for r in odom),
    'final_zero_command': True,
    'exit_code': 0,
    'limits': 'Odom/cmd evidence only; no independent collision measurement.',
}
(root / 'prepared_summary.json').write_text(json.dumps(summary, indent=2) + '\n')
print(json.dumps(summary, indent=2))
