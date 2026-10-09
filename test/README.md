# Test entry points

Build and source the workspace first. Run pure tests from the workspace:

```bash
colcon test --packages-select turn_controller --event-handlers console_direct+
colcon test-result --test-result-base build/turn_controller --verbose
```

Ten GoogleTests cover yaw wrapping, PID sign/limits/damping/reset/anti-windup,
angle units, cumulative targets, long signed arcs and rejected route inputs.

From this package, run the isolated node fixture (it sets local ROS domain 174):

```bash
python3 test/verify_turn_node.py
```

Ten cases cover wraparound completion, lost/stale/missing feedback, a yaw jump,
changed frames, moving startup, a single stopped sample, stalled motion and invalid
gains. Logs go to `/tmp/turn_controller_tests`; override with `TURN_TEST_OUTPUT`.
The executable is resolved through the sourced ament index, not a machine-specific path.

For a running course maze, collect telemetry in another sourced terminal:

```bash
mkdir -p /tmp/task3_run
python3 tools/capture_turn.py /tmp/task3_run/telemetry.csv
```

In the controller terminal, preserve its actual exit code:

```bash
set -o pipefail
ros2 run turn_controller turn_controller 2>&1 | tee /tmp/task3_run/controller.log
printf '%s\n' "${PIPESTATUS[0]}" > /tmp/task3_run/exit_code.txt
```

Wait one second after completion, interrupt only the collector, then run:

```bash
python3 tools/summarize_maze.py /tmp/task3_run
```

The summary uses both ROS sample time and wall receipt time to correlate stages.
It assumes the default four short turns without crossing the wrapped yaw boundary;
long-arc correctness is covered separately by pure/isolated tests. Sampled dwell
coverage can be slightly shorter than the full two-second log interval. These tools
observe odometry and commands, not independent collision or true-heading ground truth.
They do not start a simulator or certify hardware/official grading.
