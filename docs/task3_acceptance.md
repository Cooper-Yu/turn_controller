# Task3 acceptance and cloud handoff

## Scope and official rubric

Source: Checkpoint 18, Part 2, Task 3, the learner's local course outline.
This is a Coach assessment against its 2.5-point rubric, not an official course grade.
The required package/class, radians-based predefined route (four steps, at least three),
odometry feedback, all PID terms, angular-only commands and maze run are implemented.

| Criterion | Weight | Conservative local credit | Evidence / remaining gate |
| --- | ---: | ---: | --- |
| Three clearly defined PID gains | 0.375 | 0.375 | P=1.8, I=0.03, D=0.35, all participate in AngularPid |
| High-speed, consistent controlled turns | 0.750 | Pending | 0.6 rad/s command cap reached; cloud dynamics and qualitative speed judgement remain |
| Stop for a few seconds at each waypoint | 0.500 | 0.500 | Four two-second dwells after 0.4-second stopped qualification |
| Complete stop after final waypoint | 0.250 | 0.250 | Final command zero, measured yaw rate 0.000910 rad/s |
| Precise heading without over/undershoot | 0.375 | Pending | No sampled target crossing; max endpoint error 0.009021 rad; true-angle/cloud assessment remains |
| Program terminates | 0.250 | 0.250 | All four reached, exit code 0 |

**Conservative locally substantiated credit: 1.375 / 2.5.** The other 1.125 points
are pending assessment, not observed failures. Local endpoint tolerance is 0.01 rad;
this is a configured engineering criterion, not an invented official grading tolerance.
The supported local behavior is ready for cloud testing; do not claim official PASS
or independent learner implementation from generated code and local tests.

## Final default-command evidence (2026-10-09)

WSL Ubuntu-22.04 / ROS2 Humble, local course-maze adapter, isolated ROS domain 173.
The adapter is not proof of identical cloud plugins or dynamics. Command:
`ros2 run turn_controller turn_controller` (no parameter overrides).

| Turn | Relative angle | Final target-minus-actual error | Sampled overshoot |
| --- | ---: | ---: | ---: |
| 1 | -45 degrees | -0.008193 rad | 0 rad |
| 2 | +45 degrees | +0.008820 rad | 0 rad |
| 3 | +45 degrees | +0.009021 rad | 0 rad |
| 4 | -45 degrees | -0.007597 rad | 0 rad |

Each dwell log interval is about 2.000 seconds; the interior odom samples span
1.96 seconds at the observed sample spacing. All commanded linear velocities are
zero, maximum angular command is 0.6 rad/s, final command is zero, and exit code is 0.
No independent collision or absolute-heading ground truth was collected. Tiny odom
x/y displacement is model evidence, not proof of physical zero translation.

See `evidence/task3_default/controller.log`, `telemetry.csv`, `summary.json`,
and `node_results.json`. Reproduce the summary with:

```bash
python3 tools/summarize_maze.py docs/evidence/task3_default
```

Ten GoogleTests and ten isolated node/configuration scenarios passed. clang-format,
LLVM23 clang-tidy, Ruff and Doxygen checks passed. Tool deferrals are in tooling.md.

Damping comparison kept P/I, speed, slew and tolerances fixed: D=0.25 produced peak
sampled overshoot 0.013649 rad; D=0.4 removed sampled crossing but required an extra
stopped qualification on the fourth turn. D=0.35 removed sampled crossing in the
comparison and subsequent default run, with somewhat slower arrival. These bounded
runs justify the local default, not universal optimality. Both comparison summaries
are preserved alongside the final evidence.

## Cloud test

For the first checkout:

```bash
cd ~/ros2_ws/src
git clone https://github.com/Cooper-Yu/turn_controller.git
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select turn_controller
source install/setup.bash
```

If already cloned, inspect `git status` and use `git pull --ff-only` in that package
instead of cloning again. Preserve any local edits.

In a separate sourced terminal, use the official maze launch:

```bash
ros2 launch rosbot_xl_gazebo simulation.launch.py
```

Run with no parameter overrides:

```bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
ros2 run turn_controller turn_controller
echo "exit_code=$?"
```

Confirm initialization and four printed fixed waypoints, right/left/left/right turns,
no translation commands, stable stop and dwell at every target, final stop and exit 0.
Use test/README.md to capture transient motion if tuning or scoring is uncertain.
If feedback or a clock/segment guard fails, inspect its log before relaxing thresholds.
Run no other velocity-producing controller concurrently.

**Cloud acceptance, official grading, independent learner verification and the
`task3` tag are pending. Create the tag only after the user's successful cloud test.**
Task2's code and tag have not been changed by this delivery.

## Low-rate simulation clock repair (2026-10-09)

The first cloud run of dd48da2 initialized but timed out on turn 1 after 30 seconds;
the user observed almost no movement or intermittent turning. This is a failed cloud
acceptance, not a completed Task3. The exact cloud clock frequency was not measured.

A local reproduction at 5 Hz /clock with a 50 Hz wall timer exposed a defect:
repeated ROS timestamps caused stop() to publish zero and reset the PID slew state.
The old node could not complete the two-turn fixture within its 18-second bound.
Repeated timestamps now skip the control update without resetting or publishing;
the steady-clock feedback watchdog still runs first. A paused clock with frozen
feedback still stops and exits 2; no timeout or PID gain was relaxed.

The same 5 Hz fixture now finishes and exits 0. Ten pure tests and twelve distinct
node/configuration cases passed (the prior ten plus slow-clock completion and clock
pause timeout). The full maze regression completes four turns, max endpoint error
0.009767 rad, zero sampled crossing, final zero command and exit 0. Evidence is in
evidence/clock_repair; the full CSV remains in the local engineering record.
Turn-progress logs now show yaw, target, error, measured/commanded angular velocity
and dt once per advancing ROS second. Confirm the repair in the cloud before tagging.

## Optional prepared start

[Wall-based start preparation](start_preparation.md) is a separately verified
local convenience entry. It does not change the official angular-only command,
Task2 tag, cloud acceptance status, or the conservative score above.
