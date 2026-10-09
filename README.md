## Task4 real-robot entry

```bash
ros2 run turn_controller turn_controller 2
```

Scene 2 first reuses Task2 wall preparation (right-wall heading, centering and
0.28 m rear distance from base_link), exits that process, then turns -30, -30,
+60 degrees using fixed targets relative to the prepared heading. These angles
are initial estimates, not measured logo orientations. Scene 2 defaults to real
time and /scan_filtered; the Task2 dependency must be built and sourced.
The previous Task3 tag is unchanged. Without a scene argument, Task3 still runs
its four simulation turns without wall preparation.

At a later waypoint D, after the translation controller has stopped and exited:

```bash
ros2 run turn_controller turn_controller 2 --skip-preparation
```

This skips only wall placement. Fresh stopped odometry is still required; the
current pose defines the new turn group's origin. For a single relative turn:

```bash
ros2 run turn_controller turn_controller 2 --skip-preparation --ros-args \
  -p 'turn_angles:=[-0.5235987755982988]'
```

`turn_angles` contains signed relative radians. Each step is independently
editable route data, executed by one shared angular controller; no duplicated
1-to-2/2-to-3/3-to-1 PID functions are needed. A later route coordinator must stop
and reap the translation controller before starting this process; an automatic
AB-D-turn route dispatcher is not part of this change.

Both prepared and direct entry accept ROS overrides. Prepared entry forwards
them to both stages, so clock and topic overrides must suit both controllers.
For isolated local simulation testing of scene 2, use `use_sim_time:=true` and
`scan_topic:=/scan`. Do not use the simulation-only prepare_and_turn default for
real hardware. Initial clearance must accommodate the complete body rotation;
wall centering alone does not certify that clearance.

# turn_controller

Task3 in-place relative yaw PID for ROS2 Humble. Implemented with Coach support;
local verification is distinct from independent learner or official acceptance.

## Behavior

Wait for fresh stopped odometry, record the initial heading, then execute four
relative turns: right 45 degrees, left 45 degrees, left 45 degrees, right 45 degrees.
These angles follow the user's description of the animation, not a measurement of it.
After initialization, freeze all targets: `target[i] = initial_yaw + sum(angle[0..i])`.
Later measured stopping errors do not shift the remaining targets. Every waypoint
retains the initial x/y for inspection; those coordinates are not translation commands.
Accumulate odom yaw increments across +/-pi so the requested signed arc is preserved.
All linear command components remain zero. Angular PID uses P=1.8, I=0.03, D=0.35;
the derivative uses measured angular velocity to avoid target-step derivative kick.
Speed is limited to 0.6 rad/s and output slew to 0.6 rad/s squared. Conditional
integration prevents error accumulation into speed saturation.

Within 0.01 rad of the target, command zero; require angular speed below 0.02 rad/s
and planar speed below 0.01 m/s continuously for 0.4 s, then dwell for 0.5 s of ROS time.
Drift out of tolerance restarts qualification. Complete all turns, stop, exit zero.
Every segment has a 30 s steady-clock deadline. Startup has a 15 s deadline.
Invalid/stale odom does not refresh feedback age. A 0.5 s odom gap, frame change,
yaw sample jump over 0.35 rad, or ROS time reversal/large jump stops and exits 2.
This initial Task3 version does not include Task2's automatic feedback recovery.
Base command-timeout braking is required for transport failure. Simulation only.

## Build and run

```bash
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select turn_controller
source install/setup.bash
```

In the course simulation, start the maze in another sourced terminal:

```bash
ros2 launch rosbot_xl_gazebo simulation.launch.py
```

Then:

```bash
ros2 run turn_controller turn_controller
```

Defaults: `use_sim_time=true`, `odom_topic=/odometry/filtered`,
`cmd_vel_topic=/cmd_vel`. Local WSL uses the existing course-maze adapter:
`ros2 launch husarion_office_gz cp18_local.launch.py scene:=maze headless:=True`.
Source its existing support overlays before use; see the authoritative training
verification record for exact environment and commands. No hardware operation is included.

Override relative angles in radians if needed:

```bash
ros2 run turn_controller turn_controller --ros-args \
  -p 'turn_angles:=[-0.7853981634, 0.7853981634, 0.7853981634, -0.7853981634]'
```

## Reading and verification

- `src/main.cpp`: process entry and result code.
- `include/turn_controller/turn_controller.hpp`: node declarations and interface documentation.
- `src/turn_controller.cpp`: initialization, guards and the shared turn executor.
- `include/turn_controller/turn_route.hpp`: turn factories and fixed waypoint generation.
- `include/turn_controller/turn_math.hpp`: pure angle math and angular PID.
- `test/test_turn_math.cpp`: boundary, damping, saturation and reset checks.
- `test/verify_turn_node.py`: isolated ROS feedback/failure tests.
- `docs/flow.md`: state/data flow; Doxygen renders class and member documentation.

Run `colcon test --packages-select turn_controller`, then `colcon test-result --verbose`.
Generate documentation from this package directory with `doxygen docs/Doxyfile`.
The package has a separate local Git repository. Task2 remains unchanged;
no Task3 release tag is created before completing the acceptance work.

## Compose turns without duplicating the controller

The course stores angles in radians. `turn_by_radians()` is the canonical factory;
`turn_by_degrees()` is an explicit convenience wrapper. Both return a `TurnStep`
without publishing commands or blocking execution. Positive turns are left,
negative turns are right. Each step accepts up to one full signed revolution.

Edit `default_turn_route()` in `turn_route.hpp` to add or reorder steps:

```cpp
return {
  turn_by_degrees(-45.0),
  turn_by_degrees(45.0),
  turn_by_radians(0.7853981633974483),
  turn_by_degrees(-45.0),
};
```

One step, three steps, or a longer route use the same `execute_current_turn()`.
The `turn_angles` ROS array overrides these defaults and always uses radians.
Speed, gains, tolerances, dwell and timeout remain shared ROS parameters in this
version. Per-step policy overrides and live interactive turn commands are not added.

Once fresh stopped feedback establishes the origin, `build_turn_waypoints()`
produces the full ordered pose list. For initial yaw=30 degrees and the default
route, the planned headings are -15, 30, 75 and 30 degrees. The initialization
log prints x/y/yaw and each signed delta. `begin_turn()` selects that stored
heading; it never recomputes the route from a preceding stopping error.
Unwrapped headings preserve requested rotation direction across +/-pi.

`tick()` dispatches `handle_feedback_guard()`, `handle_initialization()`, then
`execute_current_turn()`. The executor reuses the existing PID and
`handle_completion()` for stopped qualification and dwell. Header/source separation
changes organization; the existing feedback and time failure policy remains intact.

## Local verification and cloud handoff

See [Task3 acceptance](docs/task3_acceptance.md) for rubric weights, measured results,
limitations and cloud instructions. Ten GoogleTests and ten isolated node/configuration
cases pass. The local maze completed all four default turns and exited zero.
Damping was compared at D=0.25/0.35/0.4; the selected default is D=0.35.

Startup requires accepted stopped odom stamps spanning at least 0.3 seconds; one
stopped sample cannot initialize the route. Any moving feedback sample resets the
stopped qualification. A late callback cannot hide a receipt timeout by refreshing it.

[Test entry points](test/README.md) document portable tests and telemetry capture.
[Tooling](docs/tooling.md) distinguishes executed checks from deferred GUI tools.
Cloud verification remains pending. Do not create or move a task3 tag until the
user confirms the cloud run. Task2's repository and tag remain unchanged.

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

## Optional start-position preparation

Use [wall-based start preparation](docs/start_preparation.md) to align with the right wall,
center between side walls and adjust rear distance before the angular-only program.
The separate prepare_and_turn executable delegates to the installed Task2 controller,
waits for stopped readiness and a successful process exit, then starts turns.
The ordinary Task3 command and Task2 source/tag are unchanged.
