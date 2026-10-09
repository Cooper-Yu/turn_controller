## Task4 scene selection

The default `prepare_and_turn` remains the simulation profile. The public
`turn_controller 2` entry replaces itself with `prepare_and_turn --scene 2`
before creating ROS interfaces. Scene 2 uses real time, /scan_filtered, and
Task2's 30-degree wall half-window; scene 1 retains its local 20-degree profile.
The coordinator invokes `turn_controller 2 --skip-preparation` after clean
handoff, preventing recursion. At a later point, the same skip entry never
moves the robot back to the task-start placement.

# Optional wall-based start preparation

`turn_controller` remains an angular-only Task3 executable. A separate simulation
entry composes the existing Task2 preparation with it:

```bash
ros2 run turn_controller prepare_and_turn
```

It needs the installed `distance_controller` package (tested against b189651), its
ExecuteStep service and fresh odom, laser and laser-to-base TF. Both nodes use
simulation time. No hardware deployment is included in this entry.

## Ordered ownership

1. Confirm no other /cmd_vel publisher is discovered. Stop teleoperation and other
   controllers before using this sequence; graph discovery is not an exclusive lock.
2. Start distance_controller scene 2 as task3_preparation with manual_mode and
   start_paused. It estimates the right-wall direction, aligns, then maintains that
   heading while centering laterally and adjusting the rear distance.
3. Its inherited limits remain: valid/stable wall measurements, fresh odom/scan,
   bounded stages, 0.20 m maximum preparation displacement, stopped qualification.
   This is local correction near the intended start, not navigation from anywhere.
4. Poll its private status service. WAITING means preparation completed. Request
   finish, which separately checks fresh/stopped/aligned feedback. Never send resume.
5. Wait for the preparation process to exit successfully, then start turn_controller.
   It rechecks fresh stopped odom and generates all angular waypoints from this
   adjusted pose. No two child controllers run simultaneously.
6. A preparation fault, deadline or unexpected exit prevents launching the turn
   controller. Cleanup terminates owned children and publishes zero when the ROS
   context is available. Base timeout braking is still needed if transport fails.

The preparation node may print an AB route preview and the legacy phrase "starting
route". start_paused prevents executing AB; only status and finish are requested.
The route remains unexecuted. Task2 source and its tag are unchanged.

## Distance and frame meaning

Default rear target is 0.28 m **from base_link origin**, using laser points transformed
through TF. It is not 28 cm of clearance behind the bumper. Centering accepts half the
left-minus-right distance within 0.01 m; therefore left/right may differ by up to
0.02 m. Do not infer yaw=0 in odom from parallel walls: the prepared measured yaw
becomes the subsequent turn origin.

These three walls must be visible and correspond to the intended corridor. An
opening or unrelated object is not a valid replacement wall. Preparation does not
certify the full swept footprint for every robot or arbitrary placement.

## Local and cloud commands

The local maze exposes the tested raw scan topic:

```bash
ros2 run turn_controller prepare_and_turn --scan-topic /scan
```

This simulation entry defaults to /scan, matching the verified local maze. The earlier
Task2 setup used /scan_filtered; if that is the intended topic in the cloud simulation,
select --scan-topic /scan_filtered explicitly. Confirm the actual topic and TF before
running. There is no silent topic fallback.

To inspect just the prepared pose without starting turns:

```bash
ros2 run turn_controller prepare_and_turn --prepare-only
ros2 topic echo /odometry/filtered --once --field pose.pose
```

`--rear-distance` sets a positive finite base-origin distance (inherited controller
validation still applies). `--timeout` limits the coordinator's preparation wait,
default 75 steady seconds; it does not override the controller's 60-second preparation
budget. Placement requiring more than the inherited travel limit must be corrected
manually after checking geometry, not by blindly increasing limits.

Build both sourced packages with `colcon build --packages-up-to turn_controller`.
The official angular-only command is still `ros2 run turn_controller turn_controller`.
This optional position-adjustment sequence is separate from the official Task3
angular-only scoring run. Cloud acceptance and task3 tagging remain pending.

## Placement boundary observed locally

The stock maze spawn measured roughly 0.221 m rear distance, very close to the inherited
0.22 m minimum. A yaw/lateral perturbation at that rear position caused SCAN_INVALID
during alignment and correctly prevented turn startup. Do not lower the guard to
force it through. Place slightly farther from the rear wall while keeping both side
walls visible, then restart. Right-wall fitting only accepts headings within 30 degrees
of the forward axis. A wider initial offset is outside this preparation contract.

## Simulation profile and local checks

This wrapper overrides only the right-wall fitting window to +/-20 degrees around
body-right, while retaining the inherited 0.18 m minimum span, RMS/stability and
minimum-distance guards. At an offset start, the larger Task2 window returned an
inconsistent approximately -47-degree line and was rejected. The narrower window
measured approximately -9 degrees and completed alignment. This is an observed local
profile choice, not proof that a narrow window always identifies the intended wall.
The cloud simulation must still meet the span/quality checks.

Local evidence includes default placement, a bounded roughly 9-degree yaw/position
perturbation with rear margin, and missing-scan refusal. A near-rear-limit perturbation
was rejected and is retained as a placement-boundary test. Unit checks cover successful
finish/reap ordering, fault and premature-exit refusal, bounded child cleanup and
leaving an already-active publisher untouched. No independent contact sensor was used;
odom and command checks do not certify collision-free motion in arbitrary placements.
## Verified local evidence (2026-10-09)

The final 20-degree half-window profile passed both the default maze start and a
perturbed start (about 10 degrees of yaw plus lateral/forward displacement).
[Nominal summary](evidence/start_preparation/nominal/prepared_summary.json) and
[perturbed summary](evidence/start_preparation/perturbed/prepared_summary.json)
record four completed turns, zero linear commands during the turn stage, final
zero velocity, and exit status 0. Maximum final heading errors were 0.008367 and
0.008598 rad; maximum turn-stage odometry displacement was below 0.000020 m.
These measurements do not independently certify wall clearance or contact.

The rear-boundary and wider-window rejected runs are preserved alongside the
successful evidence. Missing scan input exited with status 2 without starting
turning. Five coordinator unit tests and ten C++ tests passed; Ruff, build,
Doxygen warning checks and Git whitespace checks passed. Cloud verification
remains pending; no task3 tag is created by this change.

