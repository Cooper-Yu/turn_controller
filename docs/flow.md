# Turn control flow

1. Validate configuration and wait for fresh, stopped odom.
2. Generate all cumulative heading waypoints once from the initial pose; activate the first.
3. Compare that fixed target with accepted odom yaw; PID commands angular.z only.
4. Within angular tolerance: zero command, qualify stopped feedback, then dwell.
5. If disturbed, restart qualification; otherwise advance or stop and exit.
6. Feedback, time, or segment deadline failure: zero command and exit code 2.

```mermaid
flowchart TD
  A[Fresh stopped odom] --> B[Generate fixed waypoint list]
  B --> C[Feedback and time guards]
  C --> D{Heading within tolerance?}
  D -- No --> E[Angular PID and limits]
  E --> C
  D -- Yes --> F[Stop, settle, dwell]
  F -- Drift --> C
  F -- Complete --> G{More turns?}
  G -- Yes --> J[Select next stored waypoint]
  J --> C
  G -- No --> H[Stop and exit zero]
  C -- Fault --> I[Stop and exit two]
```

The waypoint list is written only during initialization. begin_turn() selects a stored
target. on_odom() updates feedback; tick()
selects tracking or completion. No distance controller runs concurrently.

## Optional preparation sequence

```mermaid
flowchart LR
  A[Wall measurement] --> B[Right-wall heading alignment]
  B --> C[Heading hold plus side and rear positioning]
  C --> D[Stopped WAITING]
  D --> E[Finish accepted and preparation process exited]
  E --> F[Record adjusted origin and run angular waypoints]
  A --> G[Fault: stop, no turn launch]
  B --> G
  C --> G
```

See start_preparation.md. The coordinator never resumes Task2's previewed AB route.

## Task4 lifecycle

```mermaid
flowchart LR
  E[Scene 2 entry] --> Q{Skip preparation?}
  Q -- no --> P[Task2 wall preparation]
  P --> F[Finish and reap preparation]
  F --> O[Fresh stopped odom at current pose]
  Q -- yes --> O
  O --> W[Freeze three cumulative yaw targets]
  W --> T[Shared angular executor]
  T --> Z[Stop and exit]
```
