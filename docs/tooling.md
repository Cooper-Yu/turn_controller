# Tooling

Use the installed WSL Ubuntu-22.04 / Humble tools. clang-format uses the local
100-column profile; clang-tidy reviews naming and functions over 60 lines rather
than mechanically splitting them. Ruff applies to Python fixtures. Doxygen checks
interface documentation. Build/test through colcon; VS Code configurations provide
format, analysis, task and GDB entry points. Tool configuration alone is not test evidence.

The canonical template was applied during package creation. The existing workspace
settings were preserved; package-local settings and rules remain available.
The authoritative Checkpoint18 plan records deferred tools and acceptance gates.

## Delivery evidence and deferred tools

- README, interface comments and Doxygen: English project documentation; source and
  generated-warning checks. Canvas links to actual project entries; it is navigation.
- Mermaid: the control-flow source is in flow.md (GitHub Markdown renders it).
  Doxygen HTML rendering of Mermaid is not claimed.
- colcon / GoogleTest: actual build and ten pure tests; isolated ROS fixture adds ten
  end-to-end/configuration cases. It is a standalone Python assertion harness, not pytest.
- ROS2 topic and logs: the local maze publisher/subscriber types and controller output
  were captured. CSV telemetry and summarize_maze.py measure angular motion and dwell.
- clang-format, LLVM23 clang-tidy and Ruff: used on affected C++ and Python files.
  IgnoreMacros needs a compatible checker; LLVM14 alone is not equivalent.
- VS Code Testing and GDB: launch/task entries exist; GUI interaction and breakpoints
  have not been validated. Use them if a cloud-only state transition needs diagnosis.
- rqt_graph: deferred until graph/remapping ambiguity; endpoint listings suffice here.
- rosbag2 / PlotJuggler: deferred until cloud timing or dynamics differ; current bounded
  CSV capture resolves this local damping comparison without adding a second recorder.
- launch_testing: deferred; the existing bounded subprocess fixture covers this release's
  node lifecycle. gcov: no coverage percentage claimed; add when assessing uncovered paths.
- Git: separate package repository and reviewed initial delivery. Cloud acceptance and
  the task3 tag are separate gates; do not create the tag from local test results alone.

Do not copy local dynamics gains blindly into a generic package template. Reusable
policy: use named units, freeze planned goals, qualify fresh stopped feedback, measure
transient error as well as final error, and document each environment's evidence.
