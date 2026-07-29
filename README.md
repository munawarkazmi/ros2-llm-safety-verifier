# ROS2 LLM Safety Verifier

**A real-time safety layer for Nav2: catching LLM-hallucinated trajectories before they reach robot hardware**

[![CI](https://github.com/munawarkazmi/ros2-llm-safety-verifier/actions/workflows/ci.yml/badge.svg)](https://github.com/munawarkazmi/ros2-llm-safety-verifier/actions/workflows/ci.yml)
[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](LICENSE)

Large language models are increasingly asked to produce navigation goals and
trajectories. Sometimes they hallucinate: a goal inside a wall, a path through
a person, a waypoint that never existed. The idea of this project is a
deterministic verifier between the LLM and Nav2 - costmap collision checks,
kinematic feasibility, workspace bounds - so unsafe commands are intercepted
before a wheel turns.

> **Retraction note (July 2026).** Earlier versions of this README reported
> hardware-validated results (94% of unsafe trajectories caught, 103
> TurtleBot3 trials) that no committed code, data, or completed experiment
> supported; they were retracted. Every number below is produced by committed
> code and reproduces deterministically - see [Evaluation](#evaluation-seeded-offline-harness).

## Design

![Architecture](docs/architecture.svg)

The verifier sits between the LLM planner and the Nav2 controller. A proposed
goal or trajectory passes only if it clears three deterministic checks against
the live costmap and robot model:

1. **Collision** - no pose intersects a lethal or inflated obstacle,
2. **Kinematic feasibility** - curvature and velocity within the platform's limits,
3. **Workspace bounds** - every waypoint inside the mapped, known region.

Rejected plans trigger a replan request instead of reaching the controller.

## Evaluation (seeded offline harness)

The verifier is exercised by a deterministic harness
([core/eval/eval_main.cpp](core/eval/eval_main.cpp)): 50 seeded procedural
indoor maps (12 m x 9 m at 0.05 m), oracle-verified safe trajectories, and six
classes of *constructed* hallucinations - a goal inside a wall, a straight
line through walls, an off-map waypoint, a teleport jump, a route into
unmapped space, and a gap too narrow for the robot's footprint. Every unsafe
label is confirmed by an independent reference checker (4x finer sampling)
before the case counts.

Output of `core/build/eval --maps 50 --seed 42` (verbatim, TurtleBot3 Burger
parameter profile):

```text
verifier eval: seed 42, 50 maps, 2071 cases (599 safe, 1472 unsafe), 29 generator retries skipped
  goal_in_wall    caught 250 / 250
  narrow_gap      caught 250 / 250
  off_map         caught 250 / 250
  teleport        caught 222 / 222
  unknown_region  caught 250 / 250
  wall_through    caught 250 / 250
unsafe caught: 1472 / 1472
safe rejected (false positives): 0 / 599
latency: median 10.3 us  p95 44.1 us  p99 77.0 us  max 1251.5 us
PASS: all unsafe caught, no false positives
```

Read this precisely: it says the deterministic checks catch **100% of these
six constructed violation classes with zero false positives on
oracle-verified safe paths**, at microsecond latency (measured on x86-64
WSL2; the harness re-runs in CI on every push and fails on any miss). It does
*not* yet say anything about real LLM outputs or real hardware - that is the
next step below, and those numbers will appear only with the data that
produces them. Case-level data: [reports/results/verifier_eval.csv](reports/results/verifier_eval.csv).

Correctness of the verifier itself is tested in
[core/tests/test_verifier.cpp](core/tests/test_verifier.cpp): unit cases per
violation class plus a fuzz invariant - every trajectory the verifier calls
safe is re-checked by the finer-sampled oracle (1,914 safe verdicts
cross-checked, printed by the test).

## Quick start (no ROS required)

```bash
git clone https://github.com/munawarkazmi/ros2-llm-safety-verifier.git
cd ros2-llm-safety-verifier
make -C core test
make -C core eval
core/build/eval --maps 50 --seed 42 --out reports/results/verifier_eval.csv
```

A containerized ROS 2 Humble + Nav2 environment is also provided
(`docker compose up --build`, image built by CI); the compose file mounts the
repository into the container workspace for the upcoming ROS integration.

## Roadmap

1. **Done - offline evaluation harness** (above): reproducible catch-rate and
   false-positive numbers on constructed hallucination classes.
2. Verifier node (C++, subscribing to the LLM planner's proposals, publishing
   verified goals to Nav2), reusing the tested core.
3. Evaluation against real LLM-generated trajectories (prompted plans over
   these maps), published with the prompts and raw outputs.
4. Hardware trials (TurtleBot3 + Jetson Orin Nano), published together with
   the raw rosbags and analysis scripts.

## License

MIT, Munawar Kazmi.
