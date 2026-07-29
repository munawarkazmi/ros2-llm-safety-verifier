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
repository into the container workspace.

## Real-model evaluation: qwen2.5:7b-instruct

Roadmap step 3, run against a real model: `qwen2.5:7b-instruct` served
locally by Ollama at temperature 0, prompted once per scenario over 40 seeded
maps (coarse ASCII view; verification runs on the fine grid the LLM never
sees). The committed dataset is complete and replayable: prompts and raw
responses ([llm_eval/responses/](llm_eval/responses/)), parsed trajectories,
scenario grids, and the exact evaluator. The accounting taxonomy was
committed before any output was evaluated
([f9e2635](https://github.com/munawarkazmi/ros2-llm-safety-verifier/commit/f9e2635)).

Output of `core/build/llm_eval` (verbatim):

```text
model under evaluation: qwen2.5:7b-instruct (temperature 0)
responses: 40 total
  parse_failure  0
  degenerate     0
  safe_passed    5
  safe_rejected  0   (false positives vs oracle)
  unsafe_caught  35
  unsafe_missed  0   <-- the load-bearing cell; must be 0
evaluated as real plans: 40 of 40 responses (parse_failure and degenerate are excluded from every safety denominator and must be read alongside any catch claim)
unsafe plans by qwen2.5:7b-instruct (temperature 0): 35 of 40 evaluated
violation classes among verifier rejections:
  collision          8
  discontinuity      18
  off_map            9
endpoint adherence failures: 18 of 40 evaluated (task-success axis; independent of safety - the verifier speaks only to safety and no combined score is computed)
PASS: zero missed dangers - no oracle-unsafe plan passed the verifier
```

Reading this precisely - these are facts about one 7B model at one
temperature on n=40 scenarios, not about "LLMs":

- **qwen2.5:7b-instruct proposed unsafe trajectories in 35 of its 40 plans;
  the verifier caught all 35, passed all 5 safe ones, and missed zero** (the
  oracle independently confirms every verdict; CI replays this from the
  committed dataset on every push).
- All 40 responses were real plans - no parse failures, no degenerate
  outputs - so the catch numbers carry the full weight of the dataset.
- The failure modes span three classes, not one: waypoint spacing violations
  (discontinuity, 18), off-map coordinates (9), and paths through obstacles
  or cutting corners into them (collision, 8). Two constructed classes
  (routes into unmapped space, too-narrow gaps) were never elicited in these
  40 scenarios; the [constructed evaluation](#evaluation-seeded-offline-harness)
  remains the broader coverage of the check surface.
- Separately from safety, 18 of 40 plans failed endpoint adherence (start or
  goal not matched within 0.3 m) - a task-success observation about the
  model, outside the verifier's remit.

To reproduce collection you need any OpenAI-compatible endpoint
(`llm_eval/collect.py`); evaluation from the committed raw data is fully
deterministic (`llm_eval/parse.py`, then `core/build/llm_eval`).

## ROS 2 node

`verifier_node` wraps the tested core as the runtime gate
([src/verifier_node.cpp](src/verifier_node.cpp)); the package
colcon-builds against ROS 2 Humble in CI on every push. It is fail-safe by
construction: nothing is forwarded until a costmap has arrived and every
check passes.

| Topic | Type | Direction | Purpose |
| --- | --- | --- | --- |
| `costmap` | `nav_msgs/OccupancyGrid` | sub (transient_local) | remap to `/global_costmap/costmap` |
| `proposed_path` | `nav_msgs/Path` | sub | trajectory proposed by the LLM |
| `proposed_goal` | `geometry_msgs/PoseStamped` | sub | goal proposed by the LLM |
| `verified_path` | `nav_msgs/Path` | pub | forwarded only when safe |
| `verified_goal` | `geometry_msgs/PoseStamped` | pub | forwarded only when safe |
| `rejections` | `std_msgs/String` | pub | violation and waypoint for every rejection |

```bash
# inside a ROS 2 Humble workspace containing this repo (or the compose container)
colcon build --packages-select ros2_llm_safety_verifier
source install/setup.bash
ros2 launch ros2_llm_safety_verifier verifier_launch.py
```

Parameters (see [config/verifier_params.yaml](config/verifier_params.yaml))
mirror the core profile: `robot_radius`, `max_segment_length`, `sample_step`,
`min_turning_radius` (0 disables curvature for differential drive),
`allow_unknown`, `occupied_threshold`. Frames are compared textually; TF
transformation of proposals is future work. The node is build-verified in CI
and exercised against the core's tested checks; a live Nav2 integration test
is part of the hardware-trial roadmap step.

## Roadmap

1. **Done - offline evaluation harness** (above): reproducible catch-rate and
   false-positive numbers on constructed hallucination classes.
2. **Done - verifier node**: the runtime gate between `proposed_*` and
   `verified_*` topics, reusing the tested core (build-verified in CI).
3. **Done - real-model evaluation** (above): qwen2.5:7b-instruct over 40
   scenarios, raw prompts and responses committed, replayed in CI. Further
   models can be added with the same pipeline as access allows.
4. Hardware trials (TurtleBot3 + Jetson Orin Nano), published together with
   the raw rosbags and analysis scripts.

## License

MIT, Munawar Kazmi.
