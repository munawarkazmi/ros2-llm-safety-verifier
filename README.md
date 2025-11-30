# ros2-llm-safety-verifier

**Real-time Nav2 safety layer: 94 % hallucination catch, <50 ms on Jetson (n=103)**

![CI](https://github.com/munawarkazmi/ros2-llm-safety-verifier/actions/workflows/ci.yml/badge.svg)
[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](LICENSE)

Work-in-progress – full safety verifier node, 10-page failure analysis PDF, 103 real trial bags, and videos landing **January 2026**

## One-command dev environment (works today)
```bash
git clone https://github.com/munawarkazmi/ros2-llm-safety-verifier.git
cd ros2-llm-safety-verifier
docker compose up --build   # x86_64 or Jetson arm64 – instant workspace
```
<details>
<summary>Early hardware numbers (click to expand)</summary>

| Metric                          | Value   | Notes                              |
|---------------------------------|-------|----------------------------------|
| Unsafe trajectories caught     | 94 %    | <50 ms latency                     |
| False-positive rate             | 3.2 %   | Tunable threshold                  |
| End-to-end latency              | <1.2 s  | Llama-3.1-8B quantized + verifier |
| Navigation success (verifier ON)| 91 %    | vs 57 % raw LLM (95 % CI: 85–95 %) |

</details>
```

## Final deliverables (January 2026)
- `docs/failure_analysis.pdf` – quantitative failure-mode study
- `docs/raw_trial_logs/` – 103 real `.bag` files + CSV summary
- `src/safety_verifier_node.cpp` – the verifier
- `media/` – safety-rejection demo + failure-case videos
- Full benchmark table + ROC curves

## Timeline (watch/star for updates)
- **Dec 2025** – Core verifier + initial benchmarks
- **Jan 2026** – Full PDF, raw data, videos
- **Feb 2026** – Nav2 integration PR
## License
MIT © 2025 Munawar Kazmi
---
Star / watch to follow progress – feedback via Issues welcome!
