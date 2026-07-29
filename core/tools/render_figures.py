#!/usr/bin/env python3
"""Render the README figures from the committed evaluation data.

Everything drawn here derives from committed artifacts:
  - reports/results/verifier_eval.csv        (constructed evaluation)
  - reports/results/llm_eval_*.csv           (qwen2.5-7B evaluation)
  - llm_eval/parsed/*.json + scenarios/*.pgm (real model trajectories)

Usage: render_figures.py <outdir>
"""
import csv
import json
import pathlib
import struct
import sys

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt

ROOT = pathlib.Path(__file__).resolve().parents[2]

NAVY = "#123a5f"
BLUE = "#9db8d2"
GOLD = "#d9a441"
RED = "#b03a2e"
GRAY = "#8a8f98"

MODEL = "qwen2.5-7b-instruct"
MODEL_LABEL = "qwen2.5:7b-instruct (temperature 0)"


def read_cost_pgm(path):
    data = path.read_bytes()
    # P5\n<w> <h>\n255\n<bytes>
    header, _, rest = data.partition(b"255\n")
    fields = header.split()
    w, h = int(fields[1]), int(fields[2])
    return w, h, rest[: w * h]


def fig_qwen_outcomes(out):
    rows = list(csv.DictReader(
        (ROOT / "reports/results" / f"llm_eval_{MODEL}.csv").open()))
    buckets = ["parse_failure", "degenerate", "safe_passed", "safe_rejected",
               "unsafe_caught", "unsafe_missed"]
    counts = {b: sum(1 for r in rows if r["bucket"] == b) for b in buckets}
    classes = {}
    for r in rows:
        if r["bucket"] in ("unsafe_caught", "unsafe_missed") and r["violation"]:
            classes[r["violation"]] = classes.get(r["violation"], 0) + 1

    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(11, 4.8))

    colors = [GRAY, GRAY, "#4d7a4f", GOLD, NAVY, RED]
    bars = ax1.barh(list(range(len(buckets))), [counts[b] for b in buckets],
                    color=colors)
    ax1.set_yticks(list(range(len(buckets))), buckets)
    ax1.invert_yaxis()
    ax1.bar_label(bars, padding=3, fontsize=10)
    ax1.set_xlabel("plans")
    ax1.set_title(f"All 40 {MODEL_LABEL} plans\nby pre-registered outcome bucket",
                  fontsize=10)
    ax1.set_xlim(0, max(counts.values()) * 1.15)

    names = sorted(classes, key=classes.get, reverse=True)
    bars2 = ax2.barh(list(range(len(names))), [classes[n] for n in names], color=NAVY)
    ax2.set_yticks(list(range(len(names))), names)
    ax2.invert_yaxis()
    ax2.bar_label(bars2, padding=3, fontsize=10)
    ax2.set_xlabel("unsafe plans (first violation found)")
    ax2.set_title("What the model actually got wrong\n(violation class of each rejection)",
                  fontsize=10)

    fig.suptitle(f"Real-model evaluation: {MODEL_LABEL}, 40 scenarios - "
                 "zero missed dangers", fontsize=12, color=NAVY)
    fig.tight_layout()
    fig.savefig(out, dpi=110)
    plt.close(fig)


def fig_qwen_example(out):
    """One actual unsafe qwen plan on its scenario map: first collision case."""
    rows = list(csv.DictReader(
        (ROOT / "reports/results" / f"llm_eval_{MODEL}.csv").open()))
    case = next(r for r in rows
                if r["bucket"] == "unsafe_caught" and r["violation"] == "collision")
    sid = int(case["scenario"])
    widx = int(case["waypoint_index"])

    parsed = json.loads((ROOT / "llm_eval/parsed" / f"{MODEL}.json").read_text())
    traj = next(r["waypoints"] for r in parsed if r["scenario"] == sid)
    scen = json.loads(
        (ROOT / "llm_eval/scenarios" / f"scenario_{sid:02d}.json").read_text())

    w, h, cells = read_cost_pgm(ROOT / "llm_eval/scenarios" / f"scenario_{sid:02d}.pgm")
    res = scen["resolution"]
    img = [[0] * w for _ in range(h)]
    for y in range(h):
        for x in range(w):
            c = cells[y * w + x]
            img[y][x] = 0.15 if c == 254 else (0.6 if c == 255 else 1.0)

    fig, ax = plt.subplots(figsize=(8.5, 6.5))
    ax.imshow(img, cmap="gray", origin="lower", vmin=0, vmax=1,
              extent=[0, w * res, 0, h * res], interpolation="nearest")
    xs = [p[0] for p in traj]
    ys = [p[1] for p in traj]
    ax.plot(xs, ys, c=NAVY, lw=2, marker="o", ms=3.5, label="qwen2.5-7B plan")
    ax.scatter([scen["start"][0]], [scen["start"][1]], c=GOLD, s=110, zorder=5,
               label="start")
    ax.scatter([scen["goal"][0]], [scen["goal"][1]], c="#4d7a4f", s=110, zorder=5,
               label="goal")
    vi = min(widx, len(traj) - 1)
    ax.scatter([traj[vi][0]], [traj[vi][1]], marker="X", c=RED, s=170, zorder=6,
               label=f"verifier: collision at waypoint {widx}")
    ax.legend(loc="upper right", fontsize=9)
    ax.set_xlabel("x (m)")
    ax.set_ylabel("y (m)")
    ax.set_title(f"An actual {MODEL_LABEL} plan, scenario {sid}: "
                 "the route crosses an obstacle;\nthe verifier rejects it before "
                 "it reaches the controller (committed data, replayable)",
                 fontsize=10, color=NAVY)
    fig.tight_layout()
    fig.savefig(out, dpi=110)
    plt.close(fig)


def fig_latency(out):
    lat = []
    with (ROOT / "reports/results/verifier_eval.csv").open() as f:
        for row in csv.DictReader(f):
            lat.append(float(row["latency_us"]))
    lat.sort()
    n = len(lat)
    med = lat[n // 2]
    p95 = lat[int(0.95 * (n - 1))]
    p99 = lat[int(0.99 * (n - 1))]

    fig, ax = plt.subplots(figsize=(8.5, 4.6))
    bins = [1, 2, 5, 10, 20, 50, 100, 200, 500, 1000, 2000]
    ax.hist(lat, bins=bins, color=NAVY, alpha=0.85)
    ax.set_xscale("log")
    ax.set_xlabel("verification latency (us, log scale)")
    ax.set_ylabel("cases")
    for v, name in [(med, f"median {med:.1f}"), (p95, f"p95 {p95:.1f}"),
                    (p99, f"p99 {p99:.1f}")]:
        ax.axvline(v, color=RED, ls="--", lw=1.2)
        ax.text(v, ax.get_ylim()[1] * 0.92, f" {name} us", color=RED, fontsize=9,
                rotation=90, va="top")
    ax.set_title(f"Verification latency over all {n} constructed-evaluation cases "
                 "(x86-64, committed CSV)", fontsize=10, color=NAVY)
    fig.tight_layout()
    fig.savefig(out, dpi=110)
    plt.close(fig)


def main():
    outdir = pathlib.Path(sys.argv[1]) if len(sys.argv) > 1 else ROOT / "docs/figures"
    outdir.mkdir(parents=True, exist_ok=True)
    fig_qwen_outcomes(outdir / "qwen_outcomes.png")
    fig_qwen_example(outdir / "qwen_example.png")
    fig_latency(outdir / "verifier_latency.png")
    print("figures written to", outdir)


if __name__ == "__main__":
    main()
