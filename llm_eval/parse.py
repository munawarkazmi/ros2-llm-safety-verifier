#!/usr/bin/env python3
"""Parse raw LLM responses into waypoint lists.

Deterministic: reads responses/<name>.jsonl, extracts the first JSON
array of [x, y] pairs from each response (handling code fences and
surrounding prose), and writes parsed/<name>.json. Responses with no
extractable waypoint array are kept with parse_ok=false - parse
failures are part of the phenomenon being measured, not discarded.

Usage: python3 llm_eval/parse.py --name qwen2.5-7b-instruct
"""
import argparse
import json
import pathlib
import sys

ROOT = pathlib.Path(__file__).resolve().parent


def extract_waypoints(text: str):
    """First balanced JSON array in the text that decodes to [[x,y],...]."""
    for start in range(len(text)):
        if text[start] != "[":
            continue
        depth = 0
        for end in range(start, len(text)):
            if text[end] == "[":
                depth += 1
            elif text[end] == "]":
                depth -= 1
                if depth == 0:
                    candidate = text[start:end + 1]
                    try:
                        data = json.loads(candidate)
                    except json.JSONDecodeError:
                        break  # try the next '['
                    if (isinstance(data, list) and len(data) >= 1 and
                            all(isinstance(p, list) and len(p) == 2 and
                                all(isinstance(v, (int, float)) for v in p)
                                for p in data)):
                        return [[float(p[0]), float(p[1])] for p in data]
                    break
        # fall through to next start position
    return None


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--name", required=True)
    args = ap.parse_args()

    in_path = ROOT / "responses" / f"{args.name}.jsonl"
    out_path = ROOT / "parsed" / f"{args.name}.json"
    out_path.parent.mkdir(parents=True, exist_ok=True)

    records = []
    for line in in_path.read_text().splitlines():
        if not line.strip():
            continue
        rec = json.loads(line)
        waypoints = extract_waypoints(rec["response"])
        records.append({
            "scenario": rec["scenario"],
            "model": rec["model"],
            "parse_ok": waypoints is not None,
            "waypoints": waypoints or [],
        })

    records.sort(key=lambda r: r["scenario"])
    out_path.write_text(json.dumps(records, indent=1) + "\n")

    # Line-based mirror for the C++ evaluator (no JSON parser needed
    # there). Includes each scenario's start/goal for endpoint checks.
    txt_lines = []
    for r in records:
        scenario = json.loads(
            (ROOT / "scenarios" / f"scenario_{r['scenario']:02d}.json").read_text())
        txt_lines.append(f"scenario {r['scenario']}")
        txt_lines.append(f"parse_ok {1 if r['parse_ok'] else 0}")
        txt_lines.append(f"start {scenario['start'][0]} {scenario['start'][1]}")
        txt_lines.append(f"goal {scenario['goal'][0]} {scenario['goal'][1]}")
        txt_lines.append(f"waypoints {len(r['waypoints'])}")
        for x, y in r["waypoints"]:
            txt_lines.append(f"{x} {y}")
    (ROOT / "parsed" / f"{args.name}.txt").write_text("\n".join(txt_lines) + "\n")

    ok = sum(1 for r in records if r["parse_ok"])
    print(f"{args.name}: {ok}/{len(records)} responses parsed to waypoint lists")
    return 0


if __name__ == "__main__":
    sys.exit(main())
