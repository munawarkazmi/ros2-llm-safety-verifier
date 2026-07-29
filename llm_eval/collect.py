#!/usr/bin/env python3
"""Collect LLM trajectory proposals for the evaluation scenarios.

Standard library only. Talks to any OpenAI-compatible chat endpoint
(local Ollama by default). Appends one JSON record per scenario to
responses/<name>.jsonl and skips scenarios already present, so an
interrupted run resumes cleanly. The raw responses are committed as the
dataset; evaluation is a separate, deterministic step.

Usage (local Ollama):
  python3 llm_eval/collect.py --name qwen2.5-7b-instruct \
      --base-url http://localhost:11434/v1 --model qwen2.5:7b-instruct
"""
import argparse
import json
import os
import pathlib
import sys
import time
import urllib.request

ROOT = pathlib.Path(__file__).resolve().parent


def build_prompt(scenario: dict, template: str) -> str:
    rows = len(scenario["ascii"])
    cell = scenario["ascii_cell_m"]
    return (
        template.replace("{ascii_cell_m}", f"{cell:g}")
        .replace("{map_w_m}", f"{scenario['width'] * scenario['resolution']:g}")
        .replace("{map_h_m}", f"{scenario['height'] * scenario['resolution']:g}")
        .replace("{rows}", str(rows))
        .replace("{ascii_map}", "\n".join(scenario["ascii"]))
        .replace("{start_x}", f"{scenario['start'][0]:.3f}")
        .replace("{start_y}", f"{scenario['start'][1]:.3f}")
        .replace("{goal_x}", f"{scenario['goal'][0]:.3f}")
        .replace("{goal_y}", f"{scenario['goal'][1]:.3f}")
    )


def call(base_url: str, model: str, prompt: str, api_key: str | None,
         max_tokens: int, timeout_s: float) -> str:
    payload = {
        "model": model,
        "temperature": 0.0,
        "max_tokens": max_tokens,
        "messages": [{"role": "user", "content": prompt}],
    }
    headers = {"Content-Type": "application/json",
               "User-Agent": "ros2-llm-safety-verifier-eval/0.1"}
    if api_key:
        headers["Authorization"] = f"Bearer {api_key}"
    req = urllib.request.Request(
        base_url.rstrip("/") + "/chat/completions",
        data=json.dumps(payload).encode(), headers=headers, method="POST")
    with urllib.request.urlopen(req, timeout=timeout_s) as resp:
        data = json.loads(resp.read().decode())
    return data["choices"][0]["message"]["content"]


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--name", required=True, help="output name (responses/<name>.jsonl)")
    ap.add_argument("--base-url", required=True)
    ap.add_argument("--model", required=True)
    ap.add_argument("--api-key-env", default=None)
    ap.add_argument("--max-tokens", type=int, default=3000)
    ap.add_argument("--timeout-s", type=float, default=600.0)
    ap.add_argument("--min-interval-s", type=float, default=0.0)
    args = ap.parse_args()

    api_key = os.environ.get(args.api_key_env) if args.api_key_env else None
    template = (ROOT / "prompts" / "template.txt").read_text()
    out_path = ROOT / "responses" / f"{args.name}.jsonl"
    out_path.parent.mkdir(parents=True, exist_ok=True)

    done = set()
    if out_path.exists():
        for line in out_path.read_text().splitlines():
            if line.strip():
                done.add(json.loads(line)["scenario"])

    scenario_files = sorted((ROOT / "scenarios").glob("scenario_*.json"))
    if not scenario_files:
        print("no scenarios found; run dump_llm_scenarios first", file=sys.stderr)
        return 2

    last = 0.0
    with out_path.open("a") as out:
        for sf in scenario_files:
            scenario = json.loads(sf.read_text())
            sid = scenario["id"]
            if sid in done:
                continue
            prompt = build_prompt(scenario, template)
            wait = args.min_interval_s - (time.time() - last)
            if wait > 0:
                time.sleep(wait)
            t0 = time.time()
            try:
                response = call(args.base_url, args.model, prompt, api_key,
                                args.max_tokens, args.timeout_s)
            except Exception as exc:  # noqa: BLE001 - record and continue
                print(f"scenario {sid}: ERROR {exc}", file=sys.stderr)
                continue
            last = time.time()
            record = {
                "scenario": sid,
                "model": args.model,
                "temperature": 0.0,
                "prompt": prompt,
                "response": response,
                "elapsed_s": round(last - t0, 2),
            }
            out.write(json.dumps(record) + "\n")
            out.flush()
            print(f"scenario {sid}: ok ({record['elapsed_s']}s)")
    return 0


if __name__ == "__main__":
    sys.exit(main())
