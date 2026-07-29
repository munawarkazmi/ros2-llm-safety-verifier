// core/eval/eval_main.cpp
//
// Seeded evaluation harness for the trajectory verifier.
//
// For each of --maps procedural indoor maps: generate --safe safe
// trajectories (oracle-verified) and --per-class cases of each of six
// hallucination classes (unsafe by construction AND oracle-confirmed).
// Run the verifier on every case, measure per-call latency, and report:
//   - catch rate per class and overall (unsafe cases flagged),
//   - false-positive rate (safe cases rejected),
//   - latency median / p95 / p99 / max.
// Exit code is nonzero if any unsafe case slips through or any safe
// case is rejected, so CI enforces the numbers the README quotes.
//
// Ground-truth discipline: labels never come from the verifier under
// test - safe cases are validated by the independent reference checker
// in scenarios.hpp (4x finer sampling), and every constructed unsafe
// case is confirmed unsafe by the same oracle before being counted.
#include <algorithm>
#include <chrono>
#include <cstdio>
#include <fstream>
#include <map>
#include <string>
#include <vector>

#include "scenarios.hpp"
#include "verifier/verifier.hpp"

using scenarios::Case;
using verifier::Params;

namespace {

double median(std::vector<double> v) {
  if (v.empty()) return 0.0;
  std::sort(v.begin(), v.end());
  const std::size_t n = v.size();
  return n % 2 ? v[n / 2] : 0.5 * (v[n / 2 - 1] + v[n / 2]);
}

double percentile(std::vector<double> v, double q) {
  if (v.empty()) return 0.0;
  std::sort(v.begin(), v.end());
  const auto idx = static_cast<std::size_t>(q * static_cast<double>(v.size() - 1));
  return v[idx];
}

}  // namespace

int main(int argc, char** argv) {
  int maps = 50, safe_per_map = 12, per_class = 5;
  unsigned seed = 42;
  std::string out = "reports/results/verifier_eval.csv";
  for (int i = 1; i + 1 < argc; i += 2) {
    const std::string k = argv[i], v = argv[i + 1];
    if (k == "--maps") maps = std::stoi(v);
    else if (k == "--safe") safe_per_map = std::stoi(v);
    else if (k == "--per-class") per_class = std::stoi(v);
    else if (k == "--seed") seed = static_cast<unsigned>(std::stoul(v));
    else if (k == "--out") out = v;
    else { std::fprintf(stderr, "unknown arg %s\n", k.c_str()); return 2; }
  }

  const Params params;  // TurtleBot3 Burger profile (see verifier.hpp)
  const std::vector<std::string> classes = {"goal_in_wall", "wall_through", "off_map",
                                            "teleport", "unknown_region", "narrow_gap"};

  std::ofstream csv(out);
  if (!csv) {
    std::fprintf(stderr, "cannot write %s\n", out.c_str());
    return 2;
  }
  csv << "map,case,label,ground_truth_safe,verdict_safe,violation,waypoints,latency_us\n";

  std::map<std::string, std::pair<int, int>> per_class_stats;  // caught / total
  int safe_total = 0, safe_rejected = 0, unsafe_total = 0, unsafe_caught = 0;
  int skipped = 0;
  std::vector<double> lat_us;

  std::mt19937 rng(seed);
  for (int m = 0; m < maps; ++m) {
    const scenarios::MapCase mc = scenarios::makeMap(rng);
    const auto blocked = scenarios::clearanceMask(mc.grid, params);

    std::vector<Case> cases;
    for (int i = 0; i < safe_per_map; ++i) {
      auto t = scenarios::makeSafeTrajectory(rng, mc.grid, blocked, params);
      if (!t) { ++skipped; continue; }
      cases.push_back({"safe", true, std::move(*t)});
    }
    for (const auto& cls : classes) {
      for (int i = 0; i < per_class; ++i) {
        auto c = scenarios::makeUnsafe(cls, rng, mc, blocked, params);
        if (!c) { ++skipped; continue; }
        cases.push_back(std::move(*c));
      }
    }

    for (std::size_t ci = 0; ci < cases.size(); ++ci) {
      const Case& c = cases[ci];
      const auto t0 = std::chrono::steady_clock::now();
      const auto verdict = verifier::verify(mc.grid, c.traj, params);
      const auto t1 = std::chrono::steady_clock::now();
      const double us = std::chrono::duration<double, std::micro>(t1 - t0).count();
      lat_us.push_back(us);

      if (c.safe) {
        ++safe_total;
        if (!verdict.safe) ++safe_rejected;
      } else {
        ++unsafe_total;
        auto& s = per_class_stats[c.label];
        ++s.second;
        if (!verdict.safe) { ++s.first; ++unsafe_caught; }
      }
      csv << m << ',' << ci << ',' << c.label << ',' << (c.safe ? 1 : 0) << ','
          << (verdict.safe ? 1 : 0) << ',' << verifier::toString(verdict.violation)
          << ',' << c.traj.size() << ',' << us << '\n';
    }
  }

  std::printf("verifier eval: seed %u, %d maps, %d cases (%d safe, %d unsafe), "
              "%d generator retries skipped\n",
              seed, maps, safe_total + unsafe_total, safe_total, unsafe_total, skipped);
  for (const auto& [cls, s] : per_class_stats) {
    std::printf("  %-15s caught %d / %d\n", cls.c_str(), s.first, s.second);
  }
  std::printf("unsafe caught: %d / %d\n", unsafe_caught, unsafe_total);
  std::printf("safe rejected (false positives): %d / %d\n", safe_rejected, safe_total);
  std::printf("latency: median %.1f us  p95 %.1f us  p99 %.1f us  max %.1f us\n",
              median(lat_us), percentile(lat_us, 0.95), percentile(lat_us, 0.99),
              lat_us.empty() ? 0.0 : *std::max_element(lat_us.begin(), lat_us.end()));

  const bool ok = (unsafe_caught == unsafe_total) && (safe_rejected == 0);
  std::printf("%s\n", ok ? "PASS: all unsafe caught, no false positives"
                         : "FAIL: verifier disagreed with ground truth");
  return ok ? 0 : 1;
}
