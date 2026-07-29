// core/tools/dump_llm_scenarios.cpp
//
// Exports seeded navigation scenarios for the LLM evaluation:
//   llm_eval/scenarios/scenario_NN.pgm   fine grid, raw cost bytes (P5)
//   llm_eval/scenarios/scenario_NN.json  start/goal (world), map metadata,
//                                        and a coarse ASCII rendering
//
// The ASCII map downsamples the 240x180 grid by 5 (48x36 characters,
// 0.25 m per character). A coarse cell is '#' if any fine cell in its
// 5x5 block is lethal, '?' if any is unknown (and none lethal), '.'
// otherwise - so '.' regions are genuinely fully free at fine
// resolution. Start ('S') and goal ('G') are placed on fully free
// coarse cells with footprint clearance, with a route between them
// guaranteed to exist. The LLM plans on the coarse view; verification
// runs on the fine grid - exactly the deployment reality.
#include <cstdio>
#include <fstream>
#include <random>
#include <string>
#include <vector>

#include "../eval/scenarios.hpp"
#include "verifier/grid.hpp"

using scenarios::MapCase;
using verifier::Grid;
using verifier::Point;

namespace {

constexpr std::size_t kFactor = 5;

char coarseChar(const Grid& g, std::size_t cx, std::size_t cy) {
  bool unknown = false;
  for (std::size_t dy = 0; dy < kFactor; ++dy) {
    for (std::size_t dx = 0; dx < kFactor; ++dx) {
      const std::uint8_t c = g.cost(cx * kFactor + dx, cy * kFactor + dy);
      if (c == verifier::kLethal) return '#';
      if (c == verifier::kUnknown) unknown = true;
    }
  }
  return unknown ? '?' : '.';
}

bool coarseCellFullyClear(const Grid& g, const std::vector<std::uint8_t>& blocked,
                          std::size_t fx, std::size_t fy) {
  const std::size_t cx = fx / kFactor, cy = fy / kFactor;
  if (coarseChar(g, cx, cy) != '.') return false;
  return !blocked[g.index(fx, fy)];
}

void writePgm(const std::string& path, const Grid& g) {
  std::ofstream f(path, std::ios::binary);
  f << "P5\n" << g.width() << ' ' << g.height() << "\n255\n";
  f.write(reinterpret_cast<const char*>(g.data().data()),
          static_cast<std::streamsize>(g.data().size()));
}

}  // namespace

int main(int argc, char** argv) {
  const int count = argc > 1 ? std::atoi(argv[1]) : 40;
  const unsigned seed = argc > 2 ? static_cast<unsigned>(std::atoi(argv[2])) : 42;
  const std::string outdir = argc > 3 ? argv[3] : "llm_eval/scenarios";

  const verifier::Params params;  // TurtleBot3 profile
  std::mt19937 rng(seed);

  for (int id = 0; id < count; ++id) {
    const MapCase mc = scenarios::makeMap(rng);
    const Grid& g = mc.grid;
    const auto blocked = scenarios::clearanceMask(g, params);

    // Start/goal: fully free coarse cells, clearance, route exists.
    std::pair<std::size_t, std::size_t> s{}, t{};
    bool found = false;
    for (int attempt = 0; attempt < 300 && !found; ++attempt) {
      s = scenarios::randomClearCell(rng, g, blocked);
      t = scenarios::randomClearCell(rng, g, blocked);
      if (!coarseCellFullyClear(g, blocked, s.first, s.second)) continue;
      if (!coarseCellFullyClear(g, blocked, t.first, t.second)) continue;
      const auto dx = static_cast<double>(s.first) - static_cast<double>(t.first);
      const auto dy = static_cast<double>(s.second) - static_cast<double>(t.second);
      if (dx * dx + dy * dy < 100.0 * 100.0) continue;  // at least 5 m apart
      if (!scenarios::cellPath(g, blocked, s, t)) continue;
      found = true;
    }
    if (!found) {
      std::fprintf(stderr, "scenario %d: no start/goal found, regenerating map\n", id);
      --id;
      continue;
    }

    const Point start = g.cellCenter(s.first, s.second);
    const Point goal = g.cellCenter(t.first, t.second);

    char buf[64];
    std::snprintf(buf, sizeof buf, "%s/scenario_%02d.pgm", outdir.c_str(), id);
    writePgm(buf, g);

    std::snprintf(buf, sizeof buf, "%s/scenario_%02d.json", outdir.c_str(), id);
    std::ofstream j(buf);
    j << "{\n  \"id\": " << id << ",\n  \"width\": " << g.width()
      << ",\n  \"height\": " << g.height()
      << ",\n  \"resolution\": " << g.resolution()
      << ",\n  \"ascii_cell_m\": " << g.resolution() * kFactor
      << ",\n  \"start\": [" << start.x << ", " << start.y << "]"
      << ",\n  \"goal\": [" << goal.x << ", " << goal.y << "]"
      << ",\n  \"ascii\": [\n";
    const std::size_t cw = g.width() / kFactor, ch = g.height() / kFactor;
    for (std::size_t row = 0; row < ch; ++row) {
      const std::size_t cy = ch - 1 - row;  // top row = max y
      std::string line;
      for (std::size_t cx = 0; cx < cw; ++cx) {
        char c = coarseChar(g, cx, cy);
        if (cx == s.first / kFactor && cy == s.second / kFactor) c = 'S';
        if (cx == t.first / kFactor && cy == t.second / kFactor) c = 'G';
        line += c;
      }
      j << "    \"" << line << '"' << (row + 1 < ch ? "," : "") << '\n';
    }
    j << "  ]\n}\n";
  }
  std::printf("wrote %d scenarios (seed %u) to %s\n", count, seed, outdir.c_str());
  return 0;
}
