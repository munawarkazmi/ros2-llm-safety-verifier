// core/eval/scenarios.hpp
//
// Shared machinery for the evaluation harness and the fuzz tests:
//   - seeded procedural indoor maps,
//   - a reference safety checker, independent of the verifier's code
//     path and sampling 4x finer, used as the labeling oracle,
//   - generators for safe trajectories and six hallucination classes
//     whose unsafe labels hold by construction AND are re-checked with
//     the oracle before a case is emitted.
#pragma once

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <optional>
#include <queue>
#include <random>
#include <string>
#include <utility>
#include <vector>

#include "verifier/grid.hpp"
#include "verifier/verifier.hpp"

namespace scenarios {

using verifier::Grid;
using verifier::Params;
using verifier::Point;
using verifier::Trajectory;

// ---------------------------------------------------------------- maps

// 12 m x 9 m indoor map: border walls, room-divider walls with wide
// doorways, rectangular obstacles, one unknown region, and one wall
// with a deliberately too-narrow gap (used by the narrow_gap class).
struct MapCase {
  Grid grid;
  // The narrow gap's center (world) and its wall's orientation.
  Point narrow_gap_center;
  bool narrow_gap_vertical;
  // Unknown rectangle center (world).
  Point unknown_center;
};

inline MapCase makeMap(std::mt19937& rng) {
  const std::size_t W = 240, H = 180;
  const double res = 0.05;
  Grid g(W, H, res, {0.0, 0.0}, 0);

  auto wallRect = [&](std::size_t x0, std::size_t y0, std::size_t x1, std::size_t y1) {
    for (std::size_t y = y0; y <= y1 && y < H; ++y)
      for (std::size_t x = x0; x <= x1 && x < W; ++x)
        g.setCost(x, y, verifier::kLethal);
  };

  // Border, 2 cells thick.
  wallRect(0, 0, W - 1, 1);
  wallRect(0, H - 2, W - 1, H - 1);
  wallRect(0, 0, 1, H - 1);
  wallRect(W - 2, 0, W - 1, H - 1);

  std::uniform_int_distribution<std::size_t> xr(20, W - 21), yr(20, H - 21);

  // Two vertical + one horizontal divider, each with a 7-cell doorway
  // (0.35 m, comfortably wider than the 0.21 m robot).
  auto divider = [&](bool vertical) {
    if (vertical) {
      const std::size_t x = xr(rng);
      const std::size_t door = yr(rng);
      for (std::size_t y = 2; y < H - 2; ++y) {
        if (y >= door && y < door + 7) continue;
        g.setCost(x, y, verifier::kLethal);
        g.setCost(x + 1, y, verifier::kLethal);
      }
    } else {
      const std::size_t y = yr(rng);
      const std::size_t door = xr(rng);
      for (std::size_t x = 2; x < W - 2; ++x) {
        if (x >= door && x < door + 7) continue;
        g.setCost(x, y, verifier::kLethal);
        g.setCost(x, y + 1, verifier::kLethal);
      }
    }
  };
  divider(true);
  divider(true);
  divider(false);

  // Furniture: random rectangles.
  std::uniform_int_distribution<std::size_t> fw(4, 14);
  for (int i = 0; i < 10; ++i) {
    const std::size_t x0 = xr(rng), y0 = yr(rng);
    wallRect(x0, y0, x0 + fw(rng), y0 + fw(rng));
  }

  // A short free-standing wall with a 3-cell (0.15 m) gap: the gap's
  // cells are free, but the 0.21 m robot cannot fit.
  const bool vert = (rng() & 1u) != 0;
  const std::size_t gx = xr(rng), gy = yr(rng);
  if (vert) {
    for (std::size_t y = (gy > 20 ? gy - 20 : 2); y <= gy + 20 && y < H - 2; ++y) {
      if (y >= gy - 1 && y <= gy + 1) continue;  // the gap
      g.setCost(gx, y, verifier::kLethal);
    }
  } else {
    for (std::size_t x = (gx > 20 ? gx - 20 : 2); x <= gx + 20 && x < W - 2; ++x) {
      if (x >= gx - 1 && x <= gx + 1) continue;
      g.setCost(x, gy, verifier::kLethal);
    }
  }
  // Clear the gap cells themselves in case furniture landed there.
  if (vert) {
    for (std::size_t y = gy - 1; y <= gy + 1; ++y) g.setCost(gx, y, 0);
  } else {
    for (std::size_t x = gx - 1; x <= gx + 1; ++x) g.setCost(x, gy, 0);
  }

  // Unknown region: a 20x20-cell rectangle of kUnknown over free space.
  const std::size_t ux = xr(rng), uy = yr(rng);
  for (std::size_t y = uy; y < uy + 20 && y < H - 2; ++y)
    for (std::size_t x = ux; x < ux + 20 && x < W - 2; ++x)
      if (g.cost(x, y) == 0) g.setCost(x, y, verifier::kUnknown);

  const Point gap_center = g.cellCenter(gx, gy);
  const Point unknown_center = g.cellCenter(ux + 10, uy + 10);
  return {std::move(g), gap_center, vert, unknown_center};
}

// ------------------------------------------------------------- oracle

// Reference safety check, used to label cases. Independent loop
// structure from the verifier and 4x finer sampling; conservative by
// construction (finer sampling can only find more violations).
inline bool referenceSafe(const Grid& g, const Trajectory& traj, const Params& p) {
  if (traj.empty()) return false;
  const double step = p.sample_step / 4.0;

  auto circleClear = [&](Point c) {
    const double r = p.robot_radius;
    const double res = g.resolution();
    const auto x0 = static_cast<std::int64_t>(std::floor((c.x - r - g.origin().x) / res));
    const auto x1 = static_cast<std::int64_t>(std::floor((c.x + r - g.origin().x) / res));
    const auto y0 = static_cast<std::int64_t>(std::floor((c.y - r - g.origin().y) / res));
    const auto y1 = static_cast<std::int64_t>(std::floor((c.y + r - g.origin().y) / res));
    for (std::int64_t y = y0; y <= y1; ++y) {
      for (std::int64_t x = x0; x <= x1; ++x) {
        // Intersection test before the bounds test, so only cells the
        // circle truly touches can declare the footprint off-map
        // (mirrors the fix in the verifier itself).
        const double ccx = g.origin().x + (static_cast<double>(x) + 0.5) * res;
        const double ccy = g.origin().y + (static_cast<double>(y) + 0.5) * res;
        const double dx = std::max(0.0, std::abs(ccx - c.x) - res / 2.0);
        const double dy = std::max(0.0, std::abs(ccy - c.y) - res / 2.0);
        if (dx * dx + dy * dy > r * r) continue;
        if (x < 0 || y < 0 ||
            !g.inBounds(static_cast<std::size_t>(x), static_cast<std::size_t>(y))) {
          return false;
        }
        const std::uint8_t cost = g.cost(static_cast<std::size_t>(x),
                                        static_cast<std::size_t>(y));
        if (cost == verifier::kUnknown ? !p.allow_unknown : cost >= verifier::kLethal) {
          return false;
        }
      }
    }
    return true;
  };

  for (std::size_t i = 0; i < traj.size(); ++i) {
    std::size_t mx, my;
    if (!g.worldToMap(traj[i], mx, my)) return false;
    if (i == 0) {
      if (!circleClear(traj[0])) return false;
      continue;
    }
    const Point a = traj[i - 1], b = traj[i];
    const double len = std::hypot(b.x - a.x, b.y - a.y);
    if (len > p.max_segment_length) return false;
    const auto n = static_cast<std::size_t>(std::ceil(len / step));
    for (std::size_t s = 1; s <= n; ++s) {
      const double t = static_cast<double>(s) / static_cast<double>(n);
      if (!circleClear({a.x + t * (b.x - a.x), a.y + t * (b.y - a.y)})) return false;
    }
  }
  if (p.min_turning_radius > 0.0) {
    // Mirror of the verifier's curvature rule (only used by car-like
    // profiles, which the current harness does not exercise).
    for (std::size_t i = 2; i < traj.size(); ++i) {
      const Point a = traj[i - 2], b = traj[i - 1], c = traj[i];
      const double cross = (b.x - a.x) * (c.y - a.y) - (b.y - a.y) * (c.x - a.x);
      if (std::abs(cross) < 1e-12) continue;
      const double rr = std::hypot(b.x - a.x, b.y - a.y) *
                        std::hypot(c.x - b.x, c.y - b.y) *
                        std::hypot(c.x - a.x, c.y - a.y) / (2.0 * std::abs(cross));
      if (rr < p.min_turning_radius) return false;
    }
  }
  return true;
}

// ------------------------------------------------- safe path generator

// A* over cells whose clearance allows the robot footprint (checked by
// stamping lethal+unknown inflation of robot_radius + one cell margin).
inline std::vector<std::uint8_t> clearanceMask(const Grid& g, const Params& p) {
  const double res = g.resolution();
  const auto infl = static_cast<std::int64_t>(std::ceil((p.robot_radius + res) / res));
  std::vector<std::uint8_t> blocked(g.width() * g.height(), 0);
  for (std::size_t y = 0; y < g.height(); ++y) {
    for (std::size_t x = 0; x < g.width(); ++x) {
      const std::uint8_t c = g.cost(x, y);
      if (c < verifier::kLethal && c != verifier::kUnknown) continue;
      for (std::int64_t dy = -infl; dy <= infl; ++dy) {
        for (std::int64_t dx = -infl; dx <= infl; ++dx) {
          if (dx * dx + dy * dy > infl * infl) continue;
          const auto nx = static_cast<std::int64_t>(x) + dx;
          const auto ny = static_cast<std::int64_t>(y) + dy;
          if (nx >= 0 && ny >= 0 &&
              g.inBounds(static_cast<std::size_t>(nx), static_cast<std::size_t>(ny))) {
            blocked[g.index(static_cast<std::size_t>(nx),
                            static_cast<std::size_t>(ny))] = 1;
          }
        }
      }
    }
  }
  return blocked;
}

inline std::optional<std::vector<std::pair<std::size_t, std::size_t>>>
cellPath(const Grid& g, const std::vector<std::uint8_t>& blocked,
         std::pair<std::size_t, std::size_t> s, std::pair<std::size_t, std::size_t> t) {
  const std::size_t W = g.width(), H = g.height();
  if (blocked[g.index(s.first, s.second)] || blocked[g.index(t.first, t.second)]) {
    return std::nullopt;
  }
  // Plain BFS (8-connected) - path quality does not matter here, only
  // that the route respects clearance.
  std::vector<std::int64_t> parent(W * H, -1);
  std::queue<std::size_t> q;
  const std::size_t si = g.index(s.first, s.second), ti = g.index(t.first, t.second);
  parent[si] = static_cast<std::int64_t>(si);
  q.push(si);
  while (!q.empty()) {
    const std::size_t cur = q.front();
    q.pop();
    if (cur == ti) break;
    const std::size_t x = cur % W, y = cur / W;
    for (int dy = -1; dy <= 1; ++dy) {
      for (int dx = -1; dx <= 1; ++dx) {
        if (dx == 0 && dy == 0) continue;
        const auto nx = static_cast<std::size_t>(static_cast<std::int64_t>(x) + dx);
        const auto ny = static_cast<std::size_t>(static_cast<std::int64_t>(y) + dy);
        if (nx >= W || ny >= H) continue;
        const std::size_t ni = g.index(nx, ny);
        if (blocked[ni] || parent[ni] >= 0) continue;
        parent[ni] = static_cast<std::int64_t>(cur);
        q.push(ni);
      }
    }
  }
  if (parent[ti] < 0) return std::nullopt;
  std::vector<std::pair<std::size_t, std::size_t>> path;
  for (std::size_t cur = ti;; cur = static_cast<std::size_t>(parent[cur])) {
    path.emplace_back(cur % W, cur / W);
    if (cur == si) break;
  }
  std::reverse(path.begin(), path.end());
  return path;
}

inline std::pair<std::size_t, std::size_t> randomClearCell(
    std::mt19937& rng, const Grid& g, const std::vector<std::uint8_t>& blocked) {
  std::uniform_int_distribution<std::size_t> dx(0, g.width() - 1), dy(0, g.height() - 1);
  while (true) {
    const std::size_t x = dx(rng), y = dy(rng);
    if (!blocked[g.index(x, y)]) return {x, y};
  }
}

// A safe trajectory between two random clear cells, oracle-verified.
// Waypoints are taken every 4th cell (0.2 m < max_segment_length),
// densified if the sparser chords clip a corner.
inline std::optional<Trajectory> makeSafeTrajectory(std::mt19937& rng, const Grid& g,
                                                    const std::vector<std::uint8_t>& blocked,
                                                    const Params& p) {
  for (int attempt = 0; attempt < 30; ++attempt) {
    const auto s = randomClearCell(rng, g, blocked);
    const auto t = randomClearCell(rng, g, blocked);
    const auto cells = cellPath(g, blocked, s, t);
    if (!cells || cells->size() < 20) continue;
    for (std::size_t stride : {4u, 2u, 1u}) {
      Trajectory traj;
      for (std::size_t i = 0; i < cells->size(); i += stride) {
        traj.push_back(g.cellCenter((*cells)[i].first, (*cells)[i].second));
      }
      const auto back = cells->back();
      const Point last = g.cellCenter(back.first, back.second);
      if (traj.back().x != last.x || traj.back().y != last.y) traj.push_back(last);
      if (referenceSafe(g, traj, p)) return traj;
    }
  }
  return std::nullopt;
}

// ------------------------------------------------- hallucination cases

struct Case {
  std::string label;   // "safe" or the hallucination class
  bool safe;           // ground truth
  Trajectory traj;
};

// Straight line as a waypoint list (0.2 m pitch).
inline Trajectory line(Point a, Point b) {
  const double len = std::hypot(b.x - a.x, b.y - a.y);
  const auto n = std::max<std::size_t>(1, static_cast<std::size_t>(std::ceil(len / 0.2)));
  Trajectory t;
  for (std::size_t i = 0; i <= n; ++i) {
    const double s = static_cast<double>(i) / static_cast<double>(n);
    t.push_back({a.x + s * (b.x - a.x), a.y + s * (b.y - a.y)});
  }
  return t;
}

inline Point randomLethalCellCenter(std::mt19937& rng, const Grid& g) {
  std::uniform_int_distribution<std::size_t> dx(4, g.width() - 5), dy(4, g.height() - 5);
  while (true) {
    const std::size_t x = dx(rng), y = dy(rng);
    if (g.cost(x, y) == verifier::kLethal) return g.cellCenter(x, y);
  }
}

// Each generator returns nullopt when this map/rng draw cannot produce
// the class; the harness retries with fresh randomness. Every emitted
// unsafe case is confirmed unsafe by the oracle.
inline std::optional<Case> makeUnsafe(const std::string& cls, std::mt19937& rng,
                                      const MapCase& mc,
                                      const std::vector<std::uint8_t>& blocked,
                                      const Params& p) {
  const Grid& g = mc.grid;
  auto emit = [&](Trajectory t) -> std::optional<Case> {
    if (referenceSafe(g, t, p)) return std::nullopt;  // construction fluke
    return Case{cls, false, std::move(t)};
  };

  if (cls == "goal_in_wall") {
    auto base = makeSafeTrajectory(rng, g, blocked, p);
    if (!base) return std::nullopt;
    const Point wall = randomLethalCellCenter(rng, g);
    // Walk toward the wall goal in continuous steps so only the goal
    // placement is wrong (mimics a hallucinated goal, not a teleport).
    Trajectory t = *base;
    Trajectory approach = line(t.back(), wall);
    t.insert(t.end(), approach.begin() + 1, approach.end());
    return emit(std::move(t));
  }
  if (cls == "wall_through") {
    for (int i = 0; i < 50; ++i) {
      const auto a = randomClearCell(rng, g, blocked);
      const auto b = randomClearCell(rng, g, blocked);
      Trajectory t = line(g.cellCenter(a.first, a.second), g.cellCenter(b.first, b.second));
      if (!referenceSafe(g, t, p)) return Case{cls, false, std::move(t)};
    }
    return std::nullopt;
  }
  if (cls == "off_map") {
    auto base = makeSafeTrajectory(rng, g, blocked, p);
    if (!base) return std::nullopt;
    Trajectory t = *base;
    t[t.size() / 2] = {-0.5, -0.5};
    return emit(std::move(t));
  }
  if (cls == "teleport") {
    auto base = makeSafeTrajectory(rng, g, blocked, p);
    if (!base) return std::nullopt;
    Trajectory t = *base;
    // Remove waypoints until some gap exceeds 1 m.
    std::size_t i = t.size() / 3;
    while (i + 1 < t.size()) {
      const double gap = std::hypot(t[i + 1].x - t[i].x, t[i + 1].y - t[i].y);
      if (gap > 1.0) break;
      t.erase(t.begin() + static_cast<std::ptrdiff_t>(i) + 1);
    }
    if (i + 1 >= t.size()) return std::nullopt;
    return emit(std::move(t));
  }
  if (cls == "unknown_region") {
    for (int i = 0; i < 20; ++i) {
      const auto a = randomClearCell(rng, g, blocked);
      Trajectory t = line(g.cellCenter(a.first, a.second), mc.unknown_center);
      if (!referenceSafe(g, t, p)) return Case{cls, false, std::move(t)};
    }
    return std::nullopt;
  }
  if (cls == "narrow_gap") {
    // Straight through the too-narrow gap, perpendicular to its wall.
    const Point c = mc.narrow_gap_center;
    const double off = 0.6;
    const Point a = mc.narrow_gap_vertical ? Point{c.x - off, c.y} : Point{c.x, c.y - off};
    const Point b = mc.narrow_gap_vertical ? Point{c.x + off, c.y} : Point{c.x, c.y + off};
    return emit(line(a, b));
  }
  return std::nullopt;
}

}  // namespace scenarios
