// core/tests/test_verifier.cpp
//
// Unit tests for the trajectory verifier plus a fuzz invariant against
// the independent reference checker: whenever the verifier calls a
// trajectory safe, the 4x-finer-sampled oracle must agree. (The
// converse is not required - the oracle may be stricter - but is also
// fuzzed with constructed cases in the eval harness.)
#include <cstdio>
#include <random>

#include "../eval/scenarios.hpp"
#include "verifier/grid.hpp"
#include "verifier/verifier.hpp"

using verifier::Grid;
using verifier::Params;
using verifier::Point;
using verifier::Trajectory;
using verifier::Violation;

namespace {

int g_failures = 0;

#define CHECK(cond, ...)                                        \
  do {                                                          \
    if (!(cond)) {                                              \
      ++g_failures;                                             \
      std::printf("FAIL %s:%d  ", __FILE__, __LINE__);          \
      std::printf(__VA_ARGS__);                                 \
      std::printf("\n");                                        \
    }                                                           \
  } while (0)

// 5 m x 5 m empty room with 0.1 m walls all around.
Grid emptyRoom() {
  Grid g(100, 100, 0.05, {0.0, 0.0}, 0);
  for (std::size_t i = 0; i < 100; ++i) {
    for (std::size_t b = 0; b < 2; ++b) {
      g.setCost(i, b, verifier::kLethal);
      g.setCost(i, 99 - b, verifier::kLethal);
      g.setCost(b, i, verifier::kLethal);
      g.setCost(99 - b, i, verifier::kLethal);
    }
  }
  return g;
}

Trajectory straight(Point a, Point b, double pitch = 0.1) {
  const double len = std::hypot(b.x - a.x, b.y - a.y);
  const auto n = std::max<std::size_t>(1, static_cast<std::size_t>(std::ceil(len / pitch)));
  Trajectory t;
  for (std::size_t i = 0; i <= n; ++i) {
    const double s = static_cast<double>(i) / static_cast<double>(n);
    t.push_back({a.x + s * (b.x - a.x), a.y + s * (b.y - a.y)});
  }
  return t;
}

void testUnits() {
  const Grid g = emptyRoom();
  const Params p;

  CHECK(!verifier::verify(g, {}, p).safe, "empty trajectory must fail");
  CHECK(verifier::verify(g, {}, p).violation == Violation::kEmptyTrajectory,
        "empty trajectory reason");

  // Clear straight line through the middle.
  CHECK(verifier::verify(g, straight({1.0, 2.5}, {4.0, 2.5}), p).safe,
        "clear straight line must pass");

  // Goal inside the wall.
  {
    const auto v = verifier::verify(g, straight({1.0, 2.5}, {4.99, 2.5}), p);
    CHECK(!v.safe && v.violation == Violation::kCollision, "goal in wall -> collision");
  }

  // Hugging the wall closer than the footprint radius.
  {
    const auto v = verifier::verify(g, straight({1.0, 0.15}, {4.0, 0.15}), p);
    CHECK(!v.safe && v.violation == Violation::kCollision, "wall graze -> collision");
  }

  // Waypoint off the map.
  {
    Trajectory t = straight({1.0, 2.5}, {4.0, 2.5});
    t[3] = {-1.0, 2.5};
    const auto v = verifier::verify(g, t, p);
    CHECK(!v.safe && v.violation == Violation::kOffMap, "off-map waypoint");
  }

  // Teleport.
  {
    Trajectory t{{1.0, 2.5}, {1.2, 2.5}, {3.5, 2.5}};
    const auto v = verifier::verify(g, t, p);
    CHECK(!v.safe && v.violation == Violation::kDiscontinuity, "teleport");
  }

  // Unknown region blocked by default, passable with allow_unknown.
  {
    Grid gu = emptyRoom();
    for (std::size_t y = 40; y < 60; ++y)
      for (std::size_t x = 40; x < 60; ++x) gu.setCost(x, y, verifier::kUnknown);
    const auto t = straight({1.0, 2.5}, {4.0, 2.5});
    const auto v = verifier::verify(gu, t, p);
    CHECK(!v.safe && v.violation == Violation::kUnknownRegion, "unknown blocked");
    Params allow = p;
    allow.allow_unknown = true;
    CHECK(verifier::verify(gu, t, allow).safe, "unknown allowed when configured");
  }

  // Straight through an interior wall.
  {
    Grid gw = emptyRoom();
    for (std::size_t y = 2; y < 98; ++y) gw.setCost(50, y, verifier::kLethal);
    const auto v = verifier::verify(gw, straight({1.0, 2.5}, {4.0, 2.5}), p);
    CHECK(!v.safe && v.violation == Violation::kCollision, "through-wall line");
  }

  // Curvature: off for differential drive, on for car-like params.
  {
    const Trajectory zigzag{{1.0, 2.5}, {1.2, 2.5}, {1.2, 2.7}, {1.4, 2.7}};
    CHECK(verifier::verify(g, zigzag, p).safe, "diff-drive ignores curvature");
    Params car = p;
    car.min_turning_radius = 0.5;
    const auto v = verifier::verify(g, zigzag, car);
    CHECK(!v.safe && v.violation == Violation::kCurvature, "car-like flags zigzag");
  }
}

// Fuzz invariant: verifier-safe implies oracle-safe (finer sampling).
void testFuzzAgainstOracle() {
  const Params p;
  int safe_seen = 0;
  for (unsigned seed = 1; seed <= 400; ++seed) {
    std::mt19937 rng(seed);
    const auto mc = scenarios::makeMap(rng);
    std::uniform_real_distribution<double> rx(0.2, 11.8), ry(0.2, 8.8);
    std::uniform_int_distribution<int> wp(2, 30);
    std::uniform_real_distribution<double> jump(0.02, 0.45);

    for (int c = 0; c < 12; ++c) {
      // Random walk with bounded steps (so continuity usually holds
      // and collision/unknown checks do the deciding).
      Trajectory t;
      Point cur{rx(rng), ry(rng)};
      t.push_back(cur);
      const int n = wp(rng);
      std::uniform_real_distribution<double> ang(0.0, 6.283185307);
      for (int i = 0; i < n; ++i) {
        const double a = ang(rng), d = jump(rng);
        cur = {cur.x + d * std::cos(a), cur.y + d * std::sin(a)};
        t.push_back(cur);
      }
      const auto verdict = verifier::verify(mc.grid, t, p);
      if (verdict.safe) {
        ++safe_seen;
        CHECK(scenarios::referenceSafe(mc.grid, t, p),
              "seed %u case %d: verifier said safe, oracle disagrees", seed, c);
      }
    }
  }
  // The fuzz must actually exercise the safe branch to mean anything.
  CHECK(safe_seen > 100, "fuzz produced too few safe cases (%d) to be meaningful",
        safe_seen);
  std::printf("fuzz: %d verifier-safe trajectories cross-checked against oracle\n",
              safe_seen);
}

}  // namespace

int main() {
  testUnits();
  testFuzzAgainstOracle();
  if (g_failures == 0) {
    std::printf("all tests passed\n");
    return 0;
  }
  std::printf("%d check(s) failed\n", g_failures);
  return 1;
}
