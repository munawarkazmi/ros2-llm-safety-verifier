// core/src/verifier.cpp
#include "verifier/verifier.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>

namespace verifier {

namespace {

// Checks the robot footprint (circle of `radius` around world point p)
// against the grid. Returns kNone, kOffMap (footprint leaves the map),
// kCollision, or kUnknownRegion.
Violation footprintCheck(const Grid& grid, Point p, double radius, bool allow_unknown) {
  const double res = grid.resolution();
  std::size_t cx, cy;
  if (!grid.worldToMap(p, cx, cy)) return Violation::kOffMap;

  const auto r_cells = static_cast<std::int64_t>(std::ceil(radius / res));
  for (std::int64_t dy = -r_cells; dy <= r_cells; ++dy) {
    for (std::int64_t dx = -r_cells; dx <= r_cells; ++dx) {
      const auto x = static_cast<std::int64_t>(cx) + dx;
      const auto y = static_cast<std::int64_t>(cy) + dy;
      // Distance from p to the nearest point of this cell; the cell
      // only matters if it intersects the footprint circle. This must
      // come before the bounds check: the scanned bounding box is
      // larger than the circle, and only cells the circle truly
      // touches may declare the footprint off-map.
      const double cx_w = grid.origin().x + (static_cast<double>(x) + 0.5) * res;
      const double cy_w = grid.origin().y + (static_cast<double>(y) + 0.5) * res;
      const double ddx = std::max(0.0, std::abs(cx_w - p.x) - res / 2.0);
      const double ddy = std::max(0.0, std::abs(cy_w - p.y) - res / 2.0);
      if (ddx * ddx + ddy * ddy > radius * radius) continue;

      if (x < 0 || y < 0 || !grid.inBounds(static_cast<std::size_t>(x),
                                           static_cast<std::size_t>(y))) {
        return Violation::kOffMap;  // footprint truly pokes off the map
      }
      const std::uint8_t cost = grid.cost(static_cast<std::size_t>(x),
                                          static_cast<std::size_t>(y));
      if (cost == kUnknown) {
        if (!allow_unknown) return Violation::kUnknownRegion;
      } else if (cost >= kLethal) {
        return Violation::kCollision;
      }
    }
  }
  return Violation::kNone;
}

// Radius of the circle through three points; infinity for collinear.
double circumradius(Point a, Point b, Point c) {
  const double abx = b.x - a.x, aby = b.y - a.y;
  const double acx = c.x - a.x, acy = c.y - a.y;
  const double cross = abx * acy - aby * acx;
  if (std::abs(cross) < 1e-12) return std::numeric_limits<double>::infinity();
  const double ab = std::hypot(abx, aby);
  const double bc = std::hypot(c.x - b.x, c.y - b.y);
  const double ac = std::hypot(acx, acy);
  return (ab * bc * ac) / (2.0 * std::abs(cross));
}

}  // namespace

Verdict verify(const Grid& grid, const Trajectory& traj, const Params& params) {
  if (traj.empty()) return {false, Violation::kEmptyTrajectory, 0};

  // Waypoints on the map.
  for (std::size_t i = 0; i < traj.size(); ++i) {
    std::size_t mx, my;
    if (!grid.worldToMap(traj[i], mx, my)) return {false, Violation::kOffMap, i};
  }

  // Continuity.
  for (std::size_t i = 1; i < traj.size(); ++i) {
    const double len = std::hypot(traj[i].x - traj[i - 1].x, traj[i].y - traj[i - 1].y);
    if (len > params.max_segment_length) return {false, Violation::kDiscontinuity, i};
  }

  // Curvature (car-like platforms only).
  if (params.min_turning_radius > 0.0) {
    for (std::size_t i = 2; i < traj.size(); ++i) {
      const double r = circumradius(traj[i - 2], traj[i - 1], traj[i]);
      if (r < params.min_turning_radius) return {false, Violation::kCurvature, i - 1};
    }
  }

  // Swept footprint collision along every segment (and both endpoints).
  for (std::size_t i = 0; i < traj.size(); ++i) {
    if (i == 0) {
      const Violation v = footprintCheck(grid, traj[0], params.robot_radius,
                                         params.allow_unknown);
      if (v != Violation::kNone) return {false, v, 0};
      continue;
    }
    const Point a = traj[i - 1];
    const Point b = traj[i];
    const double len = std::hypot(b.x - a.x, b.y - a.y);
    const auto steps = static_cast<std::size_t>(std::ceil(len / params.sample_step));
    for (std::size_t s = 1; s <= steps; ++s) {
      const double t = static_cast<double>(s) / static_cast<double>(steps);
      const Point p{a.x + t * (b.x - a.x), a.y + t * (b.y - a.y)};
      const Violation v = footprintCheck(grid, p, params.robot_radius,
                                         params.allow_unknown);
      if (v != Violation::kNone) return {false, v, i};
    }
  }

  return {true, Violation::kNone, 0};
}

}  // namespace verifier
