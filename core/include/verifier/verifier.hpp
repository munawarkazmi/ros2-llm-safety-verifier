// core/include/verifier/verifier.hpp
//
// Deterministic safety checks for a proposed waypoint trajectory
// against an occupancy grid. Designed as the gate between an LLM
// planner (which may hallucinate) and the Nav2 controller.
//
// Checks, in order, stopping at the first violation:
//   1. Non-empty trajectory.
//   2. Every waypoint on the map (kOffMap).
//   3. Segment continuity: no jump longer than max_segment_length
//      (kDiscontinuity - "teleports" in LLM output).
//   4. Curvature (only when min_turning_radius > 0, i.e. car-like
//      platforms; differential-drive robots can turn in place, so the
//      check is off by default).
//   5. Swept collision: the robot footprint (circle of robot_radius)
//      sampled every sample_step along every segment must stay clear
//      of lethal cells, and of unknown cells unless allow_unknown
//      (kCollision / kUnknownRegion). The final waypoint's footprint
//      check doubles as the goal-in-obstacle test.
#pragma once

#include <cstddef>
#include <string_view>

#include "verifier/grid.hpp"

namespace verifier {

struct Params {
  double robot_radius = 0.105;      // m; TurtleBot3 Burger footprint circle
  double max_segment_length = 0.5;  // m; larger jumps are discontinuities
  double sample_step = 0.02;        // m; sampling pitch along segments
  double min_turning_radius = 0.0;  // m; 0 disables the curvature check
  bool allow_unknown = false;       // treat unknown cells as traversable?
};

enum class Violation {
  kNone,
  kEmptyTrajectory,
  kOffMap,
  kDiscontinuity,
  kCurvature,
  kCollision,
  kUnknownRegion,
};

[[nodiscard]] constexpr std::string_view toString(Violation v) noexcept {
  switch (v) {
    case Violation::kNone: return "none";
    case Violation::kEmptyTrajectory: return "empty_trajectory";
    case Violation::kOffMap: return "off_map";
    case Violation::kDiscontinuity: return "discontinuity";
    case Violation::kCurvature: return "curvature";
    case Violation::kCollision: return "collision";
    case Violation::kUnknownRegion: return "unknown_region";
  }
  return "?";
}

struct Verdict {
  bool safe;
  Violation violation;          // first violation found (kNone when safe)
  std::size_t waypoint_index;   // waypoint/segment where it was found
};

[[nodiscard]] Verdict verify(const Grid& grid, const Trajectory& traj,
                             const Params& params = {});

}  // namespace verifier
