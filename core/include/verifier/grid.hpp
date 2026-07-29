// core/include/verifier/grid.hpp
//
// Occupancy grid with a world transform. Cost semantics follow
// nav2_costmap_2d: 0 = free, 1..252 = increasing penalty, 253 =
// inscribed, 254 = lethal, 255 = no information. The verifier treats
// cost >= 254 as untraversable and 255 as unknown space.
#pragma once

#include <cstddef>
#include <cstdint>
#include <vector>

namespace verifier {

inline constexpr std::uint8_t kInscribed = 253;
inline constexpr std::uint8_t kLethal = 254;
inline constexpr std::uint8_t kUnknown = 255;

struct Point {
  double x;
  double y;
};

class Grid {
public:
  Grid(std::size_t width, std::size_t height, double resolution,
       Point origin = {0.0, 0.0}, std::uint8_t fill = 0)
      : width_(width), height_(height), resolution_(resolution), origin_(origin),
        cells_(width * height, fill) {}

  [[nodiscard]] std::size_t width() const noexcept { return width_; }
  [[nodiscard]] std::size_t height() const noexcept { return height_; }
  [[nodiscard]] double resolution() const noexcept { return resolution_; }
  [[nodiscard]] Point origin() const noexcept { return origin_; }

  [[nodiscard]] std::size_t index(std::size_t x, std::size_t y) const noexcept {
    return y * width_ + x;
  }
  [[nodiscard]] bool inBounds(std::size_t x, std::size_t y) const noexcept {
    return x < width_ && y < height_;
  }
  [[nodiscard]] std::uint8_t cost(std::size_t x, std::size_t y) const noexcept {
    return cells_[index(x, y)];
  }
  void setCost(std::size_t x, std::size_t y, std::uint8_t c) noexcept {
    cells_[index(x, y)] = c;
  }

  // World <-> map. worldToMap returns false outside the grid.
  [[nodiscard]] bool worldToMap(Point p, std::size_t& mx, std::size_t& my) const noexcept {
    const double gx = (p.x - origin_.x) / resolution_;
    const double gy = (p.y - origin_.y) / resolution_;
    if (gx < 0.0 || gy < 0.0) return false;
    const auto ix = static_cast<std::size_t>(gx);
    const auto iy = static_cast<std::size_t>(gy);
    if (ix >= width_ || iy >= height_) return false;
    mx = ix;
    my = iy;
    return true;
  }
  [[nodiscard]] Point cellCenter(std::size_t mx, std::size_t my) const noexcept {
    return {origin_.x + (static_cast<double>(mx) + 0.5) * resolution_,
            origin_.y + (static_cast<double>(my) + 0.5) * resolution_};
  }

  [[nodiscard]] const std::vector<std::uint8_t>& data() const noexcept { return cells_; }
  [[nodiscard]] std::vector<std::uint8_t>& data() noexcept { return cells_; }

private:
  std::size_t width_;
  std::size_t height_;
  double resolution_;
  Point origin_;
  std::vector<std::uint8_t> cells_;
};

using Trajectory = std::vector<Point>;

}  // namespace verifier
