// src/verifier_node.cpp
//
// ROS 2 node wrapping the ROS-free verifier core (core/): the gate
// between an LLM planner and Nav2.
//
// Subscriptions
//   costmap        nav_msgs/OccupancyGrid (transient_local; remap to
//                  /global_costmap/costmap)
//   proposed_path  nav_msgs/Path - trajectory proposed by the LLM
//   proposed_goal  geometry_msgs/PoseStamped - goal proposed by the LLM
//
// Publications
//   verified_path  nav_msgs/Path - forwarded only when every check passes
//   verified_goal  geometry_msgs/PoseStamped - forwarded only when safe
//   rejections     std_msgs/String - "<violation> at waypoint <i>" for
//                  every rejected proposal (also logged)
//
// The node holds the latest costmap; proposals arriving before any
// costmap are rejected (fail-safe: nothing is forwarded unverified).
// Frames are compared textually - a proposal whose frame_id differs
// from the costmap's is rejected; TF transformation is future work.
#include <memory>
#include <optional>
#include <string>
#include <utility>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

#include "verifier/verifier.hpp"

namespace {

class VerifierNode : public rclcpp::Node {
public:
  VerifierNode() : Node("llm_safety_verifier") {
    params_.robot_radius = declare_parameter("robot_radius", 0.105);
    params_.max_segment_length = declare_parameter("max_segment_length", 0.5);
    params_.sample_step = declare_parameter("sample_step", 0.02);
    params_.min_turning_radius = declare_parameter("min_turning_radius", 0.0);
    params_.allow_unknown = declare_parameter("allow_unknown", false);
    occupied_threshold_ = static_cast<int>(declare_parameter("occupied_threshold", 65));

    const auto map_qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local();
    costmap_sub_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
        "costmap", map_qos,
        [this](nav_msgs::msg::OccupancyGrid::SharedPtr msg) { onCostmap(*msg); });
    path_sub_ = create_subscription<nav_msgs::msg::Path>(
        "proposed_path", rclcpp::QoS(10),
        [this](nav_msgs::msg::Path::SharedPtr msg) { onPath(*msg); });
    goal_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
        "proposed_goal", rclcpp::QoS(10),
        [this](geometry_msgs::msg::PoseStamped::SharedPtr msg) { onGoal(*msg); });

    verified_path_pub_ = create_publisher<nav_msgs::msg::Path>("verified_path", 10);
    verified_goal_pub_ =
        create_publisher<geometry_msgs::msg::PoseStamped>("verified_goal", 10);
    rejection_pub_ = create_publisher<std_msgs::msg::String>("rejections", 10);

    RCLCPP_INFO(get_logger(),
                "llm_safety_verifier ready (robot_radius=%.3f m, allow_unknown=%s)",
                params_.robot_radius, params_.allow_unknown ? "true" : "false");
  }

private:
  void onCostmap(const nav_msgs::msg::OccupancyGrid& msg) {
    const std::size_t w = msg.info.width;
    const std::size_t h = msg.info.height;
    verifier::Grid grid(w, h, msg.info.resolution,
                        {msg.info.origin.position.x, msg.info.origin.position.y});
    for (std::size_t i = 0; i < w * h && i < msg.data.size(); ++i) {
      const int occ = msg.data[i];
      std::uint8_t cost;
      if (occ < 0) {
        cost = verifier::kUnknown;
      } else if (occ >= occupied_threshold_) {
        cost = verifier::kLethal;
      } else {
        cost = static_cast<std::uint8_t>(occ * 252 / 100);
      }
      grid.data()[i] = cost;
    }
    grid_.emplace(std::move(grid));
    map_frame_ = msg.header.frame_id;
    RCLCPP_INFO_ONCE(get_logger(), "costmap received (%zux%zu @ %.3f m, frame '%s')",
                     w, h, static_cast<double>(msg.info.resolution), map_frame_.c_str());
  }

  void reject(const std::string& what) {
    std_msgs::msg::String out;
    out.data = what;
    rejection_pub_->publish(out);
    RCLCPP_WARN(get_logger(), "rejected: %s", what.c_str());
  }

  // Returns the verdict when preconditions hold, or rejects and
  // returns nullopt.
  std::optional<verifier::Verdict> check(const verifier::Trajectory& traj,
                                         const std::string& frame,
                                         const char* kind) {
    if (!grid_) {
      reject(std::string(kind) + " received before any costmap; dropping");
      return std::nullopt;
    }
    if (!frame.empty() && frame != map_frame_) {
      reject(std::string(kind) + " frame '" + frame + "' does not match costmap frame '" +
             map_frame_ + "'");
      return std::nullopt;
    }
    return verifier::verify(*grid_, traj, params_);
  }

  void onPath(const nav_msgs::msg::Path& msg) {
    verifier::Trajectory traj;
    traj.reserve(msg.poses.size());
    for (const auto& p : msg.poses) {
      traj.push_back({p.pose.position.x, p.pose.position.y});
    }
    const auto verdict = check(traj, msg.header.frame_id, "path");
    if (!verdict) return;
    if (verdict->safe) {
      verified_path_pub_->publish(msg);
      RCLCPP_INFO(get_logger(), "path verified (%zu waypoints)", traj.size());
    } else {
      reject("path: " + std::string(verifier::toString(verdict->violation)) +
             " at waypoint " + std::to_string(verdict->waypoint_index));
    }
  }

  void onGoal(const geometry_msgs::msg::PoseStamped& msg) {
    const verifier::Trajectory traj{{msg.pose.position.x, msg.pose.position.y}};
    const auto verdict = check(traj, msg.header.frame_id, "goal");
    if (!verdict) return;
    if (verdict->safe) {
      verified_goal_pub_->publish(msg);
      RCLCPP_INFO(get_logger(), "goal verified (%.2f, %.2f)", msg.pose.position.x,
                  msg.pose.position.y);
    } else {
      reject("goal: " + std::string(verifier::toString(verdict->violation)));
    }
  }

  verifier::Params params_;
  int occupied_threshold_{65};
  std::optional<verifier::Grid> grid_;
  std::string map_frame_;

  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr costmap_sub_;
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr verified_path_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr verified_goal_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr rejection_pub_;
};

}  // namespace

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<VerifierNode>());
  rclcpp::shutdown();
  return 0;
}
