// Copyright 2026 TagSLAM contributors
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef TAGSLAM__TRAJECTORY_VIEWER_HPP_
#define TAGSLAM__TRAJECTORY_VIEWER_HPP_

#include <chrono>
#include <memory>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rosbag2_interfaces/srv/resume.hpp>
#include <string>
#include <tagslam/segmented_trajectory.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

namespace tagslam
{
class TrajectoryViewer : public rclcpp::Node
{
public:
  explicit TrajectoryViewer(const rclcpp::NodeOptions & options);
  bool failed() const { return failed_; }

private:
  void onOdometry(const nav_msgs::msg::Odometry::ConstSharedPtr & message);
  void checkReadiness();
  void fail(const std::string & message);
  bool hasEndpoint(
    const std::vector<rclcpp::TopicEndpointInfo> & endpoints,
    const std::string & node_name) const;

  SegmentedTrajectory trajectory_;
  std::string playerNode_;
  std::string rvizNode_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odomSub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr markerPub_;
  rclcpp::Client<rosbag2_interfaces::srv::Resume>::SharedPtr resumeClient_;
  rclcpp::TimerBase::SharedPtr readinessTimer_;
  std::chrono::steady_clock::time_point deadline_;
  bool resumeRequested_{false};
  bool failed_{false};
};
}  // namespace tagslam

#endif  // TAGSLAM__TRAJECTORY_VIEWER_HPP_
