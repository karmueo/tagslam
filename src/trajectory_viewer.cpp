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

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <tagslam/trajectory_viewer.hpp>
#include <utility>

namespace tagslam
{
namespace
{
constexpr char kOdomTopic[] = "/tagslam/odom/body_rig";
constexpr char kMarkerTopic[] = "/tagslam/trajectory/body_rig";
}  // namespace

TrajectoryViewer::TrajectoryViewer(const rclcpp::NodeOptions & options)
: Node("tagslam_trajectory_viewer", options),
  trajectory_(declare_parameter<double>("max_gap", 0.03))
{
  playerNode_ =
    declare_parameter<std::string>("player_node", "tagslam_bag_player");
  rvizNode_ = declare_parameter<std::string>("rviz_node", "tagslam_view_rviz");
  const double timeout = declare_parameter<double>("startup_timeout", 30.0);
  if (!std::isfinite(timeout) || timeout <= 0) {
    throw std::invalid_argument("startup_timeout must be finite and positive");
  }
  markerPub_ = create_publisher<visualization_msgs::msg::MarkerArray>(
    kMarkerTopic, rclcpp::QoS(1).reliable().transient_local());
  odomSub_ = create_subscription<nav_msgs::msg::Odometry>(
    kOdomTopic, rclcpp::QoS(100).reliable(),
    std::bind(&TrajectoryViewer::onOdometry, this, std::placeholders::_1));
  resumeClient_ = create_client<rosbag2_interfaces::srv::Resume>(
    "/" + playerNode_ + "/resume");
  deadline_ = std::chrono::steady_clock::now() +
              std::chrono::duration_cast<std::chrono::steady_clock::duration>(
                std::chrono::duration<double>(timeout));
  readinessTimer_ = create_wall_timer(
    std::chrono::milliseconds(100),
    std::bind(&TrajectoryViewer::checkReadiness, this));
}

void TrajectoryViewer::onOdometry(
  const nav_msgs::msg::Odometry::ConstSharedPtr & message)
{
  const auto result = trajectory_.add(*message);
  if (result == SegmentedTrajectory::Result::invalid) {
    RCLCPP_WARN(
      get_logger(), "ignoring odometry with invalid frame, time or position");
  } else if (result != SegmentedTrajectory::Result::duplicate) {
    markerPub_->publish(trajectory_.snapshot());
  }
}

bool TrajectoryViewer::hasEndpoint(
  const std::vector<rclcpp::TopicEndpointInfo> & endpoints,
  const std::string & node_name) const
{
  return std::any_of(
    endpoints.begin(), endpoints.end(),
    [&](const auto & endpoint) { return endpoint.node_name() == node_name; });
}

void TrajectoryViewer::checkReadiness()
{
  if (std::chrono::steady_clock::now() >= deadline_) {
    fail("timed out waiting for RViz and the paused bag player");
    return;
  }
  if (resumeRequested_) {
    return;
  }
  const bool rvizReady =
    hasEndpoint(get_subscriptions_info_by_topic(kMarkerTopic), rvizNode_) &&
    hasEndpoint(get_subscriptions_info_by_topic("/clock"), rvizNode_);
  const bool playerReady =
    hasEndpoint(get_publishers_info_by_topic(kOdomTopic), playerNode_) &&
    hasEndpoint(get_publishers_info_by_topic("/tf"), playerNode_) &&
    resumeClient_->service_is_ready();
  if (
    !rvizReady || !playerReady || markerPub_->get_subscription_count() == 0 ||
    odomSub_->get_publisher_count() == 0 ||
    get_subscriptions_info_by_topic("/tf").empty()) {
    return;
  }
  resumeRequested_ = true;
  RCLCPP_INFO(get_logger(), "RViz and bag player are ready; resuming playback");
  auto request = std::make_shared<rosbag2_interfaces::srv::Resume::Request>();
  resumeClient_->async_send_request(
    request,
    [this](
      rclcpp::Client<rosbag2_interfaces::srv::Resume>::SharedFuture future) {
      try {
        (void)future.get();
        readinessTimer_->cancel();
      } catch (const std::exception & error) {
        fail(std::string("failed to resume the bag player: ") + error.what());
      }
    });
}

void TrajectoryViewer::fail(const std::string & message)
{
  RCLCPP_ERROR(get_logger(), "%s", message.c_str());
  failed_ = true;
  readinessTimer_->cancel();
  rclcpp::shutdown();
}
}  // namespace tagslam
