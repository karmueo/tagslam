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

#include <gtest/gtest.h>

#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rosbag2_interfaces/srv/resume.hpp>
#include <rosgraph_msgs/msg/clock.hpp>
#include <tagslam/segmented_trajectory.hpp>
#include <tagslam/trajectory_viewer.hpp>
#include <tf2_msgs/msg/tf_message.hpp>
#include <thread>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

namespace
{
using tagslam::SegmentedTrajectory;
using Marker = visualization_msgs::msg::Marker;
using MarkerArray = visualization_msgs::msg::MarkerArray;
using Odometry = nav_msgs::msg::Odometry;

Odometry odom(int64_t time_ns, double x, const std::string & frame = "map")
{
  Odometry message;
  message.header.stamp.sec = static_cast<int32_t>(time_ns / 1000000000LL);
  message.header.stamp.nanosec = static_cast<uint32_t>(time_ns % 1000000000LL);
  message.header.frame_id = frame;
  message.pose.pose.position.x = x;
  message.pose.pose.position.y = -2.0 * x;
  message.pose.pose.position.z = 3.0 * x;
  return message;
}

TEST(SegmentedTrajectory, ValidatesGap)
{
  EXPECT_THROW(SegmentedTrajectory(0), std::invalid_argument);
  EXPECT_THROW(SegmentedTrajectory(-1), std::invalid_argument);
  EXPECT_THROW(
    SegmentedTrajectory(std::numeric_limits<double>::infinity()),
    std::invalid_argument);
  EXPECT_THROW(
    SegmentedTrajectory(std::numeric_limits<double>::quiet_NaN()),
    std::invalid_argument);
  EXPECT_NO_THROW(SegmentedTrajectory(0.03));
}

TEST(SegmentedTrajectory, SplitsOnlyBeyondThresholdAndKeepsOriginalPoints)
{
  SegmentedTrajectory track(0.03);
  const int64_t t = 1000000000LL;
  EXPECT_EQ(track.add(odom(t, 1.0)), SegmentedTrajectory::Result::accepted);
  EXPECT_EQ(
    track.add(odom(t + 29999999LL, 2.0)),
    SegmentedTrajectory::Result::accepted);
  EXPECT_EQ(
    track.add(odom(t + 59999999LL, 3.0)),
    SegmentedTrajectory::Result::accepted);
  EXPECT_EQ(
    track.add(odom(t + 90000000LL, 4.0)),
    SegmentedTrajectory::Result::accepted);
  ASSERT_EQ(track.segments().size(), 2U);
  EXPECT_EQ(track.segments()[0].size(), 3U);
  EXPECT_EQ(track.segments()[1].size(), 1U);
  const auto markers = track.snapshot().markers;
  ASSERT_EQ(markers.size(), 3U);
  EXPECT_EQ(markers[0].action, Marker::DELETEALL);
  EXPECT_EQ(markers[1].type, Marker::LINE_STRIP);
  EXPECT_EQ(markers[2].type, Marker::POINTS);
  ASSERT_EQ(markers[1].points.size(), 3U);
  EXPECT_DOUBLE_EQ(markers[1].points[0].x, 1.0);
  EXPECT_DOUBLE_EQ(markers[1].points[1].y, -4.0);
  EXPECT_DOUBLE_EQ(markers[1].points[2].z, 9.0);
  EXPECT_DOUBLE_EQ(markers[2].points[0].x, 4.0);
  EXPECT_EQ(markers[1].header.frame_id, "map");
  EXPECT_EQ(markers[2].header.frame_id, "map");
}

TEST(
  SegmentedTrajectory,
  IgnoresInvalidAndDuplicateSamplesAndResetsOnDiscontinuity)
{
  SegmentedTrajectory track(0.03);
  EXPECT_EQ(
    track.add(odom(1000000000LL, 1.0)), SegmentedTrajectory::Result::accepted);
  EXPECT_EQ(
    track.add(odom(1000000000LL, 99.0)),
    SegmentedTrajectory::Result::duplicate);
  EXPECT_EQ(
    track.add(odom(1010000000LL, 2.0, "")),
    SegmentedTrajectory::Result::invalid);
  auto bad = odom(1010000000LL, 2.0);
  bad.pose.pose.position.z = std::numeric_limits<double>::infinity();
  EXPECT_EQ(track.add(bad), SegmentedTrajectory::Result::invalid);
  bad.pose.pose.position.z = std::numeric_limits<double>::quiet_NaN();
  EXPECT_EQ(track.add(bad), SegmentedTrajectory::Result::invalid);
  bad = odom(1010000000LL, 2.0);
  bad.header.stamp.nanosec = 1000000000U;
  EXPECT_EQ(track.add(bad), SegmentedTrajectory::Result::invalid);
  EXPECT_EQ(
    track.add(odom(1020000000LL, 2.0)), SegmentedTrajectory::Result::accepted);
  ASSERT_EQ(track.segments().size(), 1U);
  ASSERT_EQ(track.segments()[0].size(), 2U);
  EXPECT_EQ(
    track.add(odom(500000000LL, 3.0)), SegmentedTrajectory::Result::reset);
  auto snapshot = track.snapshot();
  ASSERT_EQ(snapshot.markers.size(), 2U);
  EXPECT_EQ(snapshot.markers[0].action, Marker::DELETEALL);
  EXPECT_EQ(snapshot.markers[1].type, Marker::POINTS);
  EXPECT_DOUBLE_EQ(snapshot.markers[1].points[0].x, 3.0);
  EXPECT_EQ(
    track.add(odom(500000000LL, 4.0, "new_map")),
    SegmentedTrajectory::Result::reset);
  snapshot = track.snapshot();
  ASSERT_EQ(snapshot.markers.size(), 2U);
  EXPECT_EQ(snapshot.markers[0].action, Marker::DELETEALL);
  EXPECT_EQ(snapshot.markers[1].header.frame_id, "new_map");
  EXPECT_DOUBLE_EQ(snapshot.markers[1].points[0].x, 4.0);
}

class ViewerIntegration : public ::testing::Test
{
protected:
  static void SetUpTestSuite()
  {
    if (!rclcpp::ok()) {
      int argc = 0;
      rclcpp::init(argc, nullptr);
    }
  }
  static void TearDownTestSuite() { rclcpp::shutdown(); }

  template <typename Predicate>
  bool waitFor(Predicate predicate, std::chrono::milliseconds timeout)
  {
    const auto deadline = std::chrono::steady_clock::now() + timeout;
    while (std::chrono::steady_clock::now() < deadline) {
      executor_.spin_some();
      if (predicate()) {
        return true;
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    return false;
  }

  rclcpp::executors::SingleThreadedExecutor executor_;
};

TEST_F(ViewerIntegration, RequiresActualRvizAndRetainsFullTransientSnapshot)
{
  using namespace std::chrono_literals;
  rclcpp::NodeOptions options;
  options.append_parameter_override("player_node", "test_trajectory_player");
  options.append_parameter_override("rviz_node", "test_trajectory_rviz");
  options.append_parameter_override("startup_timeout", 8.0);
  auto viewer = std::make_shared<tagslam::TrajectoryViewer>(options);
  auto player = std::make_shared<rclcpp::Node>("test_trajectory_player");
  auto observer = std::make_shared<rclcpp::Node>("test_trajectory_observer");
  auto odom_pub = player->create_publisher<Odometry>(
    "/tagslam/odom/body_rig", rclcpp::QoS(100).reliable());
  auto tf_pub = player->create_publisher<tf2_msgs::msg::TFMessage>("/tf", 10);
  bool resumed = false;
  auto resume = player->create_service<rosbag2_interfaces::srv::Resume>(
    "/test_trajectory_player/resume",
    [&resumed](
      const std::shared_ptr<rosbag2_interfaces::srv::Resume::Request>,
      std::shared_ptr<rosbag2_interfaces::srv::Resume::Response>) {
      resumed = true;
    });
  auto observer_sub = observer->create_subscription<MarkerArray>(
    "/tagslam/trajectory/body_rig", rclcpp::QoS(1).reliable().transient_local(),
    [](MarkerArray::ConstSharedPtr) {});
  executor_.add_node(viewer);
  executor_.add_node(player);
  executor_.add_node(observer);
  EXPECT_FALSE(waitFor([&resumed]() { return resumed; }, 400ms));

  auto rviz = std::make_shared<rclcpp::Node>("test_trajectory_rviz");
  MarkerArray latest;
  size_t received = 0;
  auto marker_sub = rviz->create_subscription<MarkerArray>(
    "/tagslam/trajectory/body_rig", rclcpp::QoS(1).reliable().transient_local(),
    [&latest, &received](MarkerArray::ConstSharedPtr message) {
      latest = *message;
      ++received;
    });
  auto clock_sub = rviz->create_subscription<rosgraph_msgs::msg::Clock>(
    "/clock", 10, [](rosgraph_msgs::msg::Clock::ConstSharedPtr) {});
  auto tf_sub = rviz->create_subscription<tf2_msgs::msg::TFMessage>(
    "/tf", 10, [](tf2_msgs::msg::TFMessage::ConstSharedPtr) {});
  executor_.add_node(rviz);
  ASSERT_TRUE(waitFor([&resumed]() { return resumed; }, 5s));
  ASSERT_TRUE(waitFor(
    [&odom_pub]() { return odom_pub->get_subscription_count() != 0; }, 5s));
  for (const auto & message :
       {odom(1000000000LL, 1), odom(1020000000LL, 2), odom(1080000000LL, 3)}) {
    const auto previous = received;
    odom_pub->publish(message);
    ASSERT_TRUE(
      waitFor([&received, previous]() { return received > previous; }, 3s));
  }
  ASSERT_EQ(latest.markers.size(), 3U);
  EXPECT_EQ(latest.markers[0].action, Marker::DELETEALL);
  EXPECT_EQ(latest.markers[1].type, Marker::LINE_STRIP);
  EXPECT_EQ(latest.markers[2].type, Marker::POINTS);

  auto late = std::make_shared<rclcpp::Node>("test_trajectory_late");
  MarkerArray retained;
  bool gotRetained = false;
  auto late_sub = late->create_subscription<MarkerArray>(
    "/tagslam/trajectory/body_rig", rclcpp::QoS(1).reliable().transient_local(),
    [&retained, &gotRetained](MarkerArray::ConstSharedPtr message) {
      retained = *message;
      gotRetained = true;
    });
  executor_.add_node(late);
  ASSERT_TRUE(waitFor([&gotRetained]() { return gotRetained; }, 5s));
  ASSERT_EQ(retained.markers.size(), 3U);
  EXPECT_EQ(retained.markers[1].points.size(), 2U);
  EXPECT_DOUBLE_EQ(retained.markers[2].points[0].x, 3.0);

  const auto previous = received;
  odom_pub->publish(odom(500000000LL, 4));
  ASSERT_TRUE(
    waitFor([&received, previous]() { return received > previous; }, 3s));
  ASSERT_EQ(latest.markers.size(), 2U);
  EXPECT_EQ(latest.markers[0].action, Marker::DELETEALL);
  EXPECT_EQ(latest.markers[1].type, Marker::POINTS);
  EXPECT_DOUBLE_EQ(latest.markers[1].points[0].x, 4.0);
}
}  // namespace
