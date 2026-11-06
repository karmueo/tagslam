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
#include <unistd.h>

#include <chrono>
#include <filesystem>
#include <fstream>
#include <limits>
#include <memory>
#include <rosbag2_cpp/reader.hpp>
#include <string>
#include <tagslam/factor/relative_pose_prior.hpp>
#include <tagslam/tagslam.hpp>
#include <vector>

namespace tagslam
{
using Detection = apriltag_msgs::msg::AprilTagDetection;
using DetectionArray = apriltag_msgs::msg::AprilTagDetectionArray;
using ArrayConstPtr = DetectionArray::ConstSharedPtr;
using Odometry = nav_msgs::msg::Odometry;

class TagSLAMTestAccess
{
public:
  static size_t count(TagSLAM & node, const std::vector<ArrayConstPtr> & arrays)
  {
    std::vector<ArrayConstPtr> remapped;
    node.remapAndSquash(
      rclcpp::Time(arrays.front()->header.stamp).nanoseconds(), &remapped,
      arrays);
    return node.countUsableTags(remapped);
  }

  static void process(
    TagSLAM & node, const std::vector<ArrayConstPtr> & arrays,
    const std::vector<Odometry::ConstSharedPtr> & odom = {})
  {
    node.processTagsAndOdom(arrays, odom);
  }

  static void remap(TagSLAM & node, int source, int target)
  {
    node.tagRemap_[source].emplace_back(
      target, "cam1", 0, std::numeric_limits<uint64_t>::max());
  }

  static size_t tags(const TagSLAM & node) { return node.tagMap_.size(); }
  static size_t factors(const TagSLAM & node)
  {
    return node.graph_->getFactors().size();
  }
  static size_t times(const TagSLAM & node) { return node.times_.size(); }
  static size_t odomProcessors(const TagSLAM & node)
  {
    return node.odomProcessors_.size();
  }
  static size_t accepted(const TagSLAM & node) { return node.acceptedFrames_; }
  static size_t skipped(const TagSLAM & node) { return node.skippedFrames_; }
  static const Graph * graph(const TagSLAM & node) { return node.graph_.get(); }
  static void disableAmnesia(TagSLAM & node) { node.amnesia_ = false; }
  static void enableFakeOdom(TagSLAM & node) { node.useFakeOdom_ = true; }
  static bool containsTag(const TagSLAM & node, int id)
  {
    return node.tagMap_.count(id) != 0;
  }
  static std::string ackTopic(const TagSLAM & node)
  {
    return node.ackPub_->get_topic_name();
  }
  static size_t ackSubscribers(const TagSLAM & node)
  {
    return node.ackPub_->get_subscription_count();
  }
  static RelativePosePriorFactorConstPtr odomFactor(
    const TagSLAM & node, uint64_t t)
  {
    for (const auto & vertex : node.graph_->getFactors()) {
      auto factor = std::dynamic_pointer_cast<const factor::RelativePosePrior>(
        node.graph_->getVertex(vertex));
      if (
        factor && factor->getTime() == t &&
        factor->getName() == Graph::body_name("rig")) {
        return factor;
      }
    }
    return nullptr;
  }
};

namespace
{
const char * kCameras = R"(
cam0:
  camera_model: pinhole
  intrinsics: [675.0, 675.0, 960.0, 540.0]
  distortion_model: fisheye
  distortion_coeffs: [0.0, 0.0, 0.0, 0.0]
  resolution: [1920, 1080]
  image_topic: /cam0/image_raw
  tag_topic: /cam0/tags
  rig_body: rig
cam1:
  camera_model: pinhole
  intrinsics: [675.0, 675.0, 960.0, 540.0]
  distortion_model: fisheye
  distortion_coeffs: [0.0, 0.0, 0.0, 0.0]
  resolution: [1920, 1080]
  image_topic: /cam1/image_raw
  tag_topic: /cam1/tags
  rig_body: rig
)";

const char * kCameraPoses = R"(
cam0:
  pose:
    position: {x: 0.0, y: 0.0, z: 0.0}
    rotation: {x: 0.0, y: 0.0, z: 0.0}
    position_noise: {x: 0.0001, y: 0.0001, z: 0.0001}
    rotation_noise: {x: 0.0001, y: 0.0001, z: 0.0001}
cam1:
  pose:
    position: {x: 0.1, y: 0.0, z: 0.0}
    rotation: {x: 0.0, y: 0.0, z: 0.0}
    position_noise: {x: 0.0001, y: 0.0001, z: 0.0001}
    rotation_noise: {x: 0.0001, y: 0.0001, z: 0.0001}
)";

const char * kConfig = R"(
tagslam_parameters:
  optimizer_mode: fast
  minimum_tag_area: 2000
max_hamming_distance: 2
amnesia: true
default_body: environment
squash:
  - {camera: cam0, tags: [20]}
body_defaults:
  position_noise: 0.05
  rotation_noise: 0.01
bodies:
  - environment:
      is_static: true
      type: simple
      default_tag_size: 0.144
      ignore_tags: [9]
      pose: &origin
        position: {x: 0.0, y: 0.0, z: 0.0}
        rotation: {x: 0.0, y: 0.0, z: 0.0}
        position_noise: {x: 0.0001, y: 0.0001, z: 0.0001}
        rotation_noise: {x: 0.0001, y: 0.0001, z: 0.0001}
      tags:
        - {id: 0, pose: *origin}
        - {id: 1}
        - {id: 2}
  - rig:
      is_static: false
      type: simple
      odom_frame_id: rig
)";

Detection detection(
  int id, double x = 700.0, double side = 80.0, int hamming = 0)
{
  Detection d;
  d.id = id;
  d.hamming = hamming;
  d.corners[0].x = x;
  d.corners[0].y = 400.0;
  d.corners[1].x = x + side;
  d.corners[1].y = 400.0;
  d.corners[2].x = x + side;
  d.corners[2].y = 400.0 + side;
  d.corners[3].x = x;
  d.corners[3].y = 400.0 + side;
  return d;
}

ArrayConstPtr array(int second, std::initializer_list<Detection> detections)
{
  auto msg = std::make_shared<DetectionArray>();
  msg->header.stamp.sec = second;
  msg->detections = detections;
  return msg;
}

class MinimumTagsTest : public ::testing::Test
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

  void SetUp() override
  {
    static int sequence = 0;
    dir_ = std::filesystem::temp_directory_path() /
           ("tagslam_minimum_tags_" + std::to_string(getpid()) + "_" +
            std::to_string(++sequence));
    std::filesystem::create_directories(dir_);
    std::ofstream(dir_ / "cameras.yaml") << kCameras;
    std::ofstream(dir_ / "camera_poses.yaml") << kCameraPoses;
    std::ofstream(dir_ / "tagslam.yaml") << kConfig;
  }

  void TearDown() override
  {
    node_.reset();
    std::filesystem::remove_all(dir_);
  }

  rclcpp::NodeOptions options() const
  {
    rclcpp::NodeOptions opts;
    opts.append_parameter_override("cameras", (dir_ / "cameras.yaml").string());
    opts.append_parameter_override(
      "camera_poses", (dir_ / "camera_poses.yaml").string());
    opts.append_parameter_override(
      "tagslam_config", (dir_ / "tagslam.yaml").string());
    opts.append_parameter_override("outbag", (dir_ / "out_bag").string());
    opts.append_parameter_override("output_directory", dir_.string());
    return opts;
  }

  std::filesystem::path dir_;
  std::shared_ptr<TagSLAM> node_;
};

TEST_F(MinimumTagsTest, StartupParameterIsReadOnlyAndRejectsNegative)
{
  node_ = std::make_shared<TagSLAM>(options());
  EXPECT_EQ(node_->get_parameter("minimum_tags_for_pose").as_int(), 3);
  EXPECT_FALSE(
    node_->set_parameter(rclcpp::Parameter("minimum_tags_for_pose", 2))
      .successful);
  auto bad = options();
  bad.append_parameter_override("minimum_tags_for_pose", -1);
  EXPECT_THROW(std::make_shared<TagSLAM>(bad), std::invalid_argument);
}

TEST_F(MinimumTagsTest, CountsOnlyUsableRemappedDistinctIdsWithoutMutatingMap)
{
  node_ = std::make_shared<TagSLAM>(options());
  TagSLAMTestAccess::remap(*node_, 30, 1);
  const auto beforeTags = TagSLAMTestAccess::tags(*node_);
  const auto beforeFactors = TagSLAMTestAccess::factors(*node_);
  const auto left = array(
    1, {detection(0), detection(1), detection(1), detection(9), detection(20),
        detection(40, 700, 5), detection(41, 700, 80, 3)});
  const auto right = array(1, {detection(0), detection(30), detection(2)});
  EXPECT_EQ(TagSLAMTestAccess::count(*node_, {left, right}), 3U);
  EXPECT_EQ(TagSLAMTestAccess::tags(*node_), beforeTags);
  EXPECT_EQ(TagSLAMTestAccess::factors(*node_), beforeFactors);
}

TEST_F(MinimumTagsTest, SkipsLowCountsAndContinuesAtThreshold)
{
  node_ = std::make_shared<TagSLAM>(options());
  const auto initialTags = TagSLAMTestAccess::tags(*node_);
  const auto initialFactors = TagSLAMTestAccess::factors(*node_);
  auto odom = std::make_shared<Odometry>();
  odom->child_frame_id = "rig";
  odom->pose.pose.orientation.w = 1.0;
  TagSLAMTestAccess::process(*node_, {array(1, {}), array(1, {})}, {odom});
  TagSLAMTestAccess::process(
    *node_, {array(2, {detection(10)}), array(2, {})}, {odom});
  TagSLAMTestAccess::process(
    *node_, {array(3, {detection(10)}), array(3, {detection(11)})}, {odom});
  EXPECT_EQ(node_->getNumberOfFrames(), 3U);
  EXPECT_EQ(TagSLAMTestAccess::skipped(*node_), 3U);
  EXPECT_EQ(TagSLAMTestAccess::times(*node_), 0U);
  EXPECT_EQ(TagSLAMTestAccess::odomProcessors(*node_), 0U);
  EXPECT_EQ(TagSLAMTestAccess::tags(*node_), initialTags);
  EXPECT_EQ(TagSLAMTestAccess::factors(*node_), initialFactors);
  TagSLAMTestAccess::process(
    *node_, {array(4, {detection(0, 700), detection(1, 820)}),
             array(4, {detection(2, 940)})});
  EXPECT_EQ(TagSLAMTestAccess::accepted(*node_), 1U);
  EXPECT_EQ(TagSLAMTestAccess::times(*node_), 1U);
  const auto * graph = TagSLAMTestAccess::graph(*node_);
  const auto tags = TagSLAMTestAccess::tags(*node_);
  const auto factors = TagSLAMTestAccess::factors(*node_);
  TagSLAMTestAccess::process(
    *node_, {array(5, {detection(22)}), array(5, {})}, {odom});
  EXPECT_EQ(TagSLAMTestAccess::graph(*node_), graph);
  EXPECT_EQ(TagSLAMTestAccess::times(*node_), 1U);
  EXPECT_EQ(TagSLAMTestAccess::tags(*node_), tags);
  EXPECT_EQ(TagSLAMTestAccess::factors(*node_), factors);
  EXPECT_EQ(TagSLAMTestAccess::odomProcessors(*node_), 0U);
  TagSLAMTestAccess::process(
    *node_, {array(6, {detection(0, 700), detection(1, 820)}),
             array(6, {detection(2, 940)})});
  EXPECT_EQ(node_->getNumberOfFrames(), 6U);
  EXPECT_EQ(TagSLAMTestAccess::accepted(*node_), 2U);
  EXPECT_EQ(TagSLAMTestAccess::skipped(*node_), 4U);
}

TEST_F(MinimumTagsTest, ZeroThresholdRestoresProcessingOfEmptyFrames)
{
  auto opts = options();
  opts.append_parameter_override("minimum_tags_for_pose", 0);
  node_ = std::make_shared<TagSLAM>(opts);
  TagSLAMTestAccess::process(*node_, {array(1, {}), array(1, {})});
  EXPECT_EQ(TagSLAMTestAccess::accepted(*node_), 1U);
  EXPECT_EQ(TagSLAMTestAccess::skipped(*node_), 0U);
  EXPECT_EQ(TagSLAMTestAccess::times(*node_), 1U);
}

TEST_F(MinimumTagsTest, MissingDefaultBodyRejectsUnknownIds)
{
  std::string config = kConfig;
  const std::string defaultBody = "default_body: environment\n";
  config.erase(config.find(defaultBody), defaultBody.size());
  std::ofstream(dir_ / "tagslam.yaml") << config;
  node_ = std::make_shared<TagSLAM>(options());
  EXPECT_EQ(
    TagSLAMTestAccess::count(
      *node_, {array(1, {detection(0), detection(40)}),
               array(1, {detection(1), detection(41)})}),
    2U);
  EXPECT_EQ(
    TagSLAMTestAccess::count(
      *node_, {array(1, {detection(0), detection(1)}),
               array(1, {detection(2), detection(42)})}),
    3U);
}

TEST_F(MinimumTagsTest, NewTagsCanInitializeAfterPassingThreshold)
{
  node_ = std::make_shared<TagSLAM>(options());
  TagSLAMTestAccess::disableAmnesia(*node_);
  TagSLAMTestAccess::process(
    *node_, {array(1, {detection(0, 700), detection(10, 820)}),
             array(1, {detection(11, 940)})});
  EXPECT_EQ(TagSLAMTestAccess::accepted(*node_), 1U);
  EXPECT_TRUE(TagSLAMTestAccess::containsTag(*node_, 10));
  EXPECT_TRUE(TagSLAMTestAccess::containsTag(*node_, 11));
}

TEST_F(MinimumTagsTest, RealOdometrySpansOnlyAcceptedFrames)
{
  node_ = std::make_shared<TagSLAM>(options());
  TagSLAMTestAccess::disableAmnesia(*node_);
  auto odom = std::make_shared<Odometry>();
  odom->child_frame_id = "rig";
  odom->pose.pose.orientation.w = 1.0;
  TagSLAMTestAccess::process(
    *node_,
    {array(1, {detection(0, 700), detection(1, 820)}),
     array(1, {detection(2, 940)})},
    {odom});
  ASSERT_EQ(TagSLAMTestAccess::odomProcessors(*node_), 1U);
  const auto factors = TagSLAMTestAccess::factors(*node_);
  odom->pose.pose.position.x = 100.0;
  TagSLAMTestAccess::process(
    *node_, {array(2, {detection(0)}), array(2, {})}, {odom});
  EXPECT_EQ(TagSLAMTestAccess::factors(*node_), factors);
  EXPECT_FALSE(TagSLAMTestAccess::graph(*node_)->hasPose(
    2000000000ULL, Graph::body_name("rig")));
  odom->pose.pose.position.x = 0.1;
  TagSLAMTestAccess::process(
    *node_,
    {array(3, {detection(0, 700), detection(1, 820)}),
     array(3, {detection(2, 940)})},
    {odom});
  const auto factor = TagSLAMTestAccess::odomFactor(*node_, 3000000000ULL);
  ASSERT_NE(factor, nullptr);
  EXPECT_EQ(factor->getPreviousTime(), 1000000000ULL);
  EXPECT_NEAR(
    factor->getPoseWithNoise().getPose().translation().x(), 0.1, 1e-9);
}

TEST_F(MinimumTagsTest, FakeOdometrySpansOnlyAcceptedFrames)
{
  std::string config = kConfig;
  config +=
    "      fake_odom_translation_noise: 0.1\n"
    "      fake_odom_rotation_noise: 0.1\n";
  std::ofstream(dir_ / "tagslam.yaml") << config;
  node_ = std::make_shared<TagSLAM>(options());
  TagSLAMTestAccess::disableAmnesia(*node_);
  TagSLAMTestAccess::process(
    *node_, {array(1, {detection(0, 700), detection(1, 820)}),
             array(1, {detection(2, 940)})});
  const auto factors = TagSLAMTestAccess::factors(*node_);
  TagSLAMTestAccess::process(*node_, {array(2, {detection(0)}), array(2, {})});
  EXPECT_EQ(TagSLAMTestAccess::factors(*node_), factors);
  EXPECT_FALSE(TagSLAMTestAccess::graph(*node_)->hasPose(
    2000000000ULL, Graph::body_name("rig")));
  TagSLAMTestAccess::process(
    *node_, {array(3, {detection(0, 700), detection(1, 820)}),
             array(3, {detection(2, 940)})});
  const auto factor = TagSLAMTestAccess::odomFactor(*node_, 3000000000ULL);
  ASSERT_NE(factor, nullptr);
  EXPECT_EQ(factor->getPreviousTime(), 1000000000ULL);
  EXPECT_NEAR(
    factor->getPoseWithNoise().getPose().translation().norm(), 0.0, 1e-9);
}

TEST_F(MinimumTagsTest, AllSkippedAtFrameLimitWritesReadableEmptyBag)
{
  auto opts = options();
  opts.append_parameter_override("max_number_of_frames", 2);
  opts.append_parameter_override("publish_ack", true);
  node_ = std::make_shared<TagSLAM>(opts);
  auto listener = std::make_shared<rclcpp::Node>("minimum_tags_ack_test");
  std::vector<std_msgs::msg::Header> acknowledgements;
  auto subscriber = listener->create_subscription<std_msgs::msg::Header>(
    TagSLAMTestAccess::ackTopic(*node_), 10,
    [&acknowledgements](std_msgs::msg::Header::ConstSharedPtr msg) {
      acknowledgements.push_back(*msg);
    });
  const auto deadline =
    std::chrono::steady_clock::now() + std::chrono::seconds(5);
  while (TagSLAMTestAccess::ackSubscribers(*node_) == 0 &&
         std::chrono::steady_clock::now() < deadline) {
    rclcpp::spin_some(listener);
    rclcpp::sleep_for(std::chrono::milliseconds(10));
  }
  ASSERT_GT(TagSLAMTestAccess::ackSubscribers(*node_), 0U);
  TagSLAMTestAccess::process(*node_, {array(1, {}), array(1, {})});
  TagSLAMTestAccess::process(*node_, {array(2, {detection(1)}), array(2, {})});
  EXPECT_EQ(node_->getNumberOfFrames(), 2U);
  EXPECT_EQ(TagSLAMTestAccess::times(*node_), 0U);
  while (acknowledgements.size() < 3 &&
         std::chrono::steady_clock::now() < deadline) {
    rclcpp::spin_some(listener);
    rclcpp::sleep_for(std::chrono::milliseconds(10));
  }
  ASSERT_EQ(acknowledgements.size(), 3U);
  EXPECT_EQ(acknowledgements[0].stamp.sec, 1);
  EXPECT_EQ(acknowledgements[1].stamp.sec, 2);
  EXPECT_EQ(acknowledgements[2].stamp.sec, 2);
  EXPECT_EQ(acknowledgements[2].frame_id, "FINISHED!");
  rosbag2_cpp::Reader reader;
  ASSERT_NO_THROW(reader.open((dir_ / "out_bag").string()));
  EXPECT_FALSE(reader.has_next());
  EXPECT_FALSE(reader.get_all_topics_and_types().empty());
}
}  // namespace
}  // namespace tagslam
