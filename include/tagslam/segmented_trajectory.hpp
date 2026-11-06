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

#ifndef TAGSLAM__SEGMENTED_TRAJECTORY_HPP_
#define TAGSLAM__SEGMENTED_TRAJECTORY_HPP_

#include <cstdint>
#include <geometry_msgs/msg/point.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <std_msgs/msg/header.hpp>
#include <string>
#include <vector>
#include <visualization_msgs/msg/marker_array.hpp>

namespace tagslam
{
class SegmentedTrajectory
{
public:
  enum class Result { accepted, reset, duplicate, invalid };

  explicit SegmentedTrajectory(double max_gap_seconds);
  Result add(const nav_msgs::msg::Odometry & message);
  visualization_msgs::msg::MarkerArray snapshot() const;
  const std::vector<std::vector<geometry_msgs::msg::Point>> & segments() const
  {
    return segments_;
  }

private:
  long double maxGapNanoseconds_;
  bool hasLast_{false};
  int64_t lastStampNs_{0};
  std_msgs::msg::Header header_;
  std::vector<std::vector<geometry_msgs::msg::Point>> segments_;
};
}  // namespace tagslam

#endif  // TAGSLAM__SEGMENTED_TRAJECTORY_HPP_
