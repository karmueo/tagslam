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

#include <cmath>
#include <stdexcept>
#include <tagslam/segmented_trajectory.hpp>
#include <utility>

namespace tagslam
{
SegmentedTrajectory::SegmentedTrajectory(double max_gap_seconds)
: maxGapNanoseconds_(
    std::round(static_cast<long double>(max_gap_seconds) * 1e9L))
{
  if (!std::isfinite(max_gap_seconds) || max_gap_seconds <= 0) {
    throw std::invalid_argument("max_gap must be finite and positive");
  }
}

SegmentedTrajectory::Result SegmentedTrajectory::add(
  const nav_msgs::msg::Odometry & message)
{
  const auto & point = message.pose.pose.position;
  if (
    message.header.frame_id.empty() ||
    message.header.stamp.nanosec >= 1000000000 || !std::isfinite(point.x) ||
    !std::isfinite(point.y) || !std::isfinite(point.z)) {
    return Result::invalid;
  }
  const int64_t stamp =
    static_cast<int64_t>(message.header.stamp.sec) * 1000000000LL +
    message.header.stamp.nanosec;
  const bool reset = hasLast_ && (message.header.frame_id != header_.frame_id ||
                                  stamp < lastStampNs_);
  if (hasLast_ && !reset && stamp == lastStampNs_) {
    return Result::duplicate;
  }
  if (reset) {
    segments_.clear();
    hasLast_ = false;
  }
  const long double gap =
    static_cast<long double>(stamp) - static_cast<long double>(lastStampNs_);
  if (!hasLast_ || gap > maxGapNanoseconds_) {
    segments_.emplace_back();
  }
  segments_.back().push_back(point);
  header_ = message.header;
  lastStampNs_ = stamp;
  hasLast_ = true;
  return reset ? Result::reset : Result::accepted;
}

visualization_msgs::msg::MarkerArray SegmentedTrajectory::snapshot() const
{
  using Marker = visualization_msgs::msg::Marker;
  visualization_msgs::msg::MarkerArray result;
  Marker clear;
  clear.header = header_;
  clear.action = Marker::DELETEALL;
  result.markers.push_back(clear);
  for (size_t i = 0; i < segments_.size(); ++i) {
    Marker marker;
    marker.header = header_;
    marker.ns = "tagslam_trajectory";
    marker.id = static_cast<int32_t>(i);
    marker.type =
      segments_[i].size() == 1 ? Marker::POINTS : Marker::LINE_STRIP;
    marker.action = Marker::ADD;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = segments_[i].size() == 1 ? 0.015 : 0.006;
    marker.scale.y = 0.015;
    marker.color.r = 0.05F;
    marker.color.g = 0.8F;
    marker.color.b = 1.0F;
    marker.color.a = 1.0F;
    marker.points = segments_[i];
    result.markers.push_back(std::move(marker));
  }
  return result;
}
}  // namespace tagslam
