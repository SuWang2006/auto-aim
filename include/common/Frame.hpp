#pragma once

#include "common/Quaternion.hpp"
#include <opencv2/core.hpp>

#include <cstdint>

namespace autoaim::common {

struct Frame {
  cv::Mat image;
  Quaternion imu{};
  double timestamp{0.0};
  std::uint64_t frame_id{0};
};

} // namespace autoaim::common
