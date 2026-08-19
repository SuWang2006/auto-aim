#pragma once

#include "common/Frame.hpp"
#include "libavutil/pixfmt.h"

extern "C" {
#include <libavcodec/avcodec.h>
#include <libavformat/avformat.h>
#include <libswscale/swscale.h>
}

#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

namespace autoaim::io {
class VideoDecoder {
public:
  VideoDecoder();
  ~VideoDecoder();

  VideoDecoder(const VideoDecoder &) = delete;
  VideoDecoder &operator=(const VideoDecoder &) = delete;
  VideoDecoder(VideoDecoder &&) = delete;
  VideoDecoder &operator=(VideoDecoder &&) = delete;

  bool open_video(const std::string &path);
  bool next_frame(common::Frame &frame);

private:
  AVFormatContext *video_format_ctx_{nullptr};
  AVFormatContext *imu_format_ctx_{nullptr};
  AVCodecContext *codec_ctx_{nullptr};

  AVPacket *packet_{nullptr};
  AVFrame *frame_{nullptr};
  SwsContext *sws_ctx_{nullptr};

  int sws_width_{0}, sws_height_{0};
  AVPixelFormat sws_format_{AV_PIX_FMT_NONE};

  int video_stream_index_{-1};
  int subtitle_stream_index_{-1};

  struct IMUEntry {
    common::Quaternion q;
    double timestamp{0.0};
  };
  std::vector<IMUEntry> imu_table_;
  std::size_t imu_index_{0};
  std::uint64_t frame_id_{0};
  bool flushed_{false};

  bool load_imu_table();
  std::string parse_ass_base64(const std::string &text);
  common::Quaternion query_imu(double timestamp);

  bool receive_frame(common::Frame &frame);
  bool convert_frame(common::Frame &frame);

  void release();
};
} // namespace autoaim::io
