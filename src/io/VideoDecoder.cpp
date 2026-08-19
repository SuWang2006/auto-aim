#include "io/VideoDecoder.hpp"
#include "common/Frame.hpp"
#include "common/Quaternion.hpp"
#include "opencv2/core/mat.hpp"

extern "C" {
#include <libavutil/base64.h>
#include <libavutil/error.h>
}

#include <algorithm>
#include <array>
#include <cstring>
#include <new>

namespace autoaim::io {

VideoDecoder::VideoDecoder() {
  packet_ = av_packet_alloc();
  if (packet_ == nullptr) {
    throw std::bad_alloc{};
  }

  frame_ = av_frame_alloc();
  if (frame_ == nullptr) {
    av_packet_free(&packet_);
    throw std::bad_alloc{};
  }
}

VideoDecoder::~VideoDecoder() { release(); }

void VideoDecoder::release() {
  if (sws_ctx_ != nullptr) {
    sws_freeContext(sws_ctx_);
    sws_ctx_ = nullptr;
  }
  if (frame_ != nullptr) {
    av_frame_free(&frame_);
  }
  if (packet_ != nullptr) {
    av_packet_free(&packet_);
  }
  if (codec_ctx_ != nullptr) {
    avcodec_free_context(&codec_ctx_);
  }
  if (video_format_ctx_ != nullptr) {
    avformat_close_input(&video_format_ctx_);
  }
  if (imu_format_ctx_ != nullptr) {
    avformat_close_input(&imu_format_ctx_);
  }

  video_stream_index_ = -1;
  subtitle_stream_index_ = -1;
  imu_table_.clear();
  imu_index_ = 0;
  frame_id_ = 0;
  flushed_ = false;
  sws_width_ = 0;
  sws_height_ = 0;
  sws_format_ = AV_PIX_FMT_NONE;
}

bool VideoDecoder::open_video(const std::string &path) {
  if (avformat_open_input(&video_format_ctx_, path.c_str(), nullptr, nullptr) <
      0) {
    release();
    return false;
  }
  if (avformat_find_stream_info(video_format_ctx_, nullptr) < 0) {
    release();
    return false;
  }

  for (unsigned int i = 0; i < video_format_ctx_->nb_streams; i++) {
    AVStream *stream = video_format_ctx_->streams[i];
    if (stream->codecpar->codec_type == AVMEDIA_TYPE_VIDEO) {
      video_stream_index_ = static_cast<int>(i);
      break;
    }
  }
  if (video_stream_index_ == -1) {
    release();
    return false;
  }

  AVStream *video_stream = video_format_ctx_->streams[video_stream_index_];
  const AVCodec *codec = avcodec_find_decoder(video_stream->codecpar->codec_id);
  if (codec == nullptr) {
    release();
    return false;
  }

  codec_ctx_ = avcodec_alloc_context3(codec);
  if (codec_ctx_ == nullptr) {
    release();
    return false;
  }

  if (avcodec_parameters_to_context(codec_ctx_, video_stream->codecpar) < 0) {
    release();
    return false;
  }
  if (avcodec_open2(codec_ctx_, codec, nullptr) < 0) {
    release();
    return false;
  }

  if (avformat_open_input(&imu_format_ctx_, path.c_str(), nullptr, nullptr) <
      0) {
    release();
    return false;
  }
  if (avformat_find_stream_info(imu_format_ctx_, nullptr) < 0) {
    release();
    return false;
  }

  for (unsigned int i = 0; i < imu_format_ctx_->nb_streams; i++) {
    AVStream *stream = imu_format_ctx_->streams[i];
    if (stream->codecpar->codec_type == AVMEDIA_TYPE_SUBTITLE) {
      subtitle_stream_index_ = static_cast<int>(i);
      break;
    }
  }
  if (subtitle_stream_index_ == -1) {
    release();
    return false;
  }

  if (!load_imu_table()) {
    release();
    return false;
  }

  imu_index_ = 0;
  frame_id_ = 0;
  flushed_ = false;
  return true;
}

bool VideoDecoder::load_imu_table() {
  AVStream *stream = imu_format_ctx_->streams[subtitle_stream_index_];
  AVPacket *subtitle_packet_ = av_packet_alloc();
  if (subtitle_packet_ == nullptr) {
    return false;
  }

  while (av_read_frame(imu_format_ctx_, subtitle_packet_) >= 0) {
    if (subtitle_packet_->stream_index != subtitle_stream_index_) {
      av_packet_unref(subtitle_packet_);
      continue;
    }
    if (subtitle_packet_->pts == AV_NOPTS_VALUE) {
      av_packet_unref(subtitle_packet_);
      continue;
    }
    const double timestamp = subtitle_packet_->pts * av_q2d(stream->time_base);

    std::string ass(reinterpret_cast<const char *>(subtitle_packet_->data),
                    subtitle_packet_->size);
    std::string base64 = parse_ass_base64(ass);
    if (base64.empty()) {
      av_packet_unref(subtitle_packet_);
      continue;
    }

    std::array<uint8_t, 32> buffer{};
    const int size =
      av_base64_decode(buffer.data(), base64.c_str(), base64.size());
    if (size != 32) {
      av_packet_unref(subtitle_packet_);
      continue;
    }
    double q_data[4]{};
    std::memcpy(q_data, buffer.data(), sizeof(q_data));

    IMUEntry imu;
    imu.timestamp = timestamp;
    imu.q.qw = q_data[0];
    imu.q.qx = q_data[1];
    imu.q.qy = q_data[2];
    imu.q.qz = q_data[3];
    imu.q.normalize();
    imu_table_.push_back(imu);
    av_packet_unref(subtitle_packet_);
  }
  av_packet_free(&subtitle_packet_);

  std::sort(imu_table_.begin(), imu_table_.end(),
            [](const IMUEntry &a, const IMUEntry &b) {
              return a.timestamp < b.timestamp;
            });

  return !imu_table_.empty();
}

std::string VideoDecoder::parse_ass_base64(const std::string &text) {
  const size_t pos = text.rfind(',');
  if (pos == std::string::npos) {
    return {};
  }
  return text.substr(pos + 1);
}

common::Quaternion VideoDecoder::query_imu(double timestamp) {
  if (imu_table_.empty()) {
    return {};
  }

  while (imu_index_ + 1 < imu_table_.size() &&
         imu_table_[imu_index_ + 1].timestamp <= timestamp) {
    imu_index_++;
  }
  if (imu_index_ + 1 >= imu_table_.size()) {
    return imu_table_[imu_index_].q;
  }

  const IMUEntry &previous = imu_table_[imu_index_];
  const IMUEntry &next = imu_table_[imu_index_ + 1];
  const double dt = next.timestamp - previous.timestamp;
  if (dt <= 1e-9) {
    return previous.q;
  }

  double alpha = (timestamp - previous.timestamp) / dt;
  alpha = std::clamp(alpha, 0.0, 1.0);

  return common::slerp(previous.q, next.q, alpha);
}

bool VideoDecoder::next_frame(common::Frame &frame) {
  while (true) {
    if (receive_frame(frame)) {
      return true;
    }
    if (flushed_) {
      return false;
    }

    int ret = av_read_frame(video_format_ctx_, packet_);
    if (ret < 0) {
      if (avcodec_send_packet(codec_ctx_, nullptr) < 0) {
        return false;
      }
      flushed_ = true;
      continue;
    }

    if (packet_->stream_index != video_stream_index_) {
      av_packet_unref(packet_);
      continue;
    }

    ret = avcodec_send_packet(codec_ctx_, packet_);
    av_packet_unref(packet_);
    if (ret < 0) {
      return false;
    }
  }
}

bool VideoDecoder::receive_frame(common::Frame &frame) {
  int ret = avcodec_receive_frame(codec_ctx_, frame_);
  if (ret == AVERROR(EAGAIN) || ret == AVERROR_EOF || ret < 0) {
    return false;
  }
  return convert_frame(frame);
}

bool VideoDecoder::convert_frame(common::Frame &frame) {
  const int width = frame_->width, height = frame_->height;
  const AVPixelFormat format = static_cast<AVPixelFormat>(frame_->format);

  if (!sws_ctx_ || sws_width_ != width || sws_height_ != height ||
      sws_format_ != format) {
    if (sws_ctx_ != nullptr) {
      sws_freeContext(sws_ctx_);
      sws_ctx_ = nullptr;
    }
    sws_ctx_ =
      sws_getContext(width, height, format, width, height, AV_PIX_FMT_BGR24,
                     SWS_BILINEAR, nullptr, nullptr, nullptr);
    if (sws_ctx_ == nullptr) {
      return false;
    }
    sws_width_ = width, sws_height_ = height, sws_format_ = format;
  }

  cv::Mat image(height, width, CV_8UC3);
  uint8_t *dst_data[4] = {image.data, nullptr, nullptr, nullptr};
  int dst_linesize[4] = {static_cast<int>(image.step), 0, 0, 0};

  if (sws_scale(sws_ctx_, frame_->data, frame_->linesize, 0, height, dst_data,
                dst_linesize) <= 0) {
    return false;
  }

  AVStream *stream = video_format_ctx_->streams[video_stream_index_];

  frame.image = image;
  frame.frame_id = frame_id_++;
  if (frame_->pts != AV_NOPTS_VALUE) {
    frame.timestamp = frame_->pts * av_q2d(stream->time_base);
  } else {
    frame.timestamp = 0.0;
  }
  frame.imu = query_imu(frame.timestamp);

  return true;
}

} // namespace autoaim::io
