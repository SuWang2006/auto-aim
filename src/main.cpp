#include "common/Frame.hpp"
#include "io/VideoDecoder.hpp"

#include <chrono>
#include <iostream>

int main(int argc, char *argv[]) {
  if (argc < 2) {
    std::cerr << "Usage: ./test_decoder <video.mkv>\n";
    return -1;
  }

  const std::string video_path = argv[1];

  autoaim::io::VideoDecoder decoder;

  if (!decoder.open_video(video_path)) {
    std::cerr << "Failed to open video\n";
    return -1;
  }

  std::cout << "Video opened successfully\n";

  auto start = std::chrono::steady_clock::now();

  autoaim::common::Frame frame;

  std::size_t count = 0;

  while (decoder.next_frame(frame)) {

    if (count % 100 == 0) {
      std::cout << "frame: " << frame.frame_id
                << " timestamp: " << frame.timestamp
                << " image: " << frame.image.cols << "x" << frame.image.rows
                << "\n";

      std::cout << "imu: " << frame.imu.qw << " " << frame.imu.qx << " "
                << frame.imu.qy << " " << frame.imu.qz << "\n";
    }

    count++;
  }

  auto end = std::chrono::steady_clock::now();

  double seconds = std::chrono::duration<double>(end - start).count();

  std::cout << "\nFinished\n";
  std::cout << "Decoded frames: " << count << "\n";
  std::cout << "FPS: " << count / seconds << "\n";

  return 0;
}
