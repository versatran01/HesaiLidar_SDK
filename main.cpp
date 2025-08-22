#include <fmt/format.h>
#include <glog/logging.h>

#include <algorithm>
#include <filesystem>
#include <set>
#include <string>

#include "lidar.h"
#include "lidar_types.h"
#include "udp_parser.h"

namespace fs = std::filesystem;
namespace hl = hesai::lidar;

std::vector<char> ReadBinFile(const fs::path& file) {
  std::ifstream ifs(file, std::ios::binary);
  CHECK(ifs);
  std::vector<char> data((std::istreambuf_iterator<char>(ifs)), std::istreambuf_iterator<char>());
  CHECK_EQ(data.size(), 1127);
  return data;
}

int main(int argc, char** argv) {
  const fs::path packets_dir = "/home/ubuntu/data/temp/hesai_packets";
  const fs::path angle_correction_file = packets_dir / "QT128C2X_angle.csv";
  const fs::path firetime_correction_file = packets_dir / "QT128C2X_firetime.csv";

  std::set<fs::path> bin_files;
  std::copy_if(fs::directory_iterator(packets_dir),
               fs::directory_iterator(),
               std::inserter(bin_files, bin_files.end()),
               [](const fs::path& p) { return p.extension() == ".bin"; });
  LOG(INFO) << "Number of packets: " << bin_files.size();

  using PointT = hl::LidarPointXYZICRTT;

  UdpParser<PointT> parser("QT128C2X");
  LOG(INFO) << parser.GetLidarType();

  for (const auto& bin_file : bin_files) {
    const auto data = ReadBinFile(bin_file);

    // Get the stem of the file
    LOG(INFO) << fmt::format("Processing file: {}", bin_file.stem().string());
  }
}
