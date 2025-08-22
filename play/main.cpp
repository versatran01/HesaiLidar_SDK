#include <algorithm>
#include <filesystem>
#include <fmt/core.h>
#include <glog/logging.h>
#include <string>

#include "lidar_types.h"

namespace fs = std::filesystem;
namespace hl = hesai::lidar;

int main(int argc, char **argv) {
  const fs::path packets_dir = "/home/ubuntu/data/temp/hesai_packets";
  const fs::path angle_correction_file = packets_dir / "QT128C2X_angle.csv";
  const fs::path firetime_correction_file =
      packets_dir / "QT128C2X_firetime.csv";

  std::vector<fs::path> packets_files;
  std::copy_if(fs::directory_iterator(packets_dir), fs::directory_iterator(),
               std::back_inserter(packets_files),
               [](const fs::path &p) { return p.extension() == ".bin"; });
  std::sort(packets_files.begin(), packets_files.end());
  LOG(INFO) << "Number of packets: " << packets_files.size();
}
