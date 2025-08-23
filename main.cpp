#include <fmt/format.h>
#include <glog/logging.h>

#include <CLI/CLI.hpp>
#include <algorithm>
#include <filesystem>
#include <set>
#include <string>

#include "driver_param.h"
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

hl::UdpPacket GetPacket(const std::vector<char>& data) {
  return hl::UdpPacket((const uint8_t*)data.data(), data.size(), 0);
}

int main(int argc, char** argv) {
  CLI::App app{"Hsai Binary to PCD Converter"};
  argv = app.ensure_utf8(argv);

  std::string input_dir;
  app.add_option("-i,--input", input_dir, "Input directory")->required();
  LOG(INFO) << "Input directory: " << input_dir;

  std::string output_dir;
  app.add_option("-o,--output", output_dir, "Output directory")->required();

  int num_files;
  app.add_option("-n,--num_files", num_files, "Number of files to process")->default_val(10);

  CLI11_PARSE(app, argc, argv);
  LOG(INFO) << "Output directory: " << output_dir;

  fs::create_directories(output_dir);

  std::set<fs::path> bin_files;
  std::copy_if(fs::directory_iterator(input_dir),
               fs::directory_iterator(),
               std::inserter(bin_files, bin_files.end()),
               [](const fs::path& p) { return p.extension() == ".bin"; });
  LOG(INFO) << "Number of packets: " << bin_files.size();

  using PointT = hl::LidarPointXYZICRTT;

  hl::UdpParser<PointT> parser("QT128C2X");
  LOG(INFO) << parser.GetLidarType();
  hl::LidarDecodedFrame<PointT> frame;

  parser.setFrameRightMemorySpace(frame);
  parser.SetPcapPlay(DATA_FROM_PCAP);
  parser.SetFrameAzimuth(0);

  parser.LoadCorrectionFile(fs::path(input_dir) / "QT128C2X_angle.csv");
  CHECK(parser.isSetCorrectionSucc());
  parser.LoadFiretimesFile(fs::path(input_dir) / "QT128C2X_firetime.csv");
  CHECK(parser.isSetFiretimeSucc());

  int i = 0;
  for (const auto& bin_file : bin_files) {
    i += 1;
    const auto data = ReadBinFile(bin_file);

    // Get the stem of the file
    LOG(INFO) << fmt::format("Processing file: {}", bin_file.stem().string());

    const auto packet = GetPacket(data);

    CHECK_EQ(parser.DecodePacket(frame, packet), 0);

    if (i > num_files) {
      break;
    }
  }
}
