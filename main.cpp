#include <absl/types/span.h>
#include <fmt/format.h>
#include <glog/logging.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/register_point_struct.h>

#include <CLI/CLI.hpp>
#include <algorithm>
#include <filesystem>
#include <optional>
#include <set>
#include <string>

#include "driver_param.h"
#include "inner_com.h"
#include "lidar_types.h"
#include "udp_parser.h"

namespace fs = std::filesystem;
namespace hl = hesai::lidar;

struct PointHesaiLidar {
  PCL_ADD_POINT4D;  // Adds x, y, z, and a float for padding
  union EIGEN_ALIGN16 {
    struct {
      uint8_t refl;   // 1b
      uint8_t conf;   // 1b
      uint8_t retr;   // 1b
      uint8_t loop;   // 1b
      uint16_t ring;  // 2b
      uint16_t fire;  // 2b
    };
  };
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW  // Ensure correct memory alignment
};

POINT_CLOUD_REGISTER_POINT_STRUCT(PointHesaiLidar,
                                  (float, x, x)           //
                                  (float, y, y)           //
                                  (float, z, z)           //
                                  (uint8_t, refl, refl)   //
                                  (uint8_t, conf, conf)   //
                                  (uint8_t, retr, retr)   //
                                  (uint8_t, loop, loop)   //
                                  (uint16_t, ring, ring)  //
                                  (uint16_t, fire, fire)  //
)

hl::UdpPacket GetPacket(absl::Span<const char> data) {
  return hl::UdpPacket((const uint8_t*)data.data(), data.size(), 0);
}

class HesaiQT128Parser {
 public:
  using HesaiPoint = hesai::lidar::LidarPointXYZICRTT;

  explicit HesaiQT128Parser(bool organized = true)
      : organized_(organized), udp_parser_("QT128C2X") {
    udp_parser_.setFrameRightMemorySpace(frame_);
    udp_parser_.SetPcapPlay(DATA_FROM_PCAP);
    udp_parser_.SetFrameAzimuth(0);
  }

  void LoadCorrectionFile(const std::string& file) {
    udp_parser_.LoadCorrectionFile(file);
    CHECK_EQ(udp_parser_.isSetCorrectionSucc(), true);
  }
  void LoadFiretimesFile(const std::string& file) {
    udp_parser_.LoadFiretimesFile(file);
    CHECK_EQ(udp_parser_.isSetFiretimeSucc(), true);
  }

  void DecodePacket(absl::Span<const char> data) {
    // Prev packet is not empty, decode it first
    if (!prev_data_.empty()) {
      // This indicates we are at the start of a frame, update the frame
      frame_.Update();

      // Decode previous data
      const auto packet = GetPacket(prev_data_);
      udp_parser_.DecodePacket(frame_, packet);
      udp_parser_.ComputeXYZI(frame_, frame_.packet_num - 1);

      prev_data_.clear();
      // Unset frame complete
      frame_complete_ = false;
    }

    // Then decode current data
    const auto packet = GetPacket(data);
    udp_parser_.DecodePacket(frame_, packet);
    udp_parser_.ComputeXYZI(frame_, frame_.packet_num - 1);

    if (frame_.scan_complete) {
      // We need to save the current packet to prev_data_
      prev_data_.assign(data.begin(), data.end());

      if (got_first_frame_) {
        frame_complete_ = true;
        // Do some stuff with the cloud

        cloud_.clear();
        for (int i = 0; i < frame_.packet_num; ++i) {
          for (int j = 0; j < frame_.valid_points[i]; ++j) {
            const int ch = j % frame_.laser_num;

            const auto k = i * frame_.per_points_num + j;
            const auto& pt = frame_.points[k];
            const auto& pd = frame_.pointData[k];
            CHECK_EQ(pt.ring, ch);

            if (pd.data.dQT.loopIndex == 0) {
              // Skip
              if (pt.ring < 32) {
                continue;
              }
            } else {
              if (32 <= pt.ring && pt.ring < 64) {
                continue;
              }
            }

            PointHesaiLidar p;
            p.x = pt.x;
            p.y = pt.y;
            p.z = pt.z;
            p.refl = pt.intensity;
            p.conf = pt.confidence;
            p.retr = j < frame_.laser_num ? 1 : 2;
            p.loop = pd.data.dQT.loopIndex;
            p.ring = pt.ring;
            p.fire = i;
            cloud_.push_back(p);
          }
        }
      } else {
        got_first_frame_ = true;
      }
    }
  }

  bool frame_complete() const { return frame_complete_; }

  const auto& cloud() const { return cloud_; }

 private:
  bool organized_{true};  // output organized point cloud
  bool frame_complete_{false};
  bool got_first_frame_{false};
  pcl::PointCloud<PointHesaiLidar> cloud_;
  std::vector<char> prev_data_;  // previous packet data
  hesai::lidar::UdpParser<HesaiPoint> udp_parser_;
  hesai::lidar::LidarDecodedFrame<HesaiPoint> frame_;
};

std::vector<char> ReadBinFile(const fs::path& file) {
  std::ifstream ifs(file, std::ios::binary);
  CHECK(ifs);
  std::vector<char> data((std::istreambuf_iterator<char>(ifs)), std::istreambuf_iterator<char>());
  CHECK_EQ(data.size(), 1127);
  return data;
}

int main(int argc, char** argv) {
  CLI::App app{"Hesai Binary to PCD Converter"};
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

  HesaiQT128Parser qt128_parser;
  qt128_parser.LoadCorrectionFile(fs::path(input_dir) / "QT128C2X_angle.csv");
  qt128_parser.LoadFiretimesFile(fs::path(input_dir) / "QT128C2X_firetime.csv");

  int i = 0;
  int frame_id = 0;
  for (const auto& bin_file : bin_files) {
    i += 1;

    const auto data = ReadBinFile(bin_file);
    qt128_parser.DecodePacket(data);

    if (qt128_parser.frame_complete()) {
      LOG(INFO) << "Frame: " << frame_id;
      pcl::io::savePCDFile(fs::path(output_dir) / fmt::format("cloud_{}.pcd", frame_id),
                           qt128_parser.cloud());
      frame_id++;
    }

    if (i > num_files) {
      break;
    }
  }
}
