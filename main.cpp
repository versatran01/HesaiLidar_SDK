#include <fmt/format.h>
#include <glog/logging.h>

#include <CLI/CLI.hpp>
#include <algorithm>
#include <filesystem>
#include <set>
#include <string>

#include "driver_param.h"
#include "inner_com.h"
#include "lidar.h"
#include "lidar_types.h"
#include "udp_parser.h"
#include "udp_protocol_p40.h"

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

  std::vector<hl::FunctionSafety> func_safety;
  std::vector<PacketDecodeData> decode_data;

  bool first_frame = false;

  int i = 0;
  for (const auto& bin_file : bin_files) {
    i += 1;
    const auto data = ReadBinFile(bin_file);

    // Get the stem of the file
    // LOG(INFO) << fmt::format("Processing file: {}", bin_file.stem().string());
    const auto packet = GetPacket(data);
    CHECK_EQ(parser.DecodePacket(frame, packet), 0);
    LOG_IF(INFO, frame.scan_complete) << "Scan complete";

    if (frame.scan_complete) {
      LOG(INFO) << "First frame complete, packets: " << frame.packet_num;

      int num = 0;
      for (int i = 0; i < frame.packet_num; i++) {
        num += frame.valid_points[i];
      }
      LOG(INFO) << fmt::format("valid points: {}", num);
      frame.Update();

      parser.DecodePacket(frame, packet);

      if (first_frame == false) {
        first_frame = true;
        continue;
      }

      // if the packet which contains split frame msgs is valid, it will be
      // the first packet of new frame
    }

    // const auto* header =
    //     reinterpret_cast<const HS_LIDAR_HEADER_QT_V2*>(data.data() +
    //     sizeof(HS_LIDAR_PRE_HEADER));
    // const int unitSize = header->unitSize();

    // if (hl::hasFunctionSafety(header->m_u8Status)) {
    //   const auto* pfs = reinterpret_cast<const HS_LIDAR_FUNCTION_SAFETY*>(
    //       (const unsigned char*)header + sizeof(HS_LIDAR_HEADER_QT_V2) +
    //       (sizeof(HS_LIDAR_BODY_AZIMUTH_QT_V2) + unitSize * header->GetLaserNum()) *
    //           header->GetBlockNum() +
    //       sizeof(HS_LIDAR_BODY_CRC_QT_V2));

    // const auto lidar_state = pfs->GetLidarState();

    //   hl::FunctionSafety fs;
    //   fs.is_valid = true;
    //   fs.fs_version = pfs->m_u8Version;  // 1
    //   fs.status = pfs->m_u8Status;
    //   fs.fault_info = pfs->m_u8FaultInfo;   //
    //   fs.fault_code = pfs->GetFaultCode();  // 0 is no fault
    //   LOG_IF(INFO, false) << fmt::format(
    //       "Lidar state: {}, FS version: {}, status: {}, fault info: {}, fault code: {}",
    //       lidar_state,  // 1 is normal
    //       fs.fs_version,
    //       fs.status,
    //       fs.fault_info,
    //       fs.fault_code);
    // }

    // const auto* tail = reinterpret_cast<const HS_LIDAR_TAIL_QT_V2*>(
    //     (const unsigned char*)header + sizeof(HS_LIDAR_HEADER_QT_V2) +
    //     (sizeof(HS_LIDAR_BODY_AZIMUTH_QT_V2) + unitSize * header->GetLaserNum()) *
    //         header->GetBlockNum() +
    //     sizeof(HS_LIDAR_BODY_CRC_QT_V2) +
    //     (hasFunctionSafety(header->m_u8Status) ? sizeof(HS_LIDAR_FUNCTION_SAFETY) : 0));

    // if (hl::hasSeqNum(header->m_u8Status)) {
    //   const auto* tail_seq_num = reinterpret_cast<const HS_LIDAR_TAIL_SEQ_NUM_QT_V2*>(
    //       (const unsigned char*)tail + sizeof(HS_LIDAR_TAIL_QT_V2));
    //   LOG_IF(INFO, false) << fmt::format("Seq num: {}", tail_seq_num->GetSeqNum());
    // }

    // const auto spin_speed = tail->GetMotorSpeed();      // 600 fpm
    // const auto return_mode = tail->GetReturnMode();     // 0x3B first, last return, default
    // const auto block_num = header->GetBlockNum();       // 2
    // const auto laser_num = header->GetLaserNum();       // 128
    // const auto per_points_num = block_num * laser_num;  // 256
    // const auto distance_unit = header->GetDistUnit();   // 0.004 or 4mm
    // LOG(INFO) << fmt::format(
    //     "spin_speed: {}, return_mode: {:x}, block_num: {}, laser_num: {}, per_points_num: {}, "
    //     "dist_unit: {}",
    //     spin_speed,
    //     return_mode,
    //     block_num,
    //     laser_num,
    //     per_points_num,
    //     distance_unit);

    // LOG(INFO) << fmt::format(
    //     "per_points_num: {}, max_points_per_packet: {}", per_points_num,
    //     frame.maxPointPerPacket);

    // const auto time = tail->GetMicroLidarTimeU64(parser);

    // const auto* azimuth = reinterpret_cast<const HS_LIDAR_BODY_AZIMUTH_QT_V2*>(
    //     (const unsigned char*)header + sizeof(HS_LIDAR_HEADER_QT_V2));
    // const auto u16Azimuth = azimuth->GetAzimuth();

    if (i > num_files) {
      break;
    }
  }
}
