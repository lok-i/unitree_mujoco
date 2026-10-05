#pragma once

// Self-describing RGB-D payload used by the Vibe camera stream.
//
// The outer TCP record remains: u32 little-endian payload length + payload.
// A multi-plane payload is:
//   "VD1" | u8 plane_count | plane[plane_count]
// Each plane is:
//   u8 kind | u8 format | u16 width | u16 height
//   | f32 fx,fy,cx,cy | u32 byte_count | bytes

#include <cstdint>
#include <cstring>
#include <vector>

namespace unitree_mujoco::camera_wire {

inline constexpr char kMagic[3] = {'V', 'D', '1'};

enum class Kind : uint8_t {
  kColor = 0,
  kDepth = 1,
};

enum class Format : uint8_t {
  kRawU16Millimeters = 0,
  kJpeg = 1,
  kPng = 2,
};

namespace detail {

template <typename T>
void Append(std::vector<uint8_t> &output, T value) {
  const auto *bytes = reinterpret_cast<const uint8_t *>(&value);
  output.insert(output.end(), bytes, bytes + sizeof(T));
}

}  // namespace detail

inline std::vector<uint8_t> BeginPayload(uint8_t plane_count) {
  std::vector<uint8_t> output(kMagic, kMagic + sizeof(kMagic));
  output.push_back(plane_count);
  return output;
}

inline void AppendPlane(std::vector<uint8_t> &output, Kind kind, Format format,
                        uint16_t width, uint16_t height, float fx, float fy,
                        float cx, float cy, const void *data,
                        uint32_t byte_count) {
  detail::Append<uint8_t>(output, static_cast<uint8_t>(kind));
  detail::Append<uint8_t>(output, static_cast<uint8_t>(format));
  detail::Append<uint16_t>(output, width);
  detail::Append<uint16_t>(output, height);
  detail::Append<float>(output, fx);
  detail::Append<float>(output, fy);
  detail::Append<float>(output, cx);
  detail::Append<float>(output, cy);
  detail::Append<uint32_t>(output, byte_count);
  const auto *bytes = static_cast<const uint8_t *>(data);
  output.insert(output.end(), bytes, bytes + byte_count);
}

inline std::vector<uint8_t> FrameTcpRecord(
    const std::vector<uint8_t> &payload) {
  const auto byte_count = static_cast<uint32_t>(payload.size());
  std::vector<uint8_t> record;
  record.reserve(sizeof(byte_count) + payload.size());
  detail::Append<uint32_t>(record, byte_count);
  record.insert(record.end(), payload.begin(), payload.end());
  return record;
}

}  // namespace unitree_mujoco::camera_wire
