#include <mujoco/mujoco.h>

#include <cstdint>
#include <cstring>
#include <iostream>
#include <memory>
#include <string>
#include <vector>

#include "camera_wire.h"

namespace wire = unitree_mujoco::camera_wire;

namespace {

using ModelPtr = std::unique_ptr<mjModel, decltype(&mj_deleteModel)>;

ModelPtr LoadModel(const std::string &path) {
  char error[1024] = {};
  ModelPtr model(mj_loadXML(path.c_str(), nullptr, error, sizeof(error)),
                 mj_deleteModel);
  if (!model) {
    std::cerr << "Unable to load " << path << ": " << error << std::endl;
  }
  return model;
}

bool CheckEquivalentDynamics(const mjModel &base, const mjModel &camera) {
  return base.nq == camera.nq && base.nv == camera.nv && base.nu == camera.nu &&
         base.nbody == camera.nbody && base.njnt == camera.njnt &&
         base.nsensor == camera.nsensor && base.ngeom == camera.ngeom &&
         base.nmesh == camera.nmesh && camera.ncam == base.ncam + 1;
}

bool CheckWireRecord() {
  const std::vector<uint8_t> color = {1, 2, 3};
  const std::vector<uint16_t> depth = {100, 200};
  auto payload = wire::BeginPayload(2);
  wire::AppendPlane(payload, wire::Kind::kColor, wire::Format::kPng, 2, 1,
                    10.0f, 10.0f, 1.0f, 0.5f, color.data(), color.size());
  wire::AppendPlane(payload, wire::Kind::kDepth,
                    wire::Format::kRawU16Millimeters, 2, 1, 10.0f, 10.0f, 1.0f,
                    0.5f, depth.data(), depth.size() * sizeof(uint16_t));

  const auto record = wire::FrameTcpRecord(payload);
  uint32_t encoded_size = 0;
  std::memcpy(&encoded_size, record.data(), sizeof(encoded_size));
  return encoded_size == payload.size() &&
         record.size() == sizeof(encoded_size) + payload.size() &&
         payload.size() ==
             4 + 26 * 2 + color.size() + depth.size() * sizeof(uint16_t) &&
         std::memcmp(payload.data(), "VD1", 3) == 0 && payload[3] == 2;
}

}  // namespace

int main() {
  const std::string model_dir = G1_MODEL_DIR;
  const auto base = LoadModel(model_dir + "/g1_29dof.xml");
  const auto camera = LoadModel(model_dir + "/g1_29dof_with_camera.xml");
  if (!base || !camera) {
    return 1;
  }
  if (!CheckEquivalentDynamics(*base, *camera)) {
    std::cerr << "Camera model changes the G1 dynamics or topology"
              << std::endl;
    return 1;
  }
  if (mj_name2id(camera.get(), mjOBJ_CAMERA, "head_cam") < 0) {
    std::cerr << "Camera model has no head_cam" << std::endl;
    return 1;
  }
  if (!CheckWireRecord()) {
    std::cerr << "Camera wire record is malformed" << std::endl;
    return 1;
  }
  return 0;
}
