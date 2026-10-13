// Exercise the actual OpenGL renderer, TCP framing, PNG and depth intrinsics.
#include <GLFW/glfw3.h>
#include <arpa/inet.h>
#include <sys/socket.h>
#include <unistd.h>

#include <cmath>
#include <cstring>
#include <iostream>
#include <stdexcept>
#include <vector>

#include "head_camera_server.h"
#include "lodepng.h"
#include "param.h"

void Check(bool value, const char *message) {
  if (!value) throw std::runtime_error(message);
}

void Receive(int fd, void *buffer, size_t size) {
  auto *bytes = static_cast<char *>(buffer);
  while (size) {
    const ssize_t n = recv(fd, bytes, size, 0);
    Check(n > 0, "camera receive failed/timed out");
    bytes += n;
    size -= n;
  }
}

template <typename T>
T Read(const std::vector<uint8_t> &bytes, size_t &offset) {
  Check(offset + sizeof(T) <= bytes.size(), "truncated plane");
  T result;
  std::memcpy(&result, bytes.data() + offset, sizeof(T));
  offset += sizeof(T);
  return result;
}

int main(int argc, char **argv) {
  if (!glfwInit()) return 77;
  glfwWindowHint(GLFW_VISIBLE, GLFW_FALSE);
  GLFWwindow *window = glfwCreateWindow(112, 63, "camera test", nullptr, nullptr);
  if (!window) { glfwTerminate(); return 77; }
  char error[1024] = {};
  const char *model_path = argc > 1 ? argv[1] : G1_MODEL_DIR "/g1_29dof_with_camera.xml";
  mjModel *model = mj_loadXML(model_path, nullptr,
                            error, sizeof(error));
  if (!model) { std::cerr << error; return 1; }
  mjData *data = mj_makeData(model);
  mj_forward(model, data);
  int result = 0;
  try {
    param::config.camera_port = 20000 + getpid() % 30000;
    // Deliberately nonuniform scaling: catches using scale_x for fy.
    param::config.camera_depth_width = 160;
    param::config.camera_depth_height = 120;
    for (const auto size : {std::pair<int, int>{112, 63}, {640, 480}}) {
      param::config.camera_width = size.first;
      param::config.camera_height = size.second;
      param::config.validate_camera();
      HeadCameraServer server(model, data, window);
      Check(server.start(), "server start");
      const int fd = socket(AF_INET, SOCK_STREAM, 0);
      timeval timeout{10, 0};
      setsockopt(fd, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof(timeout));
      sockaddr_in address{};
      address.sin_family = AF_INET;
      address.sin_port = htons(param::config.camera_port);
      address.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
      Check(connect(fd, reinterpret_cast<sockaddr *>(&address), sizeof(address)) == 0,
            "camera connect");
      uint32_t length = 0;
      Receive(fd, &length, sizeof(length));
      Check(length < 10000000, "invalid payload length");
      std::vector<uint8_t> payload(length);
      Receive(fd, payload.data(), payload.size());
      close(fd);
      Check(length > 4 && std::memcmp(payload.data(), "VD1\2", 4) == 0, "RGB-D header");
      size_t offset = 4;
      const double focal = size.second / (2.0 * std::tan(42.5 * mjPI / 360.0));
      for (int plane = 0; plane < 2; ++plane) {
        const auto kind = Read<uint8_t>(payload, offset);
        const auto format = Read<uint8_t>(payload, offset);
        const auto width = Read<uint16_t>(payload, offset);
        const auto height = Read<uint16_t>(payload, offset);
        const auto fx = Read<float>(payload, offset);
        const auto fy = Read<float>(payload, offset);
        const auto cx = Read<float>(payload, offset);
        const auto cy = Read<float>(payload, offset);
        const auto bytes = Read<uint32_t>(payload, offset);
        Check(kind == plane && offset + bytes <= payload.size(), "plane bounds/kind");
        Check(width == (plane ? 160 : size.first) &&
              height == (plane ? 120 : size.second), "plane dimensions");
        Check(std::abs(fx - focal * width / size.first) < 1e-3 &&
              std::abs(fy - focal * height / size.second) < 1e-3,
              "independently scaled depth intrinsics");
        Check(cx == width / 2.0f && cy == height / 2.0f, "principal point");
        if (plane == 0) {
          std::vector<unsigned char> rgb;
          unsigned png_width = 0, png_height = 0;
          Check(format == 2 && lodepng::decode(rgb, png_width, png_height,
                payload.data() + offset, bytes, LCT_RGB) == 0, "PNG decode");
          Check(png_width == width && png_height == height, "PNG/wire dimensions");
        } else {
          Check(format == 0 && bytes == width * height * 2u, "depth bytes");
        }
        offset += bytes;
      }
      Check(offset == payload.size(), "trailing bytes");
      std::cout << "PASS RGB-D render/stream " << size.first << 'x' << size.second << '\n';
    }
  } catch (const std::exception &e) {
    std::cerr << e.what() << '\n';
    result = 1;
  }
  mj_deleteData(data);
  mj_deleteModel(model);
  glfwDestroyWindow(window);
  glfwTerminate();
  return result;
}
