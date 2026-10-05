#include "head_camera_server.h"

#include <GLFW/glfw3.h>
#include <arpa/inet.h>
#include <netinet/tcp.h>
#include <sys/socket.h>
#include <unistd.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstring>
#include <iostream>

#include "camera_wire.h"
#include "lodepng.h"
#include "param.h"

namespace wire = unitree_mujoco::camera_wire;

HeadCameraServer::HeadCameraServer(mjModel *model, mjData *data,
                                   GLFWwindow *window)
    : model_(model), data_(data), window_(window) {}

HeadCameraServer::~HeadCameraServer() { stop(); }

bool HeadCameraServer::start() {
  camera_id_ = mj_name2id(model_, mjOBJ_CAMERA, "head_cam");
  if (camera_id_ < 0 || window_ == nullptr) {
    return false;
  }
  if (!startTcpServer()) {
    return false;
  }

  running_.store(true);
  accept_thread_ = std::thread(&HeadCameraServer::acceptClients, this);
  camera_thread_ = std::thread(&HeadCameraServer::cameraLoop, this);

  std::cout << "Head camera TCP server listening on port "
            << param::config.camera_port << " (" << kColorWidth << "x"
            << kColorHeight << " PNG, " << param::config.camera_fps << " fps)"
            << std::endl;
  return true;
}

bool HeadCameraServer::startTcpServer() {
  listen_fd_ = ::socket(AF_INET, SOCK_STREAM, 0);
  if (listen_fd_ < 0) {
    std::cerr << "Head camera socket failed: " << std::strerror(errno)
              << std::endl;
    return false;
  }

  int enabled = 1;
  ::setsockopt(listen_fd_, SOL_SOCKET, SO_REUSEADDR, &enabled, sizeof(enabled));

  sockaddr_in address{};
  address.sin_family = AF_INET;
  address.sin_addr.s_addr = htonl(INADDR_ANY);
  address.sin_port = htons(static_cast<uint16_t>(param::config.camera_port));
  if (::bind(listen_fd_, reinterpret_cast<sockaddr *>(&address),
             sizeof(address)) < 0) {
    std::cerr << "Head camera bind(" << param::config.camera_port
              << ") failed: " << std::strerror(errno) << std::endl;
    ::close(listen_fd_);
    listen_fd_ = -1;
    return false;
  }
  if (::listen(listen_fd_, 5) < 0) {
    std::cerr << "Head camera listen failed: " << std::strerror(errno)
              << std::endl;
    ::close(listen_fd_);
    listen_fd_ = -1;
    return false;
  }
  return true;
}

bool HeadCameraServer::initializeCamera() {
  glfwMakeContextCurrent(window_);

  mjv_defaultScene(&scene_);
  mjv_makeScene(model_, &scene_, 2000);
  mjr_defaultContext(&context_);
  mjr_makeContext(model_, &context_, mjFONTSCALE_100);

  mjv_defaultCamera(&camera_);
  camera_.type = mjCAMERA_FIXED;
  camera_.fixedcamid = camera_id_;
  mjv_defaultOption(&option_);

  rgb_bottom_up_.resize(kColorWidth * kColorHeight * 3);
  rgb_top_down_.resize(kColorWidth * kColorHeight * 3);
  if (param::config.camera_depth != 0) {
    depth_buffer_.resize(kColorWidth * kColorHeight);
    depth_millimeters_.resize(param::config.camera_depth_width *
                              param::config.camera_depth_height);
  }

  return true;
}

void HeadCameraServer::acceptClients() {
  while (running_.load()) {
    sockaddr_in client_address{};
    socklen_t address_length = sizeof(client_address);
    const int fd =
        ::accept(listen_fd_, reinterpret_cast<sockaddr *>(&client_address),
                 &address_length);
    if (fd < 0) {
      if (!running_.load()) {
        break;
      }
      continue;
    }

    int enabled = 1;
    ::setsockopt(fd, IPPROTO_TCP, TCP_NODELAY, &enabled, sizeof(enabled));
    std::lock_guard<std::mutex> lock(clients_mutex_);
    clients_.push_back(fd);
    std::cout << "Head camera client connected (total: " << clients_.size()
              << ")" << std::endl;
  }
}

void HeadCameraServer::cameraLoop() {
  if (!initializeCamera()) {
    running_.store(false);
    return;
  }

  const auto period =
      std::chrono::microseconds(1000000 / param::config.camera_fps);
  auto next_frame = std::chrono::steady_clock::now();
  while (running_.load()) {
    renderCamera();
    next_frame += period;
    const auto now = std::chrono::steady_clock::now();
    if (next_frame < now) {
      next_frame = now;
    }
    std::this_thread::sleep_until(next_frame);
  }

  mjr_freeContext(&context_);
  mjv_freeScene(&scene_);
  glfwMakeContextCurrent(nullptr);
}

void HeadCameraServer::renderCamera() {
  {
    std::lock_guard<std::mutex> lock(clients_mutex_);
    if (clients_.empty()) {
      return;
    }
  }

  mjv_updateScene(model_, data_, &option_, nullptr, &camera_, mjCAT_ALL,
                  &scene_);
  mjr_setBuffer(mjFB_OFFSCREEN, &context_);
  mjr_render(viewport_, &scene_, &context_);
  const bool include_depth = param::config.camera_depth != 0;
  mjr_readPixels(rgb_bottom_up_.data(),
                 include_depth ? depth_buffer_.data() : nullptr, viewport_,
                 &context_);

  constexpr size_t row_bytes = kColorWidth * 3;
  for (int row = 0; row < kColorHeight; ++row) {
    std::memcpy(rgb_top_down_.data() + row * row_bytes,
                rgb_bottom_up_.data() + (kColorHeight - 1 - row) * row_bytes,
                row_bytes);
  }

  std::vector<unsigned char> png;
  const unsigned png_error = lodepng::encode(png, rgb_top_down_, kColorWidth,
                                             kColorHeight, LCT_RGB, 8);
  if (png_error != 0) {
    std::cerr << "Head camera PNG encode failed: "
              << lodepng_error_text(png_error) << std::endl;
    return;
  }

  std::vector<uint8_t> payload;
  if (!include_depth) {
    payload.assign(png.begin(), png.end());
  } else {
    const float fovy = static_cast<float>(model_->cam_fovy[camera_id_]);
    const float fy = (kColorHeight / 2.0f) /
                     std::tan(fovy * static_cast<float>(mjPI) / 360.0f);
    payload = wire::BeginPayload(2);
    wire::AppendPlane(payload, wire::Kind::kColor, wire::Format::kPng,
                      kColorWidth, kColorHeight, fy, fy, kColorWidth / 2.0f,
                      kColorHeight / 2.0f, png.data(),
                      static_cast<uint32_t>(png.size()));
    appendDepthPlane(payload, fy);
  }

  broadcast(wire::FrameTcpRecord(payload));
}

void HeadCameraServer::appendDepthPlane(std::vector<uint8_t> &payload,
                                        float color_fy) {
  const int width = param::config.camera_depth_width;
  const int height = param::config.camera_depth_height;
  const float extent = static_cast<float>(model_->stat.extent);
  const float near_plane = extent * static_cast<float>(model_->vis.map.znear);
  const float far_plane = extent * static_cast<float>(model_->vis.map.zfar);

  for (int row = 0; row < height; ++row) {
    const int source_y = (row * kColorHeight) / height;
    const int buffer_y = kColorHeight - 1 - source_y;
    for (int column = 0; column < width; ++column) {
      const int source_x = (column * kColorWidth) / width;
      const float normalized_depth =
          depth_buffer_[buffer_y * kColorWidth + source_x];
      const float meters =
          near_plane * far_plane /
          (far_plane - normalized_depth * (far_plane - near_plane));
      const float millimeters = std::clamp(meters * 1000.0f, 0.0f, 65535.0f);
      depth_millimeters_[row * width + column] =
          static_cast<uint16_t>(std::lround(millimeters));
    }
  }

  const float scale = static_cast<float>(width) / kColorWidth;
  wire::AppendPlane(
      payload, wire::Kind::kDepth, wire::Format::kRawU16Millimeters,
      static_cast<uint16_t>(width), static_cast<uint16_t>(height),
      color_fy * scale, color_fy * scale, width / 2.0f, height / 2.0f,
      depth_millimeters_.data(),
      static_cast<uint32_t>(depth_millimeters_.size() * sizeof(uint16_t)));
}

void HeadCameraServer::broadcast(const std::vector<uint8_t> &record) {
  std::lock_guard<std::mutex> lock(clients_mutex_);
  for (auto client = clients_.begin(); client != clients_.end();) {
    size_t sent = 0;
    bool connected = true;
    while (sent < record.size()) {
      const ssize_t count = ::send(*client, record.data() + sent,
                                   record.size() - sent, MSG_NOSIGNAL);
      if (count <= 0) {
        connected = false;
        break;
      }
      sent += static_cast<size_t>(count);
    }

    if (!connected) {
      ::close(*client);
      client = clients_.erase(client);
      std::cout << "Head camera client disconnected (remaining: "
                << clients_.size() << ")" << std::endl;
    } else {
      ++client;
    }
  }
}

void HeadCameraServer::stop() {
  running_.store(false);
  if (listen_fd_ >= 0) {
    ::shutdown(listen_fd_, SHUT_RDWR);
    ::close(listen_fd_);
    listen_fd_ = -1;
  }
  if (accept_thread_.joinable()) {
    accept_thread_.join();
  }
  if (camera_thread_.joinable()) {
    camera_thread_.join();
  }

  std::lock_guard<std::mutex> lock(clients_mutex_);
  for (const int fd : clients_) {
    ::close(fd);
  }
  clients_.clear();
}
