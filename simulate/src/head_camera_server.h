#pragma once

#include <mujoco/mujoco.h>

#include <atomic>
#include <cstdint>
#include <mutex>
#include <thread>
#include <vector>

struct GLFWwindow;

class HeadCameraServer {
 public:
  HeadCameraServer(mjModel *model, mjData *data, GLFWwindow *window);
  ~HeadCameraServer();

  HeadCameraServer(const HeadCameraServer &) = delete;
  HeadCameraServer &operator=(const HeadCameraServer &) = delete;

  bool start();

 private:
  int color_width_;
  int color_height_;

  bool startTcpServer();
  bool initializeCamera();
  void acceptClients();
  void cameraLoop();
  void renderCamera();
  void appendDepthPlane(std::vector<uint8_t> &payload, float color_fy);
  void broadcast(const std::vector<uint8_t> &record);
  void stop();

    mjModel *model_;
    mjData *data_;
    GLFWwindow *window_;
    int camera_id_ = -1;

  mjvScene scene_{};
  mjvCamera camera_{};
  mjvOption option_{};
  mjrContext context_{};
  mjrRect viewport_{};

  std::vector<unsigned char> rgb_bottom_up_;
  std::vector<unsigned char> rgb_top_down_;
  std::vector<float> depth_buffer_;
  std::vector<uint16_t> depth_millimeters_;

  int listen_fd_ = -1;
  std::atomic<bool> running_{false};
  std::thread accept_thread_;
  std::thread camera_thread_;
  std::vector<int> clients_;
  std::mutex clients_mutex_;
};
