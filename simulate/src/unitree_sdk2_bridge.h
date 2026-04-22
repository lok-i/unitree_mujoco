#pragma once

#include <mujoco/mujoco.h>

#include <unitree/robot/channel/channel_publisher.hpp>
#include <unitree/robot/channel/channel_subscriber.hpp>
#include <unitree/dds_wrapper/robots/go2/go2.h>
#include <unitree/dds_wrapper/robots/g1/g1.h>
#include <unitree/idl/hg/BmsState_.hpp>
#include <unitree/idl/hg/IMUState_.hpp>

#include <opencv2/opencv.hpp>

#include <arpa/inet.h>
#include <netinet/tcp.h>
#include <sys/socket.h>
#include <unistd.h>

#include <GLFW/glfw3.h>
#include <atomic>
#include <cstring>
#include <iostream>
#include <mutex>
#include <thread>
#include <vector>

#include "param.h"
#include "physics_joystick.h"

extern GLFWwindow* g_offscreen_window;

#define MOTOR_SENSOR_NUM 3

class UnitreeSDK2BridgeBase
{
public:
    UnitreeSDK2BridgeBase(mjModel *model, mjData *data)
    : mj_model_(model), mj_data_(data)
    {
        _check_sensor();
        if(param::config.print_scene_information == 1) {
            printSceneInformation();
        }
        if(param::config.use_joystick == 1) {
            if(param::config.joystick_type == "xbox") {
                joystick = std::make_shared<XBoxJoystick>(param::config.joystick_device, param::config.joystick_bits);
            } else if(param::config.joystick_type == "switch") {
                joystick  = std::make_shared<SwitchJoystick>(param::config.joystick_device, param::config.joystick_bits);
            } else {
                std::cerr << "Unsupported joystick type: " << param::config.joystick_type << std::endl;
                exit(EXIT_FAILURE);
            }
        }

    }

    virtual void start() {}

    void printSceneInformation()
    {
        auto printObjects = [this](const char* title, int count, int type, auto getIndex) {
            std::cout << "<<------------- " << title << " ------------->> " << std::endl;
            for (int i = 0; i < count; i++) {
                const char* name = mj_id2name(mj_model_, type, i);
                if (name) {
                    std::cout << title << "_index: " << getIndex(i) << ", " << "name: " << name;
                    if (type == mjOBJ_SENSOR) {
                        std::cout << ", dim: " << mj_model_->sensor_dim[i];
                    }
                    std::cout << std::endl;
                }
            }
            std::cout << std::endl;
        };
    
        printObjects("Link", mj_model_->nbody, mjOBJ_BODY, [](int i) { return i; });
        printObjects("Joint", mj_model_->njnt, mjOBJ_JOINT, [](int i) { return i; });
        printObjects("Actuator", mj_model_->nu, mjOBJ_ACTUATOR, [](int i) { return i; });
    
        int sensorIndex = 0;
        printObjects("Sensor", mj_model_->nsensor, mjOBJ_SENSOR, [&](int i) {
            int currentIndex = sensorIndex;
            sensorIndex += mj_model_->sensor_dim[i];
            return currentIndex;
        });
    }

protected:
    int num_motor_ = 0;
    int dim_motor_sensor_ = 0;

    mjData *mj_data_;
    mjModel *mj_model_;

    // Sensor data indices
    int imu_quat_adr_ = -1;
    int imu_gyro_adr_ = -1;
    int imu_acc_adr_ = -1;
    int frame_pos_adr_ = -1;
    int frame_vel_adr_ = -1;

    int secondary_imu_quat_adr_ = -1;
    int secondary_imu_gyro_adr_ = -1;
    int secondary_imu_acc_adr_ = -1;

    std::shared_ptr<unitree::common::UnitreeJoystick> joystick = nullptr;

    void _check_sensor()
    {
        num_motor_ = mj_model_->nu;
        dim_motor_sensor_ = MOTOR_SENSOR_NUM * num_motor_;
    
        // Find sensor addresses by name
        int sensor_id = -1;
        
        // IMU quaternion
        sensor_id = mj_name2id(mj_model_, mjOBJ_SENSOR, "imu_quat");
        if (sensor_id >= 0) {
            imu_quat_adr_ = mj_model_->sensor_adr[sensor_id];
        }
        
        // IMU gyroscope
        sensor_id = mj_name2id(mj_model_, mjOBJ_SENSOR, "imu_gyro");
        if (sensor_id >= 0) {
            imu_gyro_adr_ = mj_model_->sensor_adr[sensor_id];
        }
        
        // IMU accelerometer
        sensor_id = mj_name2id(mj_model_, mjOBJ_SENSOR, "imu_acc");
        if (sensor_id >= 0) {
            imu_acc_adr_ = mj_model_->sensor_adr[sensor_id];
        }
        
        // Frame position
        sensor_id = mj_name2id(mj_model_, mjOBJ_SENSOR, "frame_pos");
        if (sensor_id >= 0) {
            frame_pos_adr_ = mj_model_->sensor_adr[sensor_id];
        }
        
        // Frame velocity
        sensor_id = mj_name2id(mj_model_, mjOBJ_SENSOR, "frame_vel");
        if (sensor_id >= 0) {
            frame_vel_adr_ = mj_model_->sensor_adr[sensor_id];
        }

        // Secondary IMU quaternion
        sensor_id = mj_name2id(mj_model_, mjOBJ_SENSOR, "secondary_imu_quat");
        if (sensor_id >= 0) {
            secondary_imu_quat_adr_ = mj_model_->sensor_adr[sensor_id];
        }

        // Secondary IMU gyroscope
        sensor_id = mj_name2id(mj_model_, mjOBJ_SENSOR, "secondary_imu_gyro");
        if (sensor_id >= 0) {
            secondary_imu_gyro_adr_ = mj_model_->sensor_adr[sensor_id];
        }

        // Secondary IMU accelerometer
        sensor_id = mj_name2id(mj_model_, mjOBJ_SENSOR, "secondary_imu_acc");
        if (sensor_id >= 0) {
            secondary_imu_acc_adr_ = mj_model_->sensor_adr[sensor_id];
        }
    }
};

template <typename LowCmd_t, typename LowState_t>
class RobotBridge : public UnitreeSDK2BridgeBase
{
using HighState_t = unitree::robot::go2::publisher::SportModeState;
using WirelessController_t = unitree::robot::go2::publisher::WirelessController;

public:
    RobotBridge(mjModel *model, mjData *data) : UnitreeSDK2BridgeBase(model, data)
    {
        lowcmd = std::make_shared<LowCmd_t>("rt/lowcmd");
        lowstate = std::make_unique<LowState_t>();
        lowstate->joystick = joystick;
        highstate = std::make_unique<HighState_t>();
        wireless_controller = std::make_unique<WirelessController_t>();
        wireless_controller->joystick = joystick;
    }

    void start()
    {
        thread_ = std::make_shared<unitree::common::RecurrentThread>(
            "unitree_bridge", UT_CPU_ID_NONE, 1000, [this]() { this->run(); });
    }

    virtual void run()
    {
        if(!mj_data_) return;
        if(lowstate->joystick) { lowstate->joystick->update(); }
        // lowcmd
        {
            std::lock_guard<std::mutex> lock(lowcmd->mutex_);
            for(int i(0); i<num_motor_; i++) {
                auto & m = lowcmd->msg_.motor_cmd()[i];
                mj_data_->ctrl[i] = m.tau() +
                                    m.kp() * (m.q() - mj_data_->sensordata[i]) +
                                    m.kd() * (m.dq() - mj_data_->sensordata[i + num_motor_]);
            }
        }

        // lowstate
        if(lowstate->trylock()) {
            for(int i(0); i<num_motor_; i++) {
                lowstate->msg_.motor_state()[i].q() = mj_data_->sensordata[i];
                lowstate->msg_.motor_state()[i].dq() = mj_data_->sensordata[i + num_motor_];
                lowstate->msg_.motor_state()[i].tau_est() = mj_data_->sensordata[i + 2 * num_motor_];
            }
            
            if(imu_quat_adr_ >= 0) {
                lowstate->msg_.imu_state().quaternion()[0] = mj_data_->sensordata[imu_quat_adr_ + 0];
                lowstate->msg_.imu_state().quaternion()[1] = mj_data_->sensordata[imu_quat_adr_ + 1];
                lowstate->msg_.imu_state().quaternion()[2] = mj_data_->sensordata[imu_quat_adr_ + 2];
                lowstate->msg_.imu_state().quaternion()[3] = mj_data_->sensordata[imu_quat_adr_ + 3];

                double w = lowstate->msg_.imu_state().quaternion()[0];
                double x = lowstate->msg_.imu_state().quaternion()[1];
                double y = lowstate->msg_.imu_state().quaternion()[2];
                double z = lowstate->msg_.imu_state().quaternion()[3];

                lowstate->msg_.imu_state().rpy()[0] = atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y));
                lowstate->msg_.imu_state().rpy()[1] = asin(2 * (w * y - z * x));
                lowstate->msg_.imu_state().rpy()[2] = atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z));
            }
            
            if(imu_gyro_adr_ >= 0) {
                lowstate->msg_.imu_state().gyroscope()[0] = mj_data_->sensordata[imu_gyro_adr_ + 0];
                lowstate->msg_.imu_state().gyroscope()[1] = mj_data_->sensordata[imu_gyro_adr_ + 1];
                lowstate->msg_.imu_state().gyroscope()[2] = mj_data_->sensordata[imu_gyro_adr_ + 2];
            }

            if(imu_acc_adr_ >= 0) {
                lowstate->msg_.imu_state().accelerometer()[0] = mj_data_->sensordata[imu_acc_adr_ + 0];
                lowstate->msg_.imu_state().accelerometer()[1] = mj_data_->sensordata[imu_acc_adr_ + 1];
                lowstate->msg_.imu_state().accelerometer()[2] = mj_data_->sensordata[imu_acc_adr_ + 2];
            }
            
            lowstate->msg_.tick() = std::round(mj_data_->time / 1e-3);
            lowstate->unlockAndPublish();
        }
        // highstate
        if(highstate->trylock()) {
            if(frame_pos_adr_ >= 0) {
                highstate->msg_.position()[0] = mj_data_->sensordata[frame_pos_adr_ + 0];
                highstate->msg_.position()[1] = mj_data_->sensordata[frame_pos_adr_ + 1];
                highstate->msg_.position()[2] = mj_data_->sensordata[frame_pos_adr_ + 2];
            }
            if(frame_vel_adr_ >= 0) {
                highstate->msg_.velocity()[0] = mj_data_->sensordata[frame_vel_adr_ + 0];
                highstate->msg_.velocity()[1] = mj_data_->sensordata[frame_vel_adr_ + 1];
                highstate->msg_.velocity()[2] = mj_data_->sensordata[frame_vel_adr_ + 2];
            }
            highstate->unlockAndPublish();
        }
        // wireless_controller
        if(wireless_controller->joystick) {
            wireless_controller->unlockAndPublish();
        }
    }

    std::unique_ptr<HighState_t> highstate;
    std::unique_ptr<WirelessController_t> wireless_controller;
    std::shared_ptr<LowCmd_t> lowcmd;
    std::unique_ptr<LowState_t> lowstate;
    
private:
    unitree::common::RecurrentThreadPtr thread_;
};

using Go2Bridge = RobotBridge<unitree::robot::go2::subscription::LowCmd, unitree::robot::go2::publisher::LowState>;

class G1Bridge : public RobotBridge<unitree::robot::g1::subscription::LowCmd, unitree::robot::g1::publisher::LowState>
{
public:
    G1Bridge(mjModel *model, mjData *data) : RobotBridge(model, data)
    {
        if (param::config.robot.find("g1") != std::string::npos) {
            auto* g1_lowstate = dynamic_cast<unitree::robot::g1::publisher::LowState*>(lowstate.get());
            if (g1_lowstate) {
                auto scene = param::config.robot_scene.filename().string();
                g1_lowstate->msg_.mode_machine() = scene.find("23") != std::string::npos ? 4 : 5;
            }
        }

        bmsstate = std::make_unique<BmsState_t>("rt/lf/bmsstate");
        bmsstate->msg_.soc() = 100;

        secondary_imustate = std::make_unique<IMUState_t>("rt/secondary_imu");
    }

    ~G1Bridge()
    {
        tcp_running_.store(false);
        if (tcp_listen_fd_ >= 0) {
            ::shutdown(tcp_listen_fd_, SHUT_RDWR);
            ::close(tcp_listen_fd_);
            tcp_listen_fd_ = -1;
        }
        if (tcp_accept_thread_.joinable()) tcp_accept_thread_.join();
        {
            std::lock_guard<std::mutex> lk(clients_mutex_);
            for (int fd : clients_) ::close(fd);
            clients_.clear();
        }
        if (cam_initialized_) {
            mjr_freeContext(&cam_con_);
            mjv_freeScene(&cam_scn_);
        }
    }

    void start()
    {
        RobotBridge::start();

        // Start head camera thread if camera exists in the model
        head_cam_id_ = mj_name2id(mj_model_, mjOBJ_CAMERA, "head_cam");
        if (head_cam_id_ >= 0 && g_offscreen_window) {
            if (!startTcpServer()) {
                std::cerr << "Head camera TCP server failed to start; camera disabled" << std::endl;
                return;
            }
            cam_thread_ = std::make_shared<unitree::common::RecurrentThread>(
                "head_camera", UT_CPU_ID_NONE, 33, [this]() { this->renderCamera(); });
            std::cout << "Head camera TCP server started on port " << TCP_PORT
                      << " (" << CAM_WIDTH << "x" << CAM_HEIGHT << " JPEG, ~30fps)" << std::endl;
        }
    }

    void run() override
    {
        RobotBridge::run();

        // secondary IMU state
        if (secondary_imustate->trylock()) {
            if(secondary_imu_quat_adr_ >= 0) {
                secondary_imustate->msg_.quaternion()[0] = mj_data_->sensordata[secondary_imu_quat_adr_ + 0];
                secondary_imustate->msg_.quaternion()[1] = mj_data_->sensordata[secondary_imu_quat_adr_ + 1];
                secondary_imustate->msg_.quaternion()[2] = mj_data_->sensordata[secondary_imu_quat_adr_ + 2];
                secondary_imustate->msg_.quaternion()[3] = mj_data_->sensordata[secondary_imu_quat_adr_ + 3];

                double w = secondary_imustate->msg_.quaternion()[0];
                double x = secondary_imustate->msg_.quaternion()[1];
                double y = secondary_imustate->msg_.quaternion()[2];
                double z = secondary_imustate->msg_.quaternion()[3];

                secondary_imustate->msg_.rpy()[0] = atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y));
                secondary_imustate->msg_.rpy()[1] = asin(2 * (w * y - z * x));
                secondary_imustate->msg_.rpy()[2] = atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z));
            }

            if(secondary_imu_gyro_adr_ >= 0) {
                secondary_imustate->msg_.gyroscope()[0] = mj_data_->sensordata[secondary_imu_gyro_adr_ + 0];
                secondary_imustate->msg_.gyroscope()[1] = mj_data_->sensordata[secondary_imu_gyro_adr_ + 1];
                secondary_imustate->msg_.gyroscope()[2] = mj_data_->sensordata[secondary_imu_gyro_adr_ + 2];
            }

            if(secondary_imu_acc_adr_ >= 0) {
                secondary_imustate->msg_.accelerometer()[0] = mj_data_->sensordata[secondary_imu_acc_adr_ + 0];
                secondary_imustate->msg_.accelerometer()[1] = mj_data_->sensordata[secondary_imu_acc_adr_ + 1];
                secondary_imustate->msg_.accelerometer()[2] = mj_data_->sensordata[secondary_imu_acc_adr_ + 2];
            }

            secondary_imustate->unlockAndPublish();
        }

        // In practice, bmsstate is sent at a low frequency; here it is sent with the main loop
        bmsstate->unlockAndPublish();
    }

    using BmsState_t = unitree::robot::RealTimePublisher<unitree_hg::msg::dds_::BmsState_>;
    using IMUState_t = unitree::robot::RealTimePublisher<unitree_hg::msg::dds_::IMUState_>;
    std::unique_ptr<BmsState_t> bmsstate;
    std::unique_ptr<IMUState_t> secondary_imustate;

private:
    // Head camera resolution — matches real camera_streamer.py wire format
    // so the ROS 2 theia_tiny_node treats sim and real identically.
    static constexpr int CAM_WIDTH = 640;
    static constexpr int CAM_HEIGHT = 480;
    static constexpr int JPEG_QUALITY = 80;
    static constexpr uint16_t TCP_PORT = 5555;

    // Camera state
    int head_cam_id_ = -1;
    bool cam_initialized_ = false;
    mjvScene cam_scn_;
    mjvCamera cam_cam_;
    mjvOption cam_opt_;
    mjrContext cam_con_;
    mjrRect cam_viewport_ = {0, 0, CAM_WIDTH, CAM_HEIGHT};
    std::vector<unsigned char> cam_rgb_;  // bottom-up RGB from mjr_readPixels
    unitree::common::RecurrentThreadPtr cam_thread_;

    // TCP broadcast state
    int tcp_listen_fd_ = -1;
    std::atomic<bool> tcp_running_{false};
    std::thread tcp_accept_thread_;
    std::vector<int> clients_;
    std::mutex clients_mutex_;

    bool startTcpServer()
    {
        tcp_listen_fd_ = ::socket(AF_INET, SOCK_STREAM, 0);
        if (tcp_listen_fd_ < 0) return false;

        int yes = 1;
        ::setsockopt(tcp_listen_fd_, SOL_SOCKET, SO_REUSEADDR, &yes, sizeof(yes));

        sockaddr_in addr{};
        addr.sin_family = AF_INET;
        addr.sin_addr.s_addr = htonl(INADDR_ANY);
        addr.sin_port = htons(TCP_PORT);
        if (::bind(tcp_listen_fd_, reinterpret_cast<sockaddr *>(&addr), sizeof(addr)) < 0) {
            std::cerr << "bind(" << TCP_PORT << ") failed: " << std::strerror(errno) << std::endl;
            ::close(tcp_listen_fd_);
            tcp_listen_fd_ = -1;
            return false;
        }
        if (::listen(tcp_listen_fd_, 5) < 0) {
            ::close(tcp_listen_fd_);
            tcp_listen_fd_ = -1;
            return false;
        }

        tcp_running_.store(true);
        tcp_accept_thread_ = std::thread([this]() {
            while (tcp_running_.load()) {
                sockaddr_in cli{};
                socklen_t cli_len = sizeof(cli);
                int fd = ::accept(tcp_listen_fd_, reinterpret_cast<sockaddr *>(&cli), &cli_len);
                if (fd < 0) {
                    if (!tcp_running_.load()) break;
                    continue;
                }
                int one = 1;
                ::setsockopt(fd, IPPROTO_TCP, TCP_NODELAY, &one, sizeof(one));
                std::lock_guard<std::mutex> lk(clients_mutex_);
                clients_.push_back(fd);
                std::cout << "Head camera client connected (total: " << clients_.size() << ")" << std::endl;
            }
        });
        return true;
    }

    void initCamera()
    {
        if (!g_offscreen_window || head_cam_id_ < 0) return;

        glfwMakeContextCurrent(g_offscreen_window);

        mjv_defaultScene(&cam_scn_);
        mjv_makeScene(mj_model_, &cam_scn_, 2000);
        mjr_makeContext(mj_model_, &cam_con_, mjFONTSCALE_100);

        mjv_defaultCamera(&cam_cam_);
        cam_cam_.type = mjCAMERA_FIXED;
        cam_cam_.fixedcamid = head_cam_id_;

        mjv_defaultOption(&cam_opt_);

        cam_rgb_.resize(CAM_WIDTH * CAM_HEIGHT * 3);

        cam_initialized_ = true;
        std::cout << "Head camera initialized (" << CAM_WIDTH << "x" << CAM_HEIGHT << ")" << std::endl;
    }

    void renderCamera()
    {
        if (!cam_initialized_) {
            initCamera();
            if (!cam_initialized_) return;
        }
        if (!mj_data_) return;

        // Skip the render + encode entirely if no one is listening.
        {
            std::lock_guard<std::mutex> lk(clients_mutex_);
            if (clients_.empty()) return;
        }

        // Render offscreen, read bottom-up RGB.
        mjv_updateScene(mj_model_, mj_data_, &cam_opt_, nullptr, &cam_cam_, mjCAT_ALL, &cam_scn_);
        mjr_setBuffer(mjFB_OFFSCREEN, &cam_con_);
        mjr_render(cam_viewport_, &cam_scn_, &cam_con_);
        mjr_readPixels(cam_rgb_.data(), nullptr, cam_viewport_, &cam_con_);

        // Wrap as RGB, convert to BGR, flip vertical (mjr_readPixels is bottom-up),
        // then JPEG-encode. BGR matches camera_streamer.py's realsense output so
        // the receiver's channel handling is identical for sim and real.
        cv::Mat rgb(CAM_HEIGHT, CAM_WIDTH, CV_8UC3, cam_rgb_.data());
        cv::Mat bgr;
        cv::cvtColor(rgb, bgr, cv::COLOR_RGB2BGR);
        cv::flip(bgr, bgr, 0);

        std::vector<uchar> jpg;
        cv::imencode(".jpg", bgr, jpg, {cv::IMWRITE_JPEG_QUALITY, JPEG_QUALITY});

        uint32_t length = static_cast<uint32_t>(jpg.size());
        std::vector<uint8_t> payload(4 + jpg.size());
        std::memcpy(payload.data(), &length, 4);  // little-endian on x86/ARM
        std::memcpy(payload.data() + 4, jpg.data(), jpg.size());

        // Broadcast to all clients; drop any that fail.
        std::lock_guard<std::mutex> lk(clients_mutex_);
        for (auto it = clients_.begin(); it != clients_.end(); ) {
            size_t sent = 0;
            bool ok = true;
            while (sent < payload.size()) {
                ssize_t n = ::send(*it, payload.data() + sent, payload.size() - sent, MSG_NOSIGNAL);
                if (n <= 0) { ok = false; break; }
                sent += static_cast<size_t>(n);
            }
            if (!ok) {
                ::close(*it);
                it = clients_.erase(it);
                std::cout << "Head camera client disconnected (remaining: " << clients_.size() << ")" << std::endl;
            } else {
                ++it;
            }
        }
    }
};
