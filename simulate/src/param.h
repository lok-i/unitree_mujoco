#pragma once

#include <iostream>
#include <boost/program_options.hpp>
#include <yaml-cpp/yaml.h>
#include <filesystem>
#include <stdexcept>

namespace param
{

constexpr int IDL_AUTO = -1;
constexpr int IDL_GO = 0;
constexpr int IDL_HG = 1;
constexpr int IDL_GO_MOTOR_LIMIT = 20;
constexpr int IDL_HG_MOTOR_LIMIT = 35;

inline int ResolveIdlType(const std::string &robot, int actuator_count, int configured_idl_type)
{
    if (configured_idl_type != IDL_AUTO &&
        configured_idl_type != IDL_GO &&
        configured_idl_type != IDL_HG)
    {
        throw std::invalid_argument("idl_type must be -1 (auto), 0 (unitree_go), or 1 (unitree_hg)");
    }

    int idl_type = configured_idl_type;
    if (idl_type == IDL_AUTO)
    {
        idl_type = robot == "as2" || actuator_count > IDL_GO_MOTOR_LIMIT ? IDL_HG : IDL_GO;
    }

    const int motor_limit = idl_type == IDL_GO ? IDL_GO_MOTOR_LIMIT : IDL_HG_MOTOR_LIMIT;
    if (actuator_count > motor_limit)
    {
        throw std::invalid_argument(
            std::string(idl_type == IDL_GO ? "unitree_go" : "unitree_hg") +
            " IDL supports at most " + std::to_string(motor_limit) +
            " actuators, but the model has " + std::to_string(actuator_count));
    }

    return idl_type;
}

inline struct SimulationConfig
{
    std::string robot;
    std::filesystem::path robot_scene;

    int domain_id;
    std::string interface;
    int idl_type = IDL_AUTO;

    int use_joystick;
    std::string joystick_type;
    std::string joystick_device;
    int joystick_bits;

    int print_scene_information;

    int enable_elastic_band;
    int band_attached_link = 0;

    int camera_port = 5555;
    int camera_fps = 30;
    int camera_depth = 1;
    int camera_depth_width = 160;
    int camera_depth_height = 120;

    void validate_camera() const {
      if (camera_port < 1 || camera_port > 65535) {
        throw std::invalid_argument("camera_port must be in [1, 65535]");
      }
      if (camera_fps < 1) {
        throw std::invalid_argument("camera_fps must be positive");
      }
      if (camera_depth_width < 1 || camera_depth_height < 1) {
        throw std::invalid_argument("camera depth dimensions must be positive");
      }
    }

    void load_from_yaml(const std::string &filename)
    {
        auto cfg = YAML::LoadFile(filename);
        try
        {
            robot = cfg["robot"].as<std::string>();
            robot_scene = cfg["robot_scene"].as<std::string>();
            domain_id = cfg["domain_id"].as<int>();
            interface = cfg["interface"].as<std::string>();
            if (cfg["idl_type"])
            {
                idl_type = cfg["idl_type"].as<int>();
            }
            use_joystick = cfg["use_joystick"].as<int>();
            joystick_type = cfg["joystick_type"].as<std::string>();
            joystick_device = cfg["joystick_device"].as<std::string>();
            joystick_bits = cfg["joystick_bits"].as<int>();
            print_scene_information = cfg["print_scene_information"].as<int>();
            enable_elastic_band = cfg["enable_elastic_band"].as<int>();
            if (cfg["camera_port"]) {
              camera_port = cfg["camera_port"].as<int>();
            }
            if (cfg["camera_fps"]) {
              camera_fps = cfg["camera_fps"].as<int>();
            }
            if (cfg["camera_depth"]) {
              camera_depth = cfg["camera_depth"].as<int>();
            }
            if (cfg["camera_depth_width"]) {
              camera_depth_width = cfg["camera_depth_width"].as<int>();
            }
            if (cfg["camera_depth_height"]) {
              camera_depth_height = cfg["camera_depth_height"].as<int>();
            }

            validate_camera();
        }
        catch(const std::exception& e)
        {
            std::cerr << e.what() << '\n';
            exit(EXIT_FAILURE);
        }
    }
} config;

/* ---------- Command Line Parameters ---------- */
namespace po = boost::program_options;

//※ This function must be called at the beginning of main() function
inline po::variables_map helper(int argc, char** argv)
{
    po::options_description desc("Unitree Mujoco");
    desc.add_options()("help,h", "Show help message")(
        "domain_id,i", po::value<int>(&config.domain_id),
        "DDS domain ID; -i 0")("network,n",
                               po::value<std::string>(&config.interface),
                               "DDS network interface; -n eth0")(
        "robot,r", po::value<std::string>(&config.robot), "Robot type; -r go2")(
        "scene,s", po::value<std::filesystem::path>(&config.robot_scene),
        "Robot scene file; -s scene_terrain.xml")(
        "idl_type,t", po::value<int>(&config.idl_type),
        "DDS IDL type: -1 auto, 0 unitree_go, 1 unitree_hg")(
        "camera_port", po::value<int>(&config.camera_port),
        "Head-camera TCP port")("camera_fps",
                                po::value<int>(&config.camera_fps),
                                "Head-camera frame rate");

    po::variables_map vm;
    po::store(po::parse_command_line(argc, argv, desc), vm);
    po::notify(vm);
    config.validate_camera();

    if (vm.count("help"))
    {
        std::cout << desc << std::endl;
        exit(0);
    }

    return vm;
}

}  // namespace param
