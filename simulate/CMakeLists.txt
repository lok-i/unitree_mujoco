cmake_minimum_required(VERSION 3.16)
project(unitree_mujoco)

if (CMAKE_BUILD_TYPE MATCHES "Debug")
  message("Debug mode")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++17 -g -O0 -fPIC")
else()
  message("Release mode")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++17 -O3 -DNDEBUG -fPIC")
endif()

list(APPEND CMAKE_PREFIX_PATH "/opt/unitree_robotics/lib/cmake")
find_package(unitree_sdk2 REQUIRED)

# Pin the unitree SDK's own CycloneDDS (libddsc) with classic DT_RPATH, which
# the loader searches BEFORE LD_LIBRARY_PATH. Otherwise a sourced ROS shell
# shadows it with /opt/ros/<distro>'s ABI-incompatible libddsc ->
# "free(): invalid pointer" at bridge init. (sudo never hit this: it scrubs
# LD_LIBRARY_PATH, which is why the crash only appears when run unsudo'd.)
set(CMAKE_EXE_LINKER_FLAGS
    "${CMAKE_EXE_LINKER_FLAGS} -Wl,--disable-new-dtags -Wl,-rpath,/opt/unitree_robotics/lib")
find_package(Boost REQUIRED COMPONENTS program_options)
find_package(OpenCV REQUIRED)


# The head-camera wire format is defined once, in vision_encoders, and shared by
# every producer and consumer. Header-only, so this is an include path and not a
# dependency; override with -DFRAME_WIRE_INCLUDE_DIR=... for an out-of-tree ws.
set(FRAME_WIRE_INCLUDE_DIR
    "${CMAKE_CURRENT_SOURCE_DIR}/../../cyclonedds_ws/src/vision_encoders/include"
    CACHE PATH "Directory containing vision_encoders/frame_wire.hpp")
if(NOT EXISTS "${FRAME_WIRE_INCLUDE_DIR}/vision_encoders/frame_wire.hpp")
  message(FATAL_ERROR
    "frame_wire.hpp not found under ${FRAME_WIRE_INCLUDE_DIR}\n"
    "Set -DFRAME_WIRE_INCLUDE_DIR=<ws>/src/vision_encoders/include")
endif()

include_directories(
  mujoco/include
  mujoco/simulate
  src/lodepng
  ${FRAME_WIRE_INCLUDE_DIR}
  ${OpenCV_INCLUDE_DIRS}
)
link_directories(mujoco/lib)

file(GLOB SIM_SRC
  src/joystick/joystick.cc

  # exclude mujoco/simulate/main.cc
  mujoco/simulate/glfw_*.cc
  mujoco/simulate/platform_*.cc
  mujoco/simulate/simulate.cc
  src/lodepng/lodepng.cpp
)

link_libraries(
  pthread
  mujoco
  glfw
  yaml-cpp
  unitree_sdk2
  boost_program_options
  fmt
  ${OpenCV_LIBS}
)

add_executable(unitree_mujoco ${SIM_SRC} src/main.cc)

add_executable(jstest src/joystick/jstest.cc src/joystick/joystick.cc)