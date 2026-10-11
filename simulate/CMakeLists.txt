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
find_package(Boost REQUIRED COMPONENTS program_options)

set(MUJOCO_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/mujoco" CACHE PATH
    "MuJoCo tree containing include/, lib/ (or build/lib/), and simulate/")

set(MUJOCO_INCLUDE_DIR "${MUJOCO_ROOT}/include")
set(MUJOCO_SIMULATE_DIR "${MUJOCO_ROOT}/simulate")
if(NOT EXISTS "${MUJOCO_INCLUDE_DIR}/mujoco/mujoco.h" OR
   NOT EXISTS "${MUJOCO_SIMULATE_DIR}/simulate.cc")
  message(FATAL_ERROR
    "MuJoCo is incomplete at ${MUJOCO_ROOT}. "
    "Run ${CMAKE_CURRENT_SOURCE_DIR}/../scripts/setup_mujoco.sh first, or "
    "configure with -DMUJOCO_ROOT=/path/to/mujoco.")
endif()

unset(MUJOCO_LIBRARY CACHE)
find_library(MUJOCO_LIBRARY
  NAMES mujoco
  PATHS "${MUJOCO_ROOT}/lib" "${MUJOCO_ROOT}/build/lib"
  NO_DEFAULT_PATH)
if(NOT MUJOCO_LIBRARY)
  message(FATAL_ERROR
    "libmujoco was not found below ${MUJOCO_ROOT}/lib or "
    "${MUJOCO_ROOT}/build/lib. Run scripts/setup_mujoco.sh.")
endif()
get_filename_component(MUJOCO_LIBRARY_DIR "${MUJOCO_LIBRARY}" DIRECTORY)
message(STATUS "Using MuJoCo: ${MUJOCO_LIBRARY}")

file(GLOB SIM_SRC
  src/joystick/joystick.cc
  src/head_camera_server.cc

  # exclude mujoco/simulate/main.cc
  ${MUJOCO_SIMULATE_DIR}/glfw_*.cc
  ${MUJOCO_SIMULATE_DIR}/platform_*.cc
  ${MUJOCO_SIMULATE_DIR}/simulate.cc
  src/lodepng/lodepng.cpp
)

add_executable(unitree_mujoco ${SIM_SRC} src/main.cc)
target_include_directories(unitree_mujoco PRIVATE
  /usr/local/include/iceoryx/v2.0.2
  ${MUJOCO_INCLUDE_DIR}
  ${MUJOCO_SIMULATE_DIR}
  src/lodepng
)
target_link_libraries(unitree_mujoco PRIVATE
  pthread
  ${MUJOCO_LIBRARY}
  glfw
  yaml-cpp
  unitree_sdk2
  boost_program_options
  fmt
)

# The Unitree SDK ships its own CycloneDDS build. Prefer it over a sourced ROS
# overlay's ABI-incompatible libddsc at runtime. Pin MuJoCo's runtime library to
# the same tree used at compile time as well.
if(UNIX AND NOT APPLE)
  target_link_options(unitree_mujoco PRIVATE
    "-Wl,--disable-new-dtags"
    "-Wl,-rpath,/opt/unitree_robotics/lib"
    "-Wl,-rpath,${MUJOCO_LIBRARY_DIR}")
endif()

add_executable(jstest src/joystick/jstest.cc src/joystick/joystick.cc)
target_link_libraries(jstest PRIVATE pthread)

set(MUJOCO_PLUGIN_DIR "${MUJOCO_ROOT}/bin/mujoco_plugin")
if(EXISTS "${MUJOCO_PLUGIN_DIR}")
  add_custom_command(TARGET unitree_mujoco POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy_directory
            "${MUJOCO_PLUGIN_DIR}"
            "$<TARGET_FILE_DIR:unitree_mujoco>/mujoco_plugin"
    COMMENT "Copying MuJoCo plugins")
endif()

include(CTest)
if(BUILD_TESTING)
  add_executable(camera_integration_test test/camera_integration_test.cc)
  target_include_directories(camera_integration_test PRIVATE
    ${MUJOCO_INCLUDE_DIR}
    src)
  target_compile_definitions(camera_integration_test PRIVATE
    G1_MODEL_DIR="${CMAKE_CURRENT_SOURCE_DIR}/../unitree_robots/g1")
  target_link_libraries(camera_integration_test PRIVATE ${MUJOCO_LIBRARY})
  add_test(NAME camera_integration_test COMMAND camera_integration_test)
endif()
