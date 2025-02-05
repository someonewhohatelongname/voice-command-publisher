cmake_minimum_required(VERSION 3.8)
project(drone_gripper)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(px4_msgs REQUIRED)
find_package(rcl_interfaces REQUIRED)

set(PORCUPINE_LIB_DIR "/home/hn/porcupine/lib/linux/x86_64")
set(PORCUPINE_INCLUDE_DIR "/home/hn/porcupine/include")

# Find the Porcupine library
find_library(PORCUPINE_LIB
  NAMES pv_porcupine libpv_porcupine.so
  PATHS ${PORCUPINE_LIB_DIR}
  REQUIRED
)

find_package(PkgConfig REQUIRED)
pkg_check_modules(PORTAUDIO REQUIRED portaudio-2.0)

# Include directories
include_directories(
  ${rclcpp_INCLUDE_DIRS}
  ${px4_msgs_INCLUDE_DIRS}
  ${PORCUPINE_INCLUDE_DIR}
  ${PORTAUDIO_INCLUDE_DIRS}
)

# Create executables with unique names
add_executable(actuator_control src/actuator_control.cpp)
add_executable(voice_actuator_control src/voice_actuator_control.cpp)

# Link libraries and dependencies for actuator_control
ament_target_dependencies(actuator_control
  rclcpp
  px4_msgs
)

# Link libraries and dependencies for voice_actuator_control
ament_target_dependencies(voice_actuator_control
  rclcpp
  px4_msgs
  rcl_interfaces
)

target_link_libraries(voice_actuator_control
  ${PORCUPINE_LIB}
  ${PORTAUDIO_LIBRARIES}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

# Install targets
install(TARGETS
  actuator_control
  voice_actuator_control
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
