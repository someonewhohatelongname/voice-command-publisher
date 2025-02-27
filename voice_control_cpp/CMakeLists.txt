cmake_minimum_required(VERSION 3.8)
project(voice_control_cpp)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(px4_msgs REQUIRED)
find_package(rcl_interfaces REQUIRED)  


# Specify the path to the Porcupine library and include files
set(PORCUPINE_LIB_DIR "/home/ubuntu/Documents/ws_ros2/src/porcupine/lib/linux/x86_64")  # Path to the Porcupine library
set(PORCUPINE_INCLUDE_DIR "/home/ubuntu/Documents/ws_ros2/src/porcupine/include")  # Path to the Porcupine headers
set(WHISPER_DIR "/home/ubuntu/Documents/ws_ros2/src/whisper.cpp")

set(WHISPER_BUILD_DIR "${WHISPER_DIR}/build")


# Find the Porcupine library
find_library(PORCUPINE_LIB
  NAMES pv_porcupine libpv_porcupine.so  # Add both names to cover all possibilities
  PATHS ${PORCUPINE_LIB_DIR}
  REQUIRED
)

find_library(WHISPER_LIB
  NAMES whisper libwhisper.so
  PATHS ${WHISPER_BUILD_DIR}/src
  NO_DEFAULT_PATH  # Only search in specified paths
  REQUIRED
)

# Also need ggml libraries
find_library(GGML_LIB
  NAMES ggml libggml.so
  PATHS ${WHISPER_BUILD_DIR}/ggml/src
  NO_DEFAULT_PATH
  REQUIRED
)

find_package(PkgConfig REQUIRED)
pkg_check_modules(PORTAUDIO REQUIRED portaudio-2.0)

# Include directories
# Add this to your include_directories
include_directories(
  ${rclcpp_INCLUDE_DIRS}
  ${px4_msgs_INCLUDE_DIRS}
  ${PORCUPINE_INCLUDE_DIR}
  ${PORTAUDIO_INCLUDE_DIRS}
  /home/ubuntu/Documents/ws_ros2/src/whisper.cpp/  # Path to whisper.cpp
  /home/ubuntu/Documents/ws_ros2/src/whisper.cpp/include/  # Path to whisper.h
  /home/ubuntu/Documents/ws_ros2/src/whisper.cpp/ggml/include/  # Path to ggml.h
  /home/ubuntu/Documents/ws_ros2/src/whisper.cpp/examples/  # Path to common.h
  /usr/include/SDL2  # Path to sdl.h
  )

# Declare the executable
add_executable(voice_control_node src/voice_control_node.cpp)
add_executable(voice_offboard_node src/voice_offboard_node.cpp)
add_executable(whisper_offboard_node src/whisper_offboard_node.cpp)

# Link libraries
target_link_libraries(voice_control_node
  ${rclcpp_LIBRARIES}
  ${px4_msgs_LIBRARIES}
  ${PORCUPINE_LIB}
  ${PORTAUDIO_LIBRARIES}
  
)

target_link_libraries(voice_offboard_node
  ${rclcpp_LIBRARIES}
  ${px4_msgs_LIBRARIES}
  ${rcl_interfaces_LIBRARIES}  # Add this line
  ${PORCUPINE_LIB}
  ${PORTAUDIO_LIBRARIES}
)

target_link_libraries(whisper_offboard_node
  ${rclcpp_LIBRARIES}
  ${px4_msgs_LIBRARIES}
  ${rcl_interfaces_LIBRARIES}  # Add this line
  ${PORCUPINE_LIB}
  ${PORTAUDIO_LIBRARIES}
  ${WHISPER_LIB}

)

install(TARGETS
  voice_control_node
  voice_offboard_node
  whisper_offboard_node
  DESTINATION lib/${PROJECT_NAME}
)
ament_package()