cmake_minimum_required(VERSION 3.8)
project(voice_control_cpp)

# Add compile options
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Set C++ standard
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Specify the path to the Porcupine library and include files
set(PORCUPINE_LIB_DIR "/home/raspberry/porcupine/lib/raspberry-pi/cortex-a72-aarch64")  # Changed to ARM path
set(PORCUPINE_INCLUDE_DIR "/home/raspberry/porcupine/include")

# Find the Porcupine library
find_library(PORCUPINE_LIB
  NAMES pv_porcupine libpv_porcupine.so
  PATHS ${PORCUPINE_LIB_DIR}
  REQUIRED
)

# Find PortAudio
find_package(PkgConfig REQUIRED)
pkg_check_modules(PORTAUDIO REQUIRED portaudio-2.0)

# Include directories
include_directories(
  ${PORCUPINE_INCLUDE_DIR}
  ${PORTAUDIO_INCLUDE_DIRS}
)

# Declare the executables
add_executable(voice_control src/voice_control_node.cpp)
add_executable(voice_offboard src/voice_offboard_node.cpp)  # Add this new executable

# Link libraries for voice control
target_link_libraries(voice_control
  ${PORCUPINE_LIB}
  ${PORTAUDIO_LIBRARIES}
  pthread
)

# Link libraries for voice offboard (if any different libraries needed, add them)
target_link_libraries(voice_offboard
  ${PORCUPINE_LIB}
  ${PORTAUDIO_LIBRARIES}
  pthread
)

# Install the executables
install(TARGETS
  voice_control
  voice_offboard  # Add voice_offboard to installation
  DESTINATION bin
)
