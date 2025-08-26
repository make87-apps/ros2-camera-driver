cmake_minimum_required(VERSION 3.16)
project(make87_camera_driver_cpp)

# Build option for output format
option(ENABLE_JPEG_COMPRESSION "Enable JPEG compression output (ImageCompressed) instead of raw RGB8 (Image)" OFF)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(PkgConfig REQUIRED)

# Find FFmpeg libraries
pkg_check_modules(LIBAV REQUIRED IMPORTED_TARGET
    libavcodec
    libavformat
    libavutil
    libswscale
)

# Fetch nlohmann/json
include(FetchContent)
FetchContent_Declare(
  nlohmann_json
  GIT_REPOSITORY https://github.com/nlohmann/json.git
  GIT_TAG v3.11.3
)
FetchContent_MakeAvailable(nlohmann_json)

add_executable(camera_driver src/camera_driver.cpp)
ament_target_dependencies(camera_driver rclcpp sensor_msgs)
target_link_libraries(camera_driver nlohmann_json::nlohmann_json PkgConfig::LIBAV)

# Configure compression mode
if(ENABLE_JPEG_COMPRESSION)
    target_compile_definitions(camera_driver PRIVATE ENABLE_JPEG_COMPRESSION=1)
    message(STATUS "Building with JPEG compression (ImageCompressed output)")
else()
    message(STATUS "Building with raw RGB8 (Image output)")
endif()

install(TARGETS camera_driver
  RUNTIME DESTINATION lib/${PROJECT_NAME})

ament_package()
