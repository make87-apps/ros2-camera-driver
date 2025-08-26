cmake_minimum_required(VERSION 3.16)
project(make87_camera_driver_cpp)

# Build option for output format
option(ENABLE_JPEG_COMPRESSION "Enable JPEG compression output (ImageCompressed) instead of raw RGB8 (Image)" OFF)

# Debug: Print the value of ENABLE_JPEG_COMPRESSION
message(STATUS "ENABLE_JPEG_COMPRESSION option value: ${ENABLE_JPEG_COMPRESSION}")
message(STATUS "All cache variables containing JPEG:")
get_cmake_property(_variableNames CACHE_PROPERTIES)
foreach(_variableName ${_variableNames})
    if(_variableName MATCHES "JPEG")
        message(STATUS "  ${_variableName}")
    endif()
endforeach()

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

# Configure compression mode - force it based on command line arg
message(STATUS "Before if: ENABLE_JPEG_COMPRESSION = ${ENABLE_JPEG_COMPRESSION}")
if(ENABLE_JPEG_COMPRESSION STREQUAL "ON")
    target_compile_definitions(camera_driver PRIVATE ENABLE_JPEG_COMPRESSION)
    message(STATUS "Building with JPEG compression (ImageCompressed output) - FORCED ON")
elseif(ENABLE_JPEG_COMPRESSION)
    target_compile_definitions(camera_driver PRIVATE ENABLE_JPEG_COMPRESSION)
    message(STATUS "Building with JPEG compression (ImageCompressed output) - OPTION ON")
else()
    target_compile_definitions(camera_driver PRIVATE ENABLE_RGB8_OUTPUT)
    message(STATUS "Building with raw RGB8 (Image output)")
endif()

install(TARGETS camera_driver
  RUNTIME DESTINATION lib/${PROJECT_NAME})

ament_package()
