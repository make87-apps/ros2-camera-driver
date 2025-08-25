# ROS2 Jazzy Camera Driver

A generic ROS2 C++ camera driver for publishing IP camera frames on the ROS2 middleware with make87 platform integration. This application captures and publishes camera frames to a configurable ROS2 topic.

## What This Application Demonstrates

This camera driver application showcases several key concepts and technologies:

### ROS2 Fundamentals
- **Publisher Node**: Creates a ROS2 node that publishes camera frame messages
- **Real-time Publishing**: Captures and publishes camera frames in real-time
- **Topic Communication**: Demonstrates ROS2 publish/subscribe communication for camera data
- **Node Lifecycle**: Shows proper ROS2 node initialization and shutdown procedures

### make87 Platform Integration
- **Dynamic Topic Configuration**: Reads topic names from the `MAKE87_CONFIG` environment variable
- **JSON Configuration Parsing**: Uses nlohmann/json library to parse platform configuration
- **Topic Name Sanitization**: Automatically converts topic names to ROS2-compatible format

### Containerization & Deployment
- **Multi-Stage Docker Build**: Optimized Dockerfile with separate build and runtime stages
- **Zenoh Networking**: Configured for Zenoh-based ROS2 communication middleware
- **Production Ready**: Includes proper entrypoint scripts and environment configuration
- **Development Support**: Includes development container configuration with SSH access

## Key Features

- 📷 **IP Camera Support**: Connects to and captures frames from IP cameras
- 📡 **Configurable Publishing**: Topic name determined by make87 platform configuration
- 🔄 **Robust Configuration**: Automatic fallback to default topic if configuration is missing
- 🐳 **Container Ready**: Fully containerized with Docker support
- 🌐 **Network Optimized**: Uses Zenoh middleware for efficient communication
- 🛠️ **Development Friendly**: Includes development container with git integration
- 📝 **Comprehensive Logging**: Detailed logging for debugging and monitoring

### Topic Name Resolution

1. **With Configuration**: If `MAKE87_CONFIG` is provided and contains a valid `topic_key`, the application will:
   - Extract the topic key
   - Replace dashes with underscores for ROS2 compatibility
   - Prefix with "make87_" (e.g., "my-topic" becomes "make87_my_topic")

2. **Default Fallback**: If no configuration is provided or parsing fails, the application uses the default topic "camera_frames"

## Message Format

The application publishes camera frame messages containing image data from the connected IP camera.

## Networking

This application is configured to use:
- **ROS Domain ID**: 87 (default)
- **Middleware**: Zenoh (`rmw_zenoh_cpp`)
- **Default Port**: 7447 for Zenoh communication

## Development Environment

The development container includes:
- SSH server for remote development
- Git integration with automatic repository management


## Technical Details

- **Language**: C++17
- **ROS2 Distribution**: Jazzy
- **Build System**: CMake with ament
- **Dependencies**: rclcpp, sensor_msgs, cv_bridge, OpenCV, nlohmann/json
- **Container Base**: ros:jazzy-ros-core
- **Camera Support**: IP cameras via network streams

## Use Cases

This application serves as:
- **Camera Integration**: Connect IP cameras to ROS2 systems
- **Computer Vision Pipeline**: Foundation for vision-based robotics applications
- **Security Systems**: Camera feed integration for monitoring applications
- **Robotics Perception**: Camera data input for autonomous systems
- **Integration Example**: Template for make87 platform camera integration

## License

Apache-2.0 - See LICENSE file for details.

## Maintainer

make87 <nisse@make87.com>
