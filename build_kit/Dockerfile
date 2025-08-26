# ── Build stage ────────────────────────────────────────────────────────────────
ARG ROS_DISTRO=jazzy
ARG ENABLE_JPEG_COMPRESSION=OFF
FROM ros:${ROS_DISTRO}-ros-base AS builder
SHELL ["/bin/bash", "-c"]
ENV DEBIAN_FRONTEND=noninteractive

# Build essentials + colcon + FFmpeg development libraries
RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential cmake python3-colcon-common-extensions \
    pkg-config \
    libavcodec-dev \
    libavformat-dev \
    libavutil-dev \
    libswscale-dev \
    && rm -rf /var/lib/apt/lists/*

# Workspace lives at /opt/ws; your Docker build context is the repo root (.)
WORKDIR /opt/ws
COPY . .

# Build in Release, merge-install makes a single overlay dir
RUN source /opt/ros/${ROS_DISTRO}/setup.bash && \
    colcon build \
    --merge-install \
    --cmake-args -DCMAKE_BUILD_TYPE=Release -DENABLE_JPEG_COMPRESSION=${ENABLE_JPEG_COMPRESSION}

# ── Runtime stage ──────────────────────────────────────────────────────────────
FROM ros:${ROS_DISTRO}-ros-core
SHELL ["/bin/bash", "-c"]

# Install runtime dependencies including FFmpeg runtime libraries
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-${ROS_DISTRO}-rmw-zenoh-cpp \
    jq \
    libavcodec60 \
    libavformat60 \
    libavutil58 \
    libswscale7 \
    && rm -rf /var/lib/apt/lists/*

# Copy only the built install tree
COPY --from=builder /opt/ws/install /opt/ws/install

# Copy the entrypoint script
COPY build_kit/entrypoint.sh /usr/local/bin/entrypoint.sh
RUN chmod +x /usr/local/bin/entrypoint.sh

# Optional: keep a stable domain
ENV ROS_DOMAIN_ID=87
ENV RMW_IMPLEMENTATION=rmw_zenoh_cpp

# Set production paths for the entrypoint script
ENV PROJECT_DIR=/opt/ws
ENV ROS_ENTRYPOINT=/ros_entrypoint.sh
ENV SETUP_SCRIPT=install/setup.bash
ENV PACKAGE_NAME=make87_camera_driver_cpp
ENV NODE_NAME=camera_driver

# Use our custom entrypoint script
ENTRYPOINT ["/usr/local/bin/entrypoint.sh"]
