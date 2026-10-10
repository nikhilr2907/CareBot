# 1. Base image: Official ROS 2 Humble (based on Ubuntu 22.04)
FROM osrf/ros:humble-desktop

# 2. Avoid timezone prompt freezing during installation
ENV DEBIAN_FRONTEND=noninteractive

# 3. Install essential development tools and CycloneDDS
RUN apt-get update && apt-get install -y \
    python3-pip \
    git \
    nano \
    htop \
    ros-humble-rmw-cyclonedds-cpp \
    # Add more libraries here as needed, e.g., OpenCV
    && rm -rf /var/lib/apt/lists/*

# 4. Force specify DDS to CycloneDDS (to solve communication lag/freezing)
ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

# 5. Automatically source ROS 2 environment for each new terminal
RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
# Also source custom compiled packages if they exist
RUN echo "if [ -f /workspace/install/setup.bash ]; then source /workspace/install/setup.bash; fi" >> ~/.bashrc

# 6. Set default working directory
WORKDIR /workspace