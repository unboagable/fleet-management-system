# Use a valid ROS2 Humble base image
FROM ros:humble

# Set up workspace
WORKDIR /ros2_ws

# Install system dependencies
RUN apt update && apt install -y \
    python3-pip \
    python3-colcon-common-extensions \
    ros-humble-rclpy \
    ros-humble-rclcpp-action \
    ros-humble-example-interfaces

# Ensure src/ exists before installing dependencies
COPY src/ src/

# Install ROS2 dependencies
RUN rosdep update && rosdep install --from-paths src --ignore-src -r -y

# Copy the rest of the ROS2 workspace
COPY . .

# Install Python dependencies for MQTT
COPY requirements.txt .
RUN pip install --no-cache-dir -r requirements.txt

# Source ROS2 and build the workspace
RUN /bin/bash -c "source /opt/ros/humble/setup.bash && colcon build"
