#!/bin/bash
set -e

echo "Setting up ROS Sensor Monitoring System..."

# Check if ROS is installed
if ! command -v roscore > /dev/null; then
  echo "ROS is not installed. Please install ROS first."
  exit 1
fi

# Create ROS workspace
WORKSPACE_DIR=~/ros_sensor_monitor
echo "Creating workspace at $WORKSPACE_DIR"
mkdir -p $WORKSPACE_DIR/src
cd $WORKSPACE_DIR

# Clone this repository into src directory
echo "Cloning project repository..."
cd src
git clone https://github.com/abhinavgulisetty/ros-sensor-monitor.git sensor_monitor
cd ..

# Install dependencies
echo "Installing dependencies..."
sudo apt-get update
sudo apt-get install -y \
  python3-pip \
  python3-rpi.gpio \
  ros-noetic-rosbridge-server

# Install Python packages
pip3 install flask rospkg

# Build the workspace
echo "Building the workspace..."
catkin_make

# Source the setup file
echo "source $WORKSPACE_DIR/devel/setup.bash" >> ~/.bashrc
source devel/setup.bash

echo "Setup complete! You can now run the system."
echo "Use 'roslaunch sensor_monitor sensors.launch' to start the sensors."
echo "Use 'roslaunch sensor_monitor web_interface.launch' to start the web interface."
chmod +x src/sensor_monitor/scripts/*.py

echo "Setup complete!"
