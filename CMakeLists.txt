cmake_minimum_required(VERSION 3.0.2)
project(sensor_monitor)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  sensor_msgs
  message_generation
)

add_message_files(
  FILES
  SensorData.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS rospy std_msgs sensor_msgs message_runtime
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

# Install Python scripts
catkin_install_python(PROGRAMS
  scripts/ultrasonic_sensor.py
  scripts/ir_sensor.py
  scripts/web_server.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# Install launch files
install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)

# Install web interface files
install(DIRECTORY web/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/web
)
