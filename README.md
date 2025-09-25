# ROS Sensor Monitoring System

This project integrates an HC-SR04 ultrasonic distance sensor and an IR sensor with ROS on a Raspberry Pi 4. It includes:
- ROS nodes for reading sensor data
- Web visualization for sensor readings

## Hardware Setup

### Ultrasonic Sensor (HC-SR04)
- VCC → Raspberry Pi Pin 2 (5V)
- GND → Raspberry Pi Pin 6 (GND)
- TRIG → Raspberry Pi Pin 16 (GPIO 23)
- ECHO → Raspberry Pi Pin 18 (GPIO 24) via voltage divider

### IR Sensor
- VCC → Raspberry Pi Pin 1 (3.3V)
- GND → Raspberry Pi Pin 9 (GND)
- OUT → Raspberry Pi Pin 11 (GPIO 17)

## Installation

Run the setup script:
```bash
./setup.sh
```

## Building the Project

```bash
cd ~/ros_sensor_monitor
catkin_make
source devel/setup.bash
```

## Running the System

Start the ROS core:
```bash
roscore
```

Launch the sensors:
```bash
roslaunch sensor_monitor sensors.launch
```

Start the web interface:
```bash
roslaunch sensor_monitor web_interface.launch
```

Open a browser and go to: http://localhost:8080
