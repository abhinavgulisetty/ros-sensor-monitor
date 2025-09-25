#!/usr/bin/env python3

import RPi.GPIO as GPIO
import rospy
import time
from std_msgs.msg import Float32
from sensor_monitor.msg import SensorData

# Define GPIO pins
TRIG_PIN = 23  # GPIO 23 (Pin 16)
ECHO_PIN = 24  # GPIO 24 (Pin 18)

class UltrasonicSensor:
    def __init__(self):
        # Initialize ROS node
        rospy.init_node('ultrasonic_sensor', anonymous=True)
        
        # Create publisher
        self.pub = rospy.Publisher('ultrasonic_data', Float32, queue_size=10)
        self.combined_pub = rospy.Publisher('sensor_data', SensorData, queue_size=10)
        
        # Subscribe to IR sensor data for combined message
        rospy.Subscriber('ir_data', Float32, self.ir_callback)
        
        # Set up GPIO
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(TRIG_PIN, GPIO.OUT)
        GPIO.setup(ECHO_PIN, GPIO.IN)
        
        # Initialize IR data
        self.ir_obstacle = False
        
        rospy.loginfo("Ultrasonic sensor node initialized")
    
    def ir_callback(self, data):
        # Store IR sensor data (0 = obstacle, 1 = no obstacle)
        self.ir_obstacle = data.data < 0.5
    
    def get_distance(self):
        # Ensure TRIG pin is low
        GPIO.output(TRIG_PIN, False)
        time.sleep(0.1)
        
        # Send 10us pulse to TRIG
        GPIO.output(TRIG_PIN, True)
        time.sleep(0.00001)
        GPIO.output(TRIG_PIN, False)
        
        # Measure time for ECHO to go high
        pulse_start = time.time()
        timeout = pulse_start + 0.1  # 100ms timeout
        
        while GPIO.input(ECHO_PIN) == 0:
            pulse_start = time.time()
            if pulse_start > timeout:
                return None  # Timeout
        
        # Measure time for ECHO to go low
        pulse_end = time.time()
        timeout = pulse_end + 0.1  # 100ms timeout
        
        while GPIO.input(ECHO_PIN) == 1:
            pulse_end = time.time()
            if pulse_end > timeout:
                return None  # Timeout
        
        # Calculate distance based on speed of sound (343m/s)
        pulse_duration = pulse_end - pulse_start
        distance = (pulse_duration * 34300) / 2  # Distance in cm
        
        return distance / 100  # Return distance in meters
    
    def run(self):
        rate = rospy.Rate(10)  # 10 Hz
        
        while not rospy.is_shutdown():
            distance = self.get_distance()
            
            if distance is not None:
                # Publish ultrasonic distance
                self.pub.publish(distance)
                
                # Create combined sensor message
                sensor_msg = SensorData()
                sensor_msg.distance = distance
                sensor_msg.obstacle = self.ir_obstacle
                sensor_msg.timestamp = rospy.Time.now()
                
                # Publish combined sensor data
                self.combined_pub.publish(sensor_msg)
                
                rospy.loginfo(f"Distance: {distance:.2f}m, Obstacle: {self.ir_obstacle}")
            else:
                rospy.logwarn("Ultrasonic sensor timeout")
            
            rate.sleep()

if __name__ == '__main__':
    try:
        sensor = UltrasonicSensor()
        sensor.run()
    except rospy.ROSInterruptException:
        pass
    finally:
        GPIO.cleanup()
