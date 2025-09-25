#!/usr/bin/env python3

import RPi.GPIO as GPIO
import rospy
from std_msgs.msg import Float32

# Define GPIO pins
IR_PIN = 17  # GPIO 17 (Pin 11)

class IRSensor:
    def __init__(self):
        # Initialize ROS node
        rospy.init_node('ir_sensor', anonymous=True)
        
        # Create publisher
        self.pub = rospy.Publisher('ir_data', Float32, queue_size=10)
        
        # Set up GPIO
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(IR_PIN, GPIO.IN)
        
        rospy.loginfo("IR sensor node initialized")
    
    def run(self):
        rate = rospy.Rate(10)  # 10 Hz
        
        while not rospy.is_shutdown():
            # Read IR sensor (0 when obstacle detected, 1 when no obstacle)
            ir_value = GPIO.input(IR_PIN)
            
            # Publish IR sensor reading
            self.pub.publish(float(ir_value))
            
            # Log the reading (0 = obstacle detected, 1 = no obstacle)
            obstacle_text = "No obstacle" if ir_value == 1 else "Obstacle detected"
            rospy.loginfo(f"IR Sensor: {obstacle_text}")
            
            rate.sleep()

if __name__ == '__main__':
    try:
        sensor = IRSensor()
        sensor.run()
    except rospy.ROSInterruptException:
        pass
    finally:
        GPIO.cleanup()
