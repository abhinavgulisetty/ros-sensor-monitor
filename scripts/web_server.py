#!/usr/bin/env python3

import rospy
import json
from flask import Flask, render_template, send_from_directory
from std_msgs.msg import Float32
from sensor_monitor.msg import SensorData
import threading
import os

app = Flask(__name__)

# Store latest sensor data
sensor_data = {
    'distance': 0.0,
    'obstacle': False,
    'timestamp': 0
}

@app.route('/')
def index():
    web_dir = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), 'web')
    return send_from_directory(web_dir, 'index.html')

@app.route('/static/<path:filename>')
def static_files(filename):
    web_dir = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), 'web')
    return send_from_directory(web_dir, filename)

@app.route('/api/sensor_data')
def get_sensor_data():
    return json.dumps({
        'distance': sensor_data['distance'],
        'obstacle': sensor_data['obstacle'],
        'timestamp': sensor_data['timestamp']
    })

def sensor_callback(data):
    sensor_data['distance'] = data.distance
    sensor_data['obstacle'] = data.obstacle
    sensor_data['timestamp'] = data.timestamp.to_sec()
    rospy.loginfo(f"Web: Distance: {data.distance:.2f}m, Obstacle: {data.obstacle}")

def ros_thread():
    rospy.init_node('web_server', anonymous=True)
    rospy.Subscriber('sensor_data', SensorData, sensor_callback)
    rospy.spin()

if __name__ == '__main__':
    # Start ROS thread
    t = threading.Thread(target=ros_thread)
    t.daemon = True
    t.start()
    
    # Get port from ROS parameter or default to 8080
    port = rospy.get_param('~port', 8080)
    
    # Start Flask app
    app.run(host='0.0.0.0', port=port, debug=False)
