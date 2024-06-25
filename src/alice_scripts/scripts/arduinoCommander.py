#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float64MultiArray
import serial

# Open the serial port
ser = serial.Serial('/dev/ttyACM0', 57600)

# Callback function
def joint_angles_callback(msg):
    joint_values = msg.data  # msg is a Float64MultiArray
    rounded_values = [round(value, 2) for value in joint_values]
    joint_str = ','.join(map(str, rounded_values)) + '\n' 
    ser.write(joint_str.encode())  # Send the string over serial
    rospy.logwarn("Successfully sent to arduino over Serial")


# Initialize the ROS node
rospy.init_node('arduinoCommander')

# Subscribe to /joint_angles topic and call joint_angles_callback when messages arrive
rospy.Subscriber("/alice/joint_states_arduino", Float64MultiArray, joint_angles_callback)

# Run the script
rospy.spin()