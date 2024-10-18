#!/usr/bin/python

from std_msgs.msg import Float64
from sensor_msgs.msg import JointState
import rospy
import math

position = []
velocity = []

def joint_states_callback(msg):
    global position, velocity
    position = msg.position  
    velocity = msg.velocity
    print("Joint Velocities:", velocity)

def main():
    rospy.init_node('limb')
    pubList = [
        rospy.Publisher('/joint02_effort_controller/command', Float64, queue_size=10),
        rospy.Publisher('/joint03_effort_controller/command', Float64, queue_size=10),
        rospy.Publisher('/joint04_effort_controller/command', Float64, queue_size=10),
        rospy.Publisher('/joint05_effort_controller/command', Float64, queue_size=10),
        rospy.Publisher('/joint06_effort_controller/command', Float64, queue_size=10),
        rospy.Publisher('/joint07_effort_controller/command', Float64, queue_size=10),
        rospy.Publisher('/joint08_effort_controller/command', Float64, queue_size=10),
        rospy.Publisher('/joint09_effort_controller/command', Float64, queue_size=10),
        rospy.Publisher('/joint10_effort_controller/command', Float64, queue_size=10),
        rospy.Publisher('/joint11_effort_controller/command', Float64, queue_size=10)
    ]

    # Subscribe to joint states topic
    rospy.Subscriber("/joint_states", JointState, joint_states_callback)

    rate = rospy.Rate(10)  # 20 Hz (20 cycles per second)

    scaled_torque = 0
    increment = 0.063
    max_torque = 0.25

    while not rospy.is_shutdown():
        # Check if position and velocity data are available
        if position and velocity:
            jointValList = []

            # Use the current scaled_torque for all joints
            for _ in range(len(pubList)):
                jointValList.append(Float64(scaled_torque))

            # Publish torque values
            for j in range(len(pubList)):
                pubList[j].publish(jointValList[j])

            # Gradually increase the torque after publishing
            if scaled_torque < max_torque:
                scaled_torque += increment
                if scaled_torque > max_torque:
                    scaled_torque = max_torque

        rate.sleep()  # Sleep for the defined rate (20 Hz)

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        print("Program interrupted before completion")
