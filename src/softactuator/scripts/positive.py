#!/usr/bin/python
#
# Send joint values to UR5 using messages
#

from std_msgs.msg import Header
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

def main():

    rospy.init_node('limb')
    pubList = []
    pubList.append(rospy.Publisher('/joint02_effort_controller/command', Float64, queue_size=10))
    pubList.append(rospy.Publisher('/joint03_effort_controller/command', Float64, queue_size=10))
    pubList.append(rospy.Publisher('/joint04_effort_controller/command', Float64, queue_size=10))
    pubList.append(rospy.Publisher('/joint05_effort_controller/command', Float64, queue_size=10))
    pubList.append(rospy.Publisher('/joint06_effort_controller/command', Float64, queue_size=10))
    pubList.append(rospy.Publisher('/joint07_effort_controller/command', Float64, queue_size=10))
    pubList.append(rospy.Publisher('/joint08_effort_controller/command', Float64, queue_size=10))
    pubList.append(rospy.Publisher('/joint09_effort_controller/command', Float64, queue_size=10))
    pubList.append(rospy.Publisher('/joint10_effort_controller/command', Float64, queue_size=10))
    pubList.append(rospy.Publisher('/joint11_effort_controller/command', Float64, queue_size=10))

    max_torque = 0.1  # Define maximum torque value (can adjust as needed)
    min_torque = 0.0  # Define minimum torque value (can adjust as needed)
    num_joints = 10   # Number of joints from joint02 to joint11

    # Define friction related K value
    k = 0
    d = 0

    # List to keep track of the most recent non-negative torque values
    last_non_negative_torque = [0.0] * num_joints

    # Subscribe to joint states topic
    rospy.Subscriber("/joint_states", JointState, joint_states_callback)

    rate = rospy.Rate(1)  # 1 Hz (1 cycle per second)

    while not rospy.is_shutdown():
        # Check if position and velocity data are available
        if position and velocity:
            jointValList = []
            for i in range(num_joints):
                scaled_torque = max_torque - (max_torque - min_torque) * (i / (num_joints - 1))
                t = scaled_torque - k * abs(position[i + 1]) - d * abs(velocity[i])
                
                # Update the torque only if it is non-negative, otherwise use the last non-negative value
                if t >= 0:
                    last_non_negative_torque[i] = t
                else:
                    t = last_non_negative_torque[i]
                
                print(f"{scaled_torque} --- {k * abs(position[i + 1])} == {t}")
                jointValList.append(Float64(t))
            print("--------")
            
            # Publish torque values
            for j in range(num_joints):
                pubList[j].publish(jointValList[j])

        rate.sleep()  # Sleep for the defined rate (1 Hz)

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        print("Program interrupted before completion")
