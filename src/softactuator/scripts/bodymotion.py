#!/usr/bin/python
#
# Send joint values to UR5 using messages

from std_msgs.msg import Float64
from sensor_msgs.msg import JointState
import rospy
import time
import numpy as np

def joint_states_callback(msg):
    print("Received joint states message")
    #Print the position values
    global position, velocity
    position = msg.position  # Ensure all positions are positive
    velocity = [abs(vel) for vel in msg.velocity]
    print("Joint Positions:", position)
    
    #Define friction related K value
    k = 1
    d = 1
    # Define initial torque input
    initial_torque = 0

    # Calculate the number of joints
    num_joints = len(position) - 1
    #print(num_joints)

    # Calculate torque for all joints
    t_values = []
    ts_val = []
    for i, (pos, vel) in enumerate(zip(position, velocity)):
        # Calculate scaled torque based on the ratio of current joint index to total number of joints
        #scaled_torque = initial_torque / 10 * (num_joints - i)
        #scaled_torque = initial_torque / 10 * (-i + 10)
        #ts_val.append(scaled_torque)
        # Ensure torque is positive
        #t = initial_torque - k * pos - d * vel
        t=0
        t_values.append(t)  # Create a Float64 message with the desired torque
    print("Torque values:", t_values)
    #print("Sorque values:", ts_val)

    # Publish torque commands to joints
    for pub, torque in zip(pubList, t_values):
        jointVal = Float64(data=torque)
        pub.publish(jointVal)


#def calculate_torque(position):
    #k = 1  # Friction related K value
    #initial_torque = 0.05  # Initial torque input
    #num_joints = len(position)
    #t_values = []
    #for i, pos in enumerate(position):
    #    scaled_torque = initial_torque / 9 * (num_joints - i)
    #    t = scaled_torque - k * abs(pos)  # Ensure torque is positive
    #    t_values.append(t)  # Scale torque and store
    #return t_values

def main():
    rospy.init_node('limb')
    
    # Create publishers for each joint except joint01
    global pubList
    pubList = []
    for i in range(1, 12):  # Skip joint01, start from 1
        if i != 1:  # Skip joint01
            pubList.append(rospy.Publisher('/joint{:02d}_effort_controller/command'.format(i), Float64, queue_size=10))

    # Subscribe to joint states topic
    rospy.Subscriber("/joint_states", JointState, joint_states_callback)

    # Main loop
    while not rospy.is_shutdown():
        print("Main loop")
        # Wait for joint positions to be received
        while not hasattr(main, 'position'):
            time.sleep(0.1)

        # Calculate torque based on received joint positions
        #torque_values = calculate_torque(position)
        
        # Publish torque commands to joints
        #for pub, torque in zip(pubList, torque_values):
        #    jointVal = Float64(data=torque)
        #    pub.publish(jointVal)

        #Wait for joints to stabilize
        time.sleep(0.5)

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        print("Program interrupted before completion")

