#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import JointState
import math  # Import math to use pi for conversion

class LimbBendingAngleCalculator:
    def __init__(self):
        # Initialize the ROS node
        rospy.init_node('limb_bending_angle_calculator')
        
        # Initialize the total bending angle
        self.total_bending_angle = 0.0
        
        # Joint names for the limb
        self.joint_names = [f'joint{index:02d}' for index in range(2, 11)]

        # Subscribe to the /joint_states topic
        rospy.Subscriber('/joint_states', JointState, self.joint_states_callback)
        
    def joint_states_callback(self, msg):
        # Extract the joint names and positions from the message
        joint_angles = {}
        for name, position in zip(msg.name, msg.position):
            joint_angles[name] = position
        
        # Calculate the total bending angle for the limb in radians
        total_bending_angle_radians = sum(joint_angles[joint_name] for joint_name in self.joint_names if joint_name in joint_angles)
        
        # Convert the total bending angle to degrees
        self.total_bending_angle = math.degrees(total_bending_angle_radians)

        # Print or log the total bending angle in degrees
        rospy.loginfo(f'Total bending angle of the limb: {self.total_bending_angle:.2f} degrees')

    def run(self):
        # Keep the node running
        rospy.spin()

if __name__ == '__main__':
    try:
        calculator = LimbBendingAngleCalculator()
        calculator.run()
    except rospy.ROSInterruptException:
        pass
