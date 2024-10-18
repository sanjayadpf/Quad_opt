#!/usr/bin/env python3

import rospy
from fullurdf3.srv import SetFriction, SetFrictionRequest
import time

def change_friction():
    rospy.init_node('change_friction_node')
    rospy.loginfo("Node initialized")
    
    # Wait for friction service
    rospy.wait_for_service('/gazebo/set_joint_friction_fullurdf3')
    rospy.loginfo("Service /gazebo/set_joint_friction_fullurdf3 found")
    set_friction = rospy.ServiceProxy('/gazebo/set_joint_friction_fullurdf3', SetFriction)
    rospy.loginfo("ServiceProxy for friction created")
    
    # Define the friction value to set for all joints
    friction_value = 0.2  

    # Iterate over joint indices
    for joint_index in range(2, 51):
        joint_name = f'joint{joint_index:02d}'
        
        # Prepare the friction service request
        friction_req = SetFrictionRequest()
        friction_req.joint_name = joint_name
        friction_req.friction = friction_value

        rospy.loginfo(f"Calling /gazebo/set_joint_friction_fullurdf3 service for {joint_name} with friction {friction_value}")

        # Call the friction service
        try:
            friction_resp = set_friction(friction_req)
            if friction_resp.success:
                rospy.loginfo(f"Successfully set friction of {joint_name} to {friction_value}")
            else:
                rospy.logerr(f"Failed to set friction of {joint_name} to {friction_value}: {friction_resp.message}")
        except rospy.ServiceException as e:
            rospy.logerr(f"Service call failed for friction: {e}")

        # Sleep briefly between service calls (adjust as needed)
        time.sleep(0.1)

if __name__ == '__main__':
    change_friction()
