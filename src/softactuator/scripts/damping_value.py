#!/usr/bin/env python3

import rospy
from softactuator.srv import SetDamping, SetDampingRequest
import time

def change_damping():
    rospy.init_node('change_damping_node')
    rospy.loginfo("Node initialized")
    
    # Wait for damping service
    rospy.wait_for_service('/gazebo/set_joint_damping_softactuator')
    rospy.loginfo("Service /gazebo/set_joint_damping_softactuator found")
    set_damping = rospy.ServiceProxy('/gazebo/set_joint_damping_softactuator', SetDamping)
    rospy.loginfo("ServiceProxy for damping created")
    
    # Define the damping value to set for all joints
    damping_value = 0.0001  

    # Iterate over joint indices
    for joint_index in range(2, 12):
        joint_name = f'joint{joint_index:02d}'
        
        # Prepare the damping service request
        damping_req = SetDampingRequest()
        damping_req.joint_name = joint_name
        damping_req.damping = damping_value

        rospy.loginfo(f"Calling /gazebo/set_joint_damping_softactuator service for {joint_name} with damping {damping_value}")

        # Call the damping service
        try:
            damping_resp = set_damping(damping_req)
            if damping_resp.success:
                rospy.loginfo(f"Successfully set damping of {joint_name} to {damping_value}")
            else:
                rospy.logerr(f"Failed to set damping of {joint_name} to {damping_value}: {damping_resp.message}")
        except rospy.ServiceException as e:
            rospy.logerr(f"Service call failed for damping: {e}")

        # Sleep briefly between service calls (adjust as needed)
        time.sleep(0.1)

if __name__ == '__main__':
    change_damping()
