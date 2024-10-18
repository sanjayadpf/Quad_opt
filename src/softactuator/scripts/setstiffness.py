#!/usr/bin/env python

import rospy
from softactuator.srv import SetJointStiffness

def set_joint_stiffness(joint_names, stiffnesses):
    rospy.wait_for_service('set_joint_stiffness')
    try:
        set_stiffness = rospy.ServiceProxy('set_joint_stiffness', SetJointStiffness)
        response = set_stiffness(joint_names, stiffnesses)
        return response.success
    except rospy.ServiceException as e:
        rospy.logerr("Service call failed: %s" % e)
        return False

if __name__ == "__main__":
    rospy.init_node('set_joint_stiffness_client')

    joint_names = [
        "joint2", "joint3", "joint4", "joint5",
        "joint6", "joint7", "joint8", "joint9", "joint10", "joint11"
    ]
    stiffnesses = [
        10.0, 20.0, 30.0, 40.0, 50.0,
        60.0, 70.0, 80.0, 90.0, 100.0, 110.0
    ]

    success = set_joint_stiffness(joint_names, stiffnesses)
    if success:
        rospy.loginfo("Successfully set stiffness for all joints")
    else:
        rospy.logwarn("Failed to set stiffness for all joints")
