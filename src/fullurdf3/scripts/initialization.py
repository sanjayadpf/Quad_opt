#!/usr/bin/env python3

import rospy
from fullurdf3.srv import SetStiffness, SetStiffnessRequest
from fullurdf3.srv import SetDamping, SetDampingRequest
from std_msgs.msg import Float64
import time

def change_stiffness():
    rospy.init_node('init_node')
    rospy.loginfo("Node initialized")
    
    # Wait for stiffness service
    rospy.wait_for_service('/gazebo/set_joint_stiffness_fullurdf3')
    rospy.loginfo("Service /gazebo/set_joint_stiffness_fullurdf3 found")
    set_stiffness = rospy.ServiceProxy('/gazebo/set_joint_stiffness_fullurdf3', SetStiffness)
    rospy.loginfo("ServiceProxy for stiffness created")
    
    # Define the stiffness value to set for all joints
    stiffness_value = 1.11  

    # Iterate over joint indices
    for joint_index in range(2, 51):
        joint_name = f'joint{joint_index:02d}'
        
        # Prepare the stiffness service request
        stiffness_req = SetStiffnessRequest()
        stiffness_req.joint_name = joint_name
        stiffness_req.stiffness = stiffness_value

        rospy.loginfo(f"Calling /gazebo/set_joint_stiffness_fullurdf3 service for {joint_name} with stiffness {stiffness_value}")

        # Call the stiffness service
        try:
            stiffness_resp = set_stiffness(stiffness_req)
            if stiffness_resp.success:
                rospy.loginfo(f"Successfully set stiffness of {joint_name} to {stiffness_value}")
            else:
                rospy.logerr(f"Failed to set stiffness of {joint_name} to {stiffness_value}: {stiffness_resp.message}")
        except rospy.ServiceException as e:
            rospy.logerr(f"Service call failed for stiffness: {e}")

        # Sleep briefly between service calls (adjust as needed)
        time.sleep(0.1)

def change_damping():
    #rospy.init_node('change_damping_node')
    #rospy.loginfo("Node initialized")
    
    # Wait for damping service
    rospy.wait_for_service('/gazebo/set_joint_damping_fullurdf3')
    rospy.loginfo("Service /gazebo/set_joint_damping_fullurdf3 found")
    set_damping = rospy.ServiceProxy('/gazebo/set_joint_damping_fullurdf3', SetDamping)
    rospy.loginfo("ServiceProxy for damping created")
    
    # Define the damping value to set for all joints
    damping_value = 2.8  

    # Iterate over joint indices
    for joint_index in range(2, 51):
        joint_name = f'joint{joint_index:02d}'
        
        # Prepare the damping service request
        damping_req = SetDampingRequest()
        damping_req.joint_name = joint_name
        damping_req.damping = damping_value

        rospy.loginfo(f"Calling /gazebo/set_joint_damping_fullurdf3 service for {joint_name} with damping {damping_value}")

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

def diffleg():

    #rospy.init_node('legs')
    pubList = []
    
    # Publishers for joints 12 to 30 (scaled_torque)
    pubList.append(rospy.Publisher('/joint12_effort_controller/command', Float64, queue_size=36))
    pubList.append(rospy.Publisher('/joint13_effort_controller/command', Float64, queue_size=36))
    pubList.append(rospy.Publisher('/joint14_effort_controller/command', Float64, queue_size=36))
    pubList.append(rospy.Publisher('/joint15_effort_controller/command', Float64, queue_size=36))
    pubList.append(rospy.Publisher('/joint16_effort_controller/command', Float64, queue_size=36))
    pubList.append(rospy.Publisher('/joint17_effort_controller/command', Float64, queue_size=36))
    pubList.append(rospy.Publisher('/joint18_effort_controller/command', Float64, queue_size=36))
    pubList.append(rospy.Publisher('/joint19_effort_controller/command', Float64, queue_size=36))
    pubList.append(rospy.Publisher('/joint20_effort_controller/command', Float64, queue_size=36))

    pubList.append(rospy.Publisher('/joint22_effort_controller/command', Float64, queue_size=36))
    pubList.append(rospy.Publisher('/joint23_effort_controller/command', Float64, queue_size=36))
    pubList.append(rospy.Publisher('/joint24_effort_controller/command', Float64, queue_size=36))
    pubList.append(rospy.Publisher('/joint25_effort_controller/command', Float64, queue_size=36))
    pubList.append(rospy.Publisher('/joint26_effort_controller/command', Float64, queue_size=36))
    pubList.append(rospy.Publisher('/joint27_effort_controller/command', Float64, queue_size=36))
    pubList.append(rospy.Publisher('/joint28_effort_controller/command', Float64, queue_size=36))
    pubList.append(rospy.Publisher('/joint29_effort_controller/command', Float64, queue_size=36))
    pubList.append(rospy.Publisher('/joint30_effort_controller/command', Float64, queue_size=36))

    # Publishers for joints 32 to 50 (back_torque)
    pubList.append(rospy.Publisher('/joint32_effort_controller/command', Float64, queue_size=36))
    pubList.append(rospy.Publisher('/joint33_effort_controller/command', Float64, queue_size=36))
    pubList.append(rospy.Publisher('/joint34_effort_controller/command', Float64, queue_size=36))
    pubList.append(rospy.Publisher('/joint35_effort_controller/command', Float64, queue_size=36))
    pubList.append(rospy.Publisher('/joint36_effort_controller/command', Float64, queue_size=36))
    pubList.append(rospy.Publisher('/joint37_effort_controller/command', Float64, queue_size=36))
    pubList.append(rospy.Publisher('/joint38_effort_controller/command', Float64, queue_size=36))
    pubList.append(rospy.Publisher('/joint39_effort_controller/command', Float64, queue_size=36))
    pubList.append(rospy.Publisher('/joint40_effort_controller/command', Float64, queue_size=36))

    pubList.append(rospy.Publisher('/joint42_effort_controller/command', Float64, queue_size=36))
    pubList.append(rospy.Publisher('/joint43_effort_controller/command', Float64, queue_size=36))
    pubList.append(rospy.Publisher('/joint44_effort_controller/command', Float64, queue_size=36))
    pubList.append(rospy.Publisher('/joint45_effort_controller/command', Float64, queue_size=36))
    pubList.append(rospy.Publisher('/joint46_effort_controller/command', Float64, queue_size=36))
    pubList.append(rospy.Publisher('/joint47_effort_controller/command', Float64, queue_size=36))
    pubList.append(rospy.Publisher('/joint48_effort_controller/command', Float64, queue_size=36))
    pubList.append(rospy.Publisher('/joint49_effort_controller/command', Float64, queue_size=36))
    pubList.append(rospy.Publisher('/joint50_effort_controller/command', Float64, queue_size=36))

    num_joints = len(pubList)
    scaled_torque = 0.25
    back_torque = 0.25

    rate = rospy.Rate(10)  # 5 Hz (5 cycles per second)

    while not rospy.is_shutdown():
        jointValList = []

        for i in range(num_joints):
            if i < 19:  # Joints 12 to 30 (0-indexed 0 to 18)
                jointValList.append(Float64(scaled_torque))
            else:  # Joints 32 to 50 (0-indexed 19 to 37)
                jointValList.append(Float64(back_torque))

        # Publish torque values
        for j in range(num_joints):
            pubList[j].publish(jointValList[j])
            #print(f'Published to joint {j + 12 if j < 19 else j + 13}:', jointValList[j].data)
            print('start')

        rate.sleep()
        try:
       
            #basefromt
            basefront()
            #baseback
            baseback()

        except rospy.ROSInterruptException:
            print("Program interrupted before completion")
    

def basefront():

    #rospy.init_node('basefront')
    pubList = []

    pubList.append(rospy.Publisher('/joint11_effort_controller/command', Float64, queue_size=36))
    pubList.append(rospy.Publisher('/joint21_effort_controller/command', Float64, queue_size=36))

    num_joints = len(pubList)   
    scaled_torque = -1.2  #-1

    rate = rospy.Rate(10)  # 5 Hz

    #while not rospy.is_shutdown():
    jointValList=[]
        
    for i in range(num_joints):
        jointValList.append(Float64(scaled_torque))
        print("--------")
            # Publish torque values
    for j in range(num_joints):
        pubList[j].publish(jointValList[j])

        print('start')


    rate.sleep()  # Sleep for the defined rate (1 Hz)
def baseback():

    #rospy.init_node('base')
    pubList = []

    pubList.append(rospy.Publisher('/joint31_effort_controller/command', Float64, queue_size=36))
    pubList.append(rospy.Publisher('/joint41_effort_controller/command', Float64, queue_size=36))

    num_joints = len(pubList)   
    scaled_torque = -1.8         #-0.8

    rate = rospy.Rate(10)  # 5 Hz

    #while not rospy.is_shutdown():
    jointValList=[]
        
    for i in range(num_joints):
        jointValList.append(Float64(scaled_torque))
        print("--------")
        # Publish torque values
    for j in range(num_joints):
        pubList[j].publish(jointValList[j])

        print('start')


        rate.sleep()  # Sleep for the defined rate (1 Hz)


if __name__ == '__main__':
    try:
        #update stifness
        change_stiffness()
        #update damping
        change_damping()
        #diffeleg
        diffleg()

    except rospy.ROSInterruptException:
        print("Program interrupted before completion")
    
