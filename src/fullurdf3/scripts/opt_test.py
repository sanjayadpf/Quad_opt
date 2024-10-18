#!/usr/bin/env python3

import rospy
from fullurdf3.srv import SetStiffness, SetStiffnessRequest
from fullurdf3.srv import SetDamping, SetDampingRequest
from std_msgs.msg import Float64
from std_msgs.msg import Header
from sensor_msgs.msg import JointState
from std_srvs.srv import Empty
from gazebo_msgs.msg import ModelStates
from scipy.optimize import minimize
from scipy.optimize import differential_evolution
from skopt import gp_minimize
from skopt.space import Real,Integer
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

def diffleg(diffleg_torque):

    #rospy.init_node('legs')
    pubList = []
    
    # Publishers for joints 12 to 30 (scaled_torque)
    pubList.append(rospy.Publisher('/joint12_effort_controller/command', Float64, queue_size=36))
    pubList.append(rospy.Publisher('/joint22_effort_controller/command', Float64, queue_size=36))
    pubList.append(rospy.Publisher('/joint13_effort_controller/command', Float64, queue_size=36))
    pubList.append(rospy.Publisher('/joint23_effort_controller/command', Float64, queue_size=36))
    pubList.append(rospy.Publisher('/joint14_effort_controller/command', Float64, queue_size=36))
    pubList.append(rospy.Publisher('/joint24_effort_controller/command', Float64, queue_size=36))
    pubList.append(rospy.Publisher('/joint15_effort_controller/command', Float64, queue_size=36))
    pubList.append(rospy.Publisher('/joint25_effort_controller/command', Float64, queue_size=36))
    pubList.append(rospy.Publisher('/joint16_effort_controller/command', Float64, queue_size=36))
    pubList.append(rospy.Publisher('/joint26_effort_controller/command', Float64, queue_size=36))
    pubList.append(rospy.Publisher('/joint17_effort_controller/command', Float64, queue_size=36))
    pubList.append(rospy.Publisher('/joint27_effort_controller/command', Float64, queue_size=36))
    pubList.append(rospy.Publisher('/joint18_effort_controller/command', Float64, queue_size=36))
    pubList.append(rospy.Publisher('/joint28_effort_controller/command', Float64, queue_size=36))
    pubList.append(rospy.Publisher('/joint19_effort_controller/command', Float64, queue_size=36))
    pubList.append(rospy.Publisher('/joint29_effort_controller/command', Float64, queue_size=36))
    pubList.append(rospy.Publisher('/joint20_effort_controller/command', Float64, queue_size=36))
    pubList.append(rospy.Publisher('/joint30_effort_controller/command', Float64, queue_size=36))

    
    # Publishers for joints 32 to 50 (back_torque)
    pubList.append(rospy.Publisher('/joint32_effort_controller/command', Float64, queue_size=36))
    pubList.append(rospy.Publisher('/joint42_effort_controller/command', Float64, queue_size=36))
    pubList.append(rospy.Publisher('/joint33_effort_controller/command', Float64, queue_size=36))
    pubList.append(rospy.Publisher('/joint43_effort_controller/command', Float64, queue_size=36))
    pubList.append(rospy.Publisher('/joint34_effort_controller/command', Float64, queue_size=36))
    pubList.append(rospy.Publisher('/joint44_effort_controller/command', Float64, queue_size=36))
    pubList.append(rospy.Publisher('/joint35_effort_controller/command', Float64, queue_size=36))
    pubList.append(rospy.Publisher('/joint45_effort_controller/command', Float64, queue_size=36))
    pubList.append(rospy.Publisher('/joint36_effort_controller/command', Float64, queue_size=36))
    pubList.append(rospy.Publisher('/joint46_effort_controller/command', Float64, queue_size=36))
    pubList.append(rospy.Publisher('/joint37_effort_controller/command', Float64, queue_size=36))
    pubList.append(rospy.Publisher('/joint47_effort_controller/command', Float64, queue_size=36))
    pubList.append(rospy.Publisher('/joint38_effort_controller/command', Float64, queue_size=36))
    pubList.append(rospy.Publisher('/joint48_effort_controller/command', Float64, queue_size=36))
    pubList.append(rospy.Publisher('/joint39_effort_controller/command', Float64, queue_size=36))
    pubList.append(rospy.Publisher('/joint49_effort_controller/command', Float64, queue_size=36))
    pubList.append(rospy.Publisher('/joint40_effort_controller/command', Float64, queue_size=36))
    pubList.append(rospy.Publisher('/joint50_effort_controller/command', Float64, queue_size=36))


    num_joints = len(pubList)
    #scaled_torque = 0.25
    #back_torque = 0.25
    scaled_torque = diffleg_torque[0]
    back_torque = diffleg_torque[1]

    rate = rospy.Rate(10)  # 5 Hz (5 cycles per second)

    #while not rospy.is_shutdown():
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
    #print('start')

    #rate.sleep()
        
    

def basefront():

    #rospy.init_node('basefront')
    pubList = []

    pubList.append(rospy.Publisher('/joint11_effort_controller/command', Float64, queue_size=36))
    pubList.append(rospy.Publisher('/joint21_effort_controller/command', Float64, queue_size=36))

    num_joints = len(pubList)   
    scaled_torque = -1.2  #-1

    #rate = rospy.Rate(10)  # 5 Hz

    #while not rospy.is_shutdown():
    jointValList=[]
        
    for i in range(num_joints):
        jointValList.append(Float64(scaled_torque))
        #print("--------")
            # Publish torque values
    for j in range(num_joints):
        pubList[j].publish(jointValList[j])

        #print('start')


    #rate.sleep()  # Sleep for the defined rate (1 Hz)
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
        #print("--------")
        # Publish torque values
    for j in range(num_joints):
        pubList[j].publish(jointValList[j])

        #print('start')


        #rate.sleep()  # Sleep for the defined rate (1 Hz)

def spine_activate(diffleg_torque):
    """Function to activate spine with a given torque and rate, constrained by time duration."""
    # Initialize publishers for the joints
    pubList = []
    pubList.append(rospy.Publisher('/joint02_effort_controller/command', Float64, queue_size=9))
    pubList.append(rospy.Publisher('/joint03_effort_controller/command', Float64, queue_size=9))
    pubList.append(rospy.Publisher('/joint04_effort_controller/command', Float64, queue_size=9))
    pubList.append(rospy.Publisher('/joint05_effort_controller/command', Float64, queue_size=9))
    pubList.append(rospy.Publisher('/joint06_effort_controller/command', Float64, queue_size=9))
    pubList.append(rospy.Publisher('/joint07_effort_controller/command', Float64, queue_size=9))
    pubList.append(rospy.Publisher('/joint08_effort_controller/command', Float64, queue_size=9))
    pubList.append(rospy.Publisher('/joint09_effort_controller/command', Float64, queue_size=9))
    pubList.append(rospy.Publisher('/joint10_effort_controller/command', Float64, queue_size=9))

    rate = rospy.Rate(8)  # Control the rate of execution

    num_joints = 9
    scaled_torque = diffleg_torque[2]#0.25
    direction = 1
    duration=5
    start_time = time.time()  # Record the start time
    current_time = start_time

    # Run until the specified duration has passed
    max_z=position.z
    while current_time - start_time < duration:
        if rospy.is_shutdown():
            break

        try:
            # Execute basefront, baseback, and diffleg functions
            basefront()
            baseback()
            diffleg(diffleg_torque)

        except rospy.ROSInterruptException:
            print("Program interrupted before completion")
            break

        jointValList = []

        for i in range(num_joints):
            jointValList.append(Float64(direction * scaled_torque))

        # Publish torque values to each joint
        for j in range(num_joints):
            pubList[j].publish(jointValList[j])

        direction *= -1

        rate.sleep()  # Sleep for the defined rate (in Hz)

        current_time = time.time()  # Update current time

    # Add a small delay to stabilize after the loop
    time.sleep(1)
    if position.z>max_z:
        max_z=position.z
    return -1 * position.y#+max_z  # Return the negative y position (or other relevant metric)

        
     
######-------------------------utility methods--------------------------------------####
def reset_simulation():
    rospy.wait_for_service('/gazebo/reset_simulation')
    try:
        reset_sim = rospy.ServiceProxy('/gazebo/reset_simulation', Empty)
        reset_sim()
        print("Simulation reset")
        time.sleep(2)  # Add a small delay to allow everything to reset
    except rospy.ServiceException as e:
        print(f"Service call failed: {e}")

position = None
orientation =None
def model_states_callback(data):
    global position, orientation
    try:
        # Find the index of the robot named 'fullurdf3'
        index = data.name.index('fullurdf3')  # Robot name is 'fullurdf3'
        
        # Extract the position and orientation of the base_link
        position = data.pose[index].position
        orientation = data.pose[index].orientation
        
        #rospy.loginfo(f"Base_link position: x={position.x}, y={position.y}, z={position.z}")
        #rospy.loginfo(f"Base_link orientation: x={orientation.x}, y={orientation.y}, z={orientation.z}, w={orientation.w}")
    
    except ValueError:
        rospy.logwarn("Robot model 'fullurdf3' not found in model states")

def get_base_link_position():
    #rospy.init_node('base_link_position_listener', anonymous=True)
    
    # Subscribe to the /gazebo/model_states topic
    rospy.Subscriber("/gazebo/model_states", ModelStates, model_states_callback)
    
    #rospy.spin()

#------------------------optimization stuff--------------------------------#

def spine_activate_torque(torques):
    """Wrapper function to reset simulation and activate spine with the given torques and rate."""
    reset_simulation()
    return spine_activate(diffleg_torque=torques)  # Pass the array of torques and rate

def objective_function(params):
    """Return the negative distance, because we want to maximize distance."""
    # Extract the torque values and rate from the array
    scaled_torque = round(params[0], 3)
    back_torque = round(params[1], 3)
    spine_torque = round(params[2], 3)
    
    print(f"Evaluating: scaled_torque = {scaled_torque}, back_torque = {back_torque}, spine_torque = {spine_torque}")
    
    # Call the simulation with the array of two torques and rate
    distance = spine_activate_torque([scaled_torque, back_torque, spine_torque])
    
    return -distance  # Minimize the negative distance (equivalent to maximizing distance)

if __name__ == '__main__':
    try:
        # Update stiffness
        change_stiffness()
        # Update damping
        change_damping()
        # Subscribe for distance values
        get_base_link_position()

        # Define the bounds for scaled_torque, back_torque, and rate (rate is now an integer)
        scaled_torque=0.349
        back_torque=0.242        
        spine_torque=0.349
      
        dist=objective_function([scaled_torque, back_torque, spine_torque])
        print(dist)

    except rospy.ROSInterruptException:
        print("Program interrupted before completion")