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
from skopt.space import Real
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
    #scaled_torque = 0.25
    #back_torque = 0.25
    scaled_torque = diffleg_torque
    back_torque = diffleg_torque

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

    #rospy.init_node('limb')
    pubList=[]
    #pubList.append(rospy.Publisher('/joint01_effort_controller/command', Float64, queue_size=11))
    pubList.append(rospy.Publisher('/joint02_effort_controller/command', Float64, queue_size=9))
    pubList.append(rospy.Publisher('/joint03_effort_controller/command', Float64, queue_size=9))
    pubList.append(rospy.Publisher('/joint04_effort_controller/command', Float64, queue_size=9))
    pubList.append(rospy.Publisher('/joint05_effort_controller/command', Float64, queue_size=9))
    pubList.append(rospy.Publisher('/joint06_effort_controller/command', Float64, queue_size=9))
    pubList.append(rospy.Publisher('/joint07_effort_controller/command', Float64, queue_size=9))
    pubList.append(rospy.Publisher('/joint08_effort_controller/command', Float64, queue_size=9))
    pubList.append(rospy.Publisher('/joint09_effort_controller/command', Float64, queue_size=9))
    pubList.append(rospy.Publisher('/joint10_effort_controller/command', Float64, queue_size=9))

    #pub = rospy.Publisher('/joint01_effort_controller/command',Float64,queue_size=10)
    rate = rospy.Rate(8)

    num_joints = 9
    scaled_torque = 0.25   
    #increment = 0.063
    #max_torque = 0.25
    direction = 1

    num_iterations = 25  # Set the desired number of iterations

    for _ in range(num_iterations):
        if rospy.is_shutdown():
            break

        try:
            # basefront
            basefront()
            # baseback
            baseback()
            # diffleg
            diffleg(diffleg_torque)

        except rospy.ROSInterruptException:
            print("Program interrupted before completion")
            break
            
        jointValList = []

        for i in range(num_joints):
            jointValList.append(Float64(direction * scaled_torque))
            #print("--------")

        # Publish torque values
        for j in range(num_joints):
            pubList[j].publish(jointValList[j])
            #print('start')

        direction *= -1
        #print(direction * scaled_torque)
        #print(position.y)
        rate.sleep()  # Sleep for the defined rate (8 Hz in this
    time.sleep(1)
    return -1*position.y

        
     
######-------------------------utility methods--------------------------------------####
def reset_simulation():
    rospy.wait_for_service('/gazebo/reset_simulation')
    try:
        reset_sim = rospy.ServiceProxy('/gazebo/reset_simulation', Empty)
        reset_sim()
        print("Simulation reset")
        #time.sleep(1)  # Add a small delay to allow everything to reset
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

def spine_activate_torque(torque):
    """Wrapper function to reset simulation and activate spine with the given torque."""
    reset_simulation()
    return spine_activate(diffleg_torque=torque)  # Get distance from spine activation

def objective_function(torque):
    """Return the negative distance, because we want to maximize distance."""
    # Round the torque value to 3 decimal places
    rounded_torque = round(torque[0], 3)
    print(f"Evaluating torque: {rounded_torque}")
    distance = spine_activate_torque(rounded_torque)
    return -distance  # Minimize negative distance (equivalent to maximizing distance)

if __name__ == '__main__':
    try:
        # Update stiffness
        change_stiffness()
        # Update damping
        change_damping()
        # Subscribe for distance values
        get_base_link_position()

        # Define the bounds for diffleg_torque
        torque_range = [Real(0.15, 0.30, name='diffleg_torque')]

        # Perform Bayesian Optimization using Gaussian Process
        result = gp_minimize(
            func=objective_function,          # Objective function to minimize
            dimensions=torque_range,          # Search space (torque bounds)
            n_calls=25,                       # Number of function evaluations
            acq_func="EI",                    # Expected improvement acquisition function
            random_state=42                   # For reproducibility
        )

        # Extract the optimal torque and the corresponding maximum distance
        optimal_torque = round(result.x[0], 3)  # Round the final torque to 3 decimal places
        max_distance = round(-result.fun, 4)    # Result.fun is the minimized value (negative distance)

        print(f"Optimal diffleg_torque: {optimal_torque}, Maximum distance: {max_distance}")

    except rospy.ROSInterruptException:
        print("Program interrupted before completion")