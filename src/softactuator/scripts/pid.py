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

# PID controller parameters
Kp = 1.0  # Proportional gain
Ki = 0.5  # Integral gain
Kd = 0.01 # Derivative gain

max_torque = 0.1  # Maximum torque value
min_torque = 0.0  # Minimum torque value
num_joints = 10   # Number of joints from joint02 to joint11

k = 0
d = 0

# Initialize PID controller variables
prev_error = [0.0] * num_joints
integral = [0.0] * num_joints
last_torque = [0.0] * num_joints  # Store the last valid torque value for each joint

def joint_states_callback(msg):
    # print("Received joint states message")
    #Print the position values
    global position, velocity
    position = msg.position  
    velocity = msg.velocity
    # print("Joint Positions:", position)
    # print("Joint Velocities:", velocity)


def main():

    global prev_error, integral, last_torque

    rospy.init_node('limb')
    pubList=[]
    #pubList.append(rospy.Publisher('/joint01_effort_controller/command', Float64, queue_size=11))
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

    #pub = rospy.Publisher('/joint01_effort_controller/command',Float64,queue_size=10)


    # Subscribe to joint states topic
    rospy.Subscriber("/joint_states", JointState, joint_states_callback)

    rate = rospy.Rate(1)  # 1 Hz (1 cycle per second)

    while not rospy.is_shutdown():
        # Check if position and velocity data are available
        if position and velocity:
            jointValList = []
            for i in range(num_joints):
                # Calculate scaled torque based on the ratio of current joint index to total number of joints
                scaled_torque = max_torque - (max_torque - min_torque) * (i / (num_joints - 1))
                
                # Calculate the error
                error = scaled_torque - (k * abs(position[i+1]) + d * abs(velocity[i]))

                # Calculate integral and derivative
                integral[i] += error
                derivative = error - prev_error[i]
                
                # Calculate PID output
                t = Kp * error + Ki * integral[i] 
                # Kd * derivative
                
                # Ensure torque is within bounds and non-negative
                if t < 0:
                    t = last_torque[i]
                else:
                    t = max(min_torque, min(t, max_torque))
                    last_torque[i] = t
                
                # Update previous error
                prev_error[i] = error
                
                print(f"Joint {i+1}: Scaled Torque: {scaled_torque:.4f}, Error: {error:.4f}, PID Torque: {t:.4f}")
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