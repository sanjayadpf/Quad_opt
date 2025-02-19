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
    # print("Received joint states message")
    #Print the position values
    global position, velocity
    position = msg.position  
    velocity = msg.velocity
    # print("Joint Positions:", position)
    print("Joint Velocities:", velocity)


def main():

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


    # Create the topic message
    #pi=-0.1
    jointValList=[]

   # max_torque = -0.5  # Define maximum torque value (can adjust as needed)
    #min_torque = -0.5  # Define minimum torque value (can adjust as needed)
    num_joints = 10   # Number of joints from joint02 to joint11
    scaled_torque = 0.25

    # Subscribe to joint states topic
    rospy.Subscriber("/joint_states", JointState, joint_states_callback)

    rate = rospy.Rate(20)  # 1 Hz (1 cycle per second)

    while not rospy.is_shutdown():
        # Check if position and velocity data are available
        if position and velocity:
            jointValList = []
            for i in range(num_joints):
                # Use the torque calculation with stiffness and damping
                #scaled_torque = max_torque - (max_torque - min_torque) * (i / (num_joints-1))
                # scaled_torque = (0.01*math.exp(-i) - 0.07*i + 1)*(max_torque)
                #t = max(0, scaled_torque - k * abs(position[i+1]) - d * abs(velocity[i]))  # Adjusted torque calculation
                #print(str(scaled_torque), '   ---   ', str(k * abs(position[i+1])), '  ==  ', str(t))
                jointValList.append(Float64(scaled_torque))
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