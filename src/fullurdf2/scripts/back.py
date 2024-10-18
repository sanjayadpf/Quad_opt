 #!/usr/bin/python
#
# Send joint values to UR5 using messages
#

from std_msgs.msg import Header
from std_msgs.msg import Float64
from sensor_msgs.msg import JointState
import rospy


def main():

    rospy.init_node('limb')
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
    rate = rospy.Rate(5)

    num_joints = 9
    scaled_torque = 0.25   
    #increment = 0.063
    #max_torque = 0.25
    direction = 1

    while not rospy.is_shutdown():
        jointValList=[]
        
        # # Use the current scaled_torque for all joints
        # for _ in range(len(pubList)):
        #     jointValList.append(Float64(direction*scaled_torque))

        # # Publish torque values
        # for j in range(len(pubList)):
        #     pubList[j].publish(jointValList[j])

        # # Gradually increase the torque after publishing
        # if scaled_torque < max_torque:
        #     scaled_torque += increment
        #     if scaled_torque > max_torque:
        #         scaled_torque = max_torque

        for i in range(num_joints):
            jointValList.append(Float64(direction*scaled_torque))
            print("--------")
            
            # Publish torque values
        for j in range(num_joints):
            pubList[j].publish(jointValList[j])

            print('start')

        direction *= -1
        print(direction*scaled_torque)
        rate.sleep()  # Sleep for the defined rate (1 Hz)

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        print("Program interrupted before completion")