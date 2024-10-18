#!/usr/bin/python

from std_msgs.msg import Float64
import rospy

def main():

    rospy.init_node('legs')
    pubList = []

    # Publishers for joint11, joint21, joint31, joint41
    pubList.append(rospy.Publisher('/joint11_effort_controller/command', Float64, queue_size=36))
    pubList.append(rospy.Publisher('/joint21_effort_controller/command', Float64, queue_size=36))
    pubList.append(rospy.Publisher('/joint31_effort_controller/command', Float64, queue_size=36))
    pubList.append(rospy.Publisher('/joint41_effort_controller/command', Float64, queue_size=36))

    num_joints = len(pubList)   
    scaled_torque = 0.2

    rate = rospy.Rate(5)  # 5 Hz

    while not rospy.is_shutdown():
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
        main()
    except rospy.ROSInterruptException:
        print("Program interrupted before completion")
