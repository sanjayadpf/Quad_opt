#!/usr/bin/python

from std_msgs.msg import Float64
import rospy

def main():

    rospy.init_node('base')
    pubList = []

    pubList.append(rospy.Publisher('/joint31_effort_controller/command', Float64, queue_size=36))
    pubList.append(rospy.Publisher('/joint41_effort_controller/command', Float64, queue_size=36))

    num_joints = len(pubList)   
    scaled_torque = -1         #-0.8

    rate = rospy.Rate(10)  # 5 Hz

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
