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

    num_joints = len(pubList)
    scaled_torque = 0.2   
    down_torque = -0.3

    while not rospy.is_shutdown():
            
        for j in range(num_joints):
            pubList[j].publish(Float64(scaled_torque))

            print('start')

        rospy.sleep(0.1)

        for j in range(num_joints):
            pubList[j].publish(Float64(down_torque))

            print("--------")

        rospy.sleep(0.1)  
        #rate.sleep()  # Sleep for the defined rate (1 Hz)

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        print("Program interrupted before completion")