 #!/usr/bin/python
#
# Send joint values to UR5 using messages
#

from std_msgs.msg import Header
from std_msgs.msg import Float64
from sensor_msgs.msg import JointState
import rospy


def main():

    rospy.init_node('legs')
    pubList=[]
    #pubList.append(rospy.Publisher('/joint01_effort_controller/command', Float64, queue_size=11))
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
    #pub = rospy.Publisher('/joint01_effort_controller/command',Float64,queue_size=10)
    
    num_joints = len(pubList)   
    scaled_torque = 0.5

    rate = rospy.Rate(5)  # 1 Hz (1 cycle per second)

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