#!/usr/bin/env python3

import rospy
from gazebo_msgs.msg import LinkStates

def link_states_callback(msg):
    for name in msg.name:
        print(name)

def main():
    rospy.init_node('print_link_names')

    rospy.Subscriber("/gazebo/link_states", LinkStates, link_states_callback)

    rospy.loginfo("Printing link names from /gazebo/link_states topic...")
    rospy.spin()

if __name__ == '__main__':
    main()
