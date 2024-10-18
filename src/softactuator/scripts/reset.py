#!/usr/bin/env python

import rospy
from std_srvs.srv import Empty

def reset_simulation():
    # Initialize ROS node
    rospy.init_node('reset_simulation_client', anonymous=True)
    
    # Wait for the service to be available
    rospy.wait_for_service('/gazebo/reset_simulation')
    
    try:
        # Create a service proxy
        reset_simulation = rospy.ServiceProxy('/gazebo/reset_simulation', Empty)
        
        # Call the service
        response = reset_simulation()
        
        rospy.loginfo("Simulation has been reset.")
        
    except rospy.ServiceException as e:
        rospy.logerr("Service call failed: {}".format(e))

def main():
    reset_simulation()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
