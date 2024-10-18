#!/usr/bin/env python

import rospy
from gazebo_msgs.srv import GetModelState, GetModelStateRequest, GetModelStateResponse
from std_srvs.srv import Empty, EmptyRequest

def get_model_state(model_name):
    rospy.wait_for_service('/gazebo/get_model_state')
    try:
        get_model_state_service = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
        request = GetModelStateRequest()
        request.model_name = model_name
        response = get_model_state_service(request)
        if response.success:
            return response
        else:
            rospy.logerr("Failed to get model state for {}".format(model_name))
            return None
    except rospy.ServiceException as e:
        rospy.logerr("Service call failed: {}".format(e))
        return None

def reset_simulation():
    rospy.wait_for_service('/gazebo/reset_simulation')
    try:
        reset_simulation_service = rospy.ServiceProxy('/gazebo/reset_simulation', Empty)
        request = EmptyRequest()
        reset_simulation_service(request)
        rospy.loginfo("Simulation reset successfully.")
    except rospy.ServiceException as e:
        rospy.logerr("Service call failed: {}".format(e))

def main():
    rospy.init_node('gazebo_services_client', anonymous=True)

    model_name = "softactuator"  # Change this to your model's name
    
    # Get model state
    response = get_model_state(model_name)
    if response:
        rospy.loginfo("Model Pose: \n{}".format(response.pose))
        rospy.loginfo("Model Twist: \n{}".format(response.twist))
    
    # Reset simulation
    reset_simulation()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass