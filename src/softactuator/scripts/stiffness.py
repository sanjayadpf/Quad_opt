import rospy
from your_custom_srv import SetJointStiffnessDamping  # Import your custom service message

# Initialize the ROS node
rospy.init_node('set_joint_properties')

# Wait for the service to become available
rospy.wait_for_service('set_joint_stiffness_damping')

try:
    # Create a service proxy
    set_stiffness_damping = rospy.ServiceProxy('set_joint_stiffness_damping', SetJointStiffnessDamping)

    # Call the service
    response = set_stiffness_damping(joint_name="joint10", stiffness=1000.0, damping=10.0)
    
    if response.success:
        print("Successfully set joint stiffness and damping.")
    else:
        print("Failed to set joint stiffness and damping.")
except rospy.ServiceException as e:
    print("Service call failed: %s" % e)