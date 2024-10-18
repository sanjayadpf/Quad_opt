import rospy
import math
from gazebo_msgs.msg import ModelStates
from gazebo_msgs.srv import GetJointProperties

class FrictionAdjustPlugin:
    def __init__(self):
        self.model_name = "softactuator"
        self.joint_name = "softactuator/joint11"
        self.joint_position = 0.0

    def model_states_callback(self, msg):
        try:
            idx = msg.name.index(self.model_name)
            self.joint_position = msg.position[idx]
        except ValueError:
            rospy.logerr("Model not found in ModelStates message")

    def set_joint_friction(self):
        try:
            joint_properties = rospy.ServiceProxy('/gazebo/get_joint_properties', GetJointProperties)
            resp = joint_properties(self.joint_name)
            friction = self.calculate_friction(self.joint_position)
            resp.friction = friction
        except rospy.ServiceException as e:
            rospy.logerr("Service call failed: %s" % e)

    def calculate_friction(self, angle):
        # Calculate friction based on the joint angle
        # You can define your own function here to adjust friction dynamically
        print(angle)
        return 0.1 * angle  # Example: Friction increases linearly with joint angle

    def main(self):
        rospy.init_node('friction_adjust_plugin')
        rospy.Subscriber('/gazebo/model_states', ModelStates, self.model_states_callback)
        rate = rospy.Rate(10)  # Adjust rate as needed
        while not rospy.is_shutdown():
            self.set_joint_friction()
            rate.sleep()

if __name__ == '__main__':
    try:
        plugin = FrictionAdjustPlugin()
        plugin.main()
    except rospy.ROSInterruptException:
        pass
