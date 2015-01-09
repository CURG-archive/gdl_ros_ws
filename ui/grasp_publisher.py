from rospy import Publisher
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Pose

class GraspPublisher():

    def __init__(self, joint_state_topic='gdl_joint_states', hand_pose_topic='gdl_robot_pose'):
        self.joint_state_topic = joint_state_topic
        self.hand_pose_topic = hand_pose_topic

        self.gdl_joint_states_pub = Publisher(self.joint_state_topic, JointState, queue_size=10)
        self.gdl_hand_pose_pub = Publisher(self.hand_pose_topic, Pose, queue_size=10)

    def publish_grasp(self,grasp):
        # Publish grasp info on rostopics "gdl_joint_states" and "gdl_robot_pose"
        self.gdl_joint_states_pub.publish(grasp.joint_values)
        self.gdl_hand_pose_pub.publish(grasp.pose)
