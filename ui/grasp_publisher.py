

class GraspPublisher():

    def __init__(self, joint_state_topic='gdl_joint_states', hand_pose_topic='gdl_robot_pose'):
        self.joint_state_topic = joint_state_topic
        self.hand_pose_topic = hand_pose_topic

    def publish_grasp(self,grasp):
        # Publish grasp info on rostopics "gdl_joint_states" and "gdl_robot_pose"

        raise NotImplementedError
