from rospy import Publisher
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Vector3
from geometry_msgs.msg import Transform
import tf
import tf_conversions
import numpy as np
import rospy
import math

class GraspPublisher():

    def __init__(self, joint_state_topic='gdl_joint_states', hand_pose_topic='gdl_robot_pose'):
        self.joint_state_topic = joint_state_topic
        self.hand_pose_topic = hand_pose_topic

        self.gdl_joint_states_pub = Publisher(self.joint_state_topic, JointState, queue_size=10)
        #self.gdl_hand_pose_pub = Publisher(self.hand_pose_topic, Transform, queue_size=10)
        self.br = tf.TransformBroadcaster()


    def publish_grasp(self, grasp):
        #this publishes the joint states of the robot to
        #the topic that the joint state publisher is subscribed to
        #it will then continue to rebroadcase these joint values until they are updated again
        self.gdl_joint_states_pub.publish(grasp.joint_values)

        #this will publish the updated transform between
        #the demo hand base link and the camera_link
        tf_out = tf_conversions.fromMsg(grasp.pose)
        tf_conv = tf_conversions.toTf(tf_out)

        self.br.sendTransform(tf_conv[0], tf_conv[1], rospy.Time.now(), 'demo_hand/base_link', 'camera_link')

