#!/usr/bin/env python
import rospy
from grasp_server.srv import CalculateGraspsService

import cPickle
from rospy.numpy_msg import numpy_msg
from rospy_tutorials.msg import Floats

import numpy as np

class GraspServer:

    def __init__(self):
        self.pylearn_model = None
        self.service = rospy.Service('calculate_grasps_service', CalculateGraspsService, self.service_request_handler)
        rospy.spin()

    def service_request_handler(self, request):
        rospy.loginfo("received request")
        rgbd = np.array(request.rgbd).reshape((480, 640, 4))
        mask = np.array(request.mask).reshape((480, 640))

        return []


if __name__ == "__main__":

    rospy.init_node('grasp_server_node')
    rospy.loginfo("starting grasp_server_node!!!!!!!!!!!!!!!!!!!!!!!!!!!")
    grasp_server = GraspServer()
