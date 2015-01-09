#!/usr/bin/env python
import rospy
import image_geometry
from sensor_msgs.msg import CameraInfo
import math

from uvd_xyz_conversion.srv import UVDTOXYZ

class UVD_XYZ_Converter():

    def __init__(self, camera_param_topic='/camera/rgb/camera_info'):
        self.camera_param_topic = camera_param_topic

        rospy.Subscriber(self.camera_param_topic, CameraInfo, self.camera_info_cb)
        self.camera_info = None

        self.pinhole_model = image_geometry.PinholeCameraModel()

        self.conversion_service = rospy.Service('uvd_to_xyz', UVDTOXYZ, self.conversion_service_cb)

    #get the camera info from the kinect
    def camera_info_cb(self, msg):
        if not self.camera_info:
            self.camera_info = msg
            self.pinhole_model.fromCameraInfo(self.camera_info)

    #take a uvd point from the kinect and
    #convert it to x,y,z
    def conversion_service_cb(self, msg):
        u, v, d = msg.u, msg.v, msg.d

        x, y, z = self.pinhole_model.projectPixelTo3dRay((u, v))
        h = math.hypot(x,z)
        return x*h, y*h, z*h

if __name__ == "__main__":

    rospy.init_node("UVD_XYZ_Conversion_Node")
    uvd_xyz_converter = UVD_XYZ_Converter()
    rospy.spin()

