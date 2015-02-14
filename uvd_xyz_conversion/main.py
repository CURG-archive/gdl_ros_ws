#!/usr/bin/env python
import rospy
import image_geometry
from sensor_msgs.msg import CameraInfo
import math

from uvd_xyz_conversion.srv import UVDTOXYZ, XYZTOUV

class UVD_XYZ_Converter_Service():

    def __init__(self, camera_param_topic='/camera/rgb/camera_info'):
        self.camera_param_topic = camera_param_topic

        camera_info = rospy.wait_for_message(self.camera_param_topic, CameraInfo)
        self.pinhole_model = image_geometry.PinholeCameraModel()
        self.pinhole_model.fromCameraInfo(camera_info)

        self.uvd_xyz_converter = UVD_XYZ_Converter(self.pinhole_model)

        self.conversion_service = rospy.Service('uvd_to_xyz', UVDTOXYZ, self.uvd_xyz_cb)
        self.conversion_service = rospy.Service('xyz_to_uvd', XYZTOUV, self.xyz_uv_cb)

    def uvd_xyz_cb(self, msg):
        return self.uvd_xyz_converter.convert_uvd_xyz(msg.u, msg.v, msg.d)

    def xyz_uv_cb(self, msg):
        return self.uvd_xyz_converter.convert_xyz_uv(msg.x, msg.y, msg.z)


class UVD_XYZ_Converter():

    def __init__(self, pinhole_model):
        self.pinhole_model = pinhole_model

    def convert_uvd_xyz(self, u, v, d):
        x, y, z = self.pinhole_model.projectPixelTo3dRay((u, v))
        tx_fx = self.pinhole_model.Tx() / self.pinhole_model.fx()
        x, y, z = d * (x + tx_fx)-tx_fx, d*y, d*z
        return x, y, z

    def convert_xyz_uv(self, x, y, z):
        return self.pinhole_model.project3dToPixel((x, y, z))

if __name__ == "__main__":

    rospy.init_node("UVD_XYZ_Conversion_Node")
    uvd_xyz_converter = UVD_XYZ_Converter_Service()
    rospy.spin()

