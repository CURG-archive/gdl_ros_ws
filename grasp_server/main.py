
import rospy
import cPickle


class GraspServer:

    def __init__(self):
        self.pylearn_model = None
        self.service = rospy.Service('calculate_grasps_service', CalculateGraspsService, self.service_request_handler)
        rospy.spin()

    def service_request_handler(self, request):
        rgbd_image = request.image
        mask = request.mask
        return []


if __name__ == "__main__":
    rospy.init_node('grasp_server_node')
    grasp_server = GraspServer()
