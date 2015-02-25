import numpy as np
import rospy
from sensor_msgs.msg import Image, PointCloud2
from cv_bridge import CvBridge, CvBridgeError

# Segmentation
from skimage.segmentation import slic

class RGBDListener():

    def __init__(self,
                 depth_topic="/camera/depth_registered/image_raw",
                 rgb_topic="/camera/rgb/image_color"):

        self.depth_topic = depth_topic
        self.rgb_topic = rgb_topic

        self.data = None
        self.slic = None


        #this is the current image
        self.rgbd_image = np.zeros((480, 640, 4))
        self.bridge = CvBridge()


    def depth_image_callback(self, data):
        depth_image_np = self.image2numpy(data)
        self.depth_image = depth_image_np[:, :, 0]
        self.rgbd_image[:, :, 3] = self.depth_image
        self.rgbd_image[:, :, 3] /= 1000.0


    def rgb_image_callback(self, data):
        self.data = data
        rgbd_image_np = self.image2numpy(data)
        self.rgbd_image[:, :, 0:3] = (rgbd_image_np / 255.0)


    def getSlic(self):
        self.slic = slic(self.depth_image,
                    n_segments=50,
                    compactness=.001,
                    sigma=1,
                    multichannel=False)
        return self.slic

    def resetSlic(self):
        self.slic = None

    #this method from:
    #https://github.com/rll/sushichallenge/blob/master/python/brett2/ros_utils.py
    def image2numpy(self, image):
        if image.encoding == 'rgb8':
            return np.fromstring(image.data, dtype=np.uint8).reshape(image.height, image.width, 3)[:, :, ::-1]
        if image.encoding == 'bgr8':
            return np.fromstring(image.data, dtype=np.uint8).reshape(image.height, image.width, 3)
        elif image.encoding == 'mono8':
            return np.fromstring(image.data, dtype=np.uint8).reshape(image.height, image.width)
        elif image.encoding == '32FC1':
            return np.fromstring(image.data, dtype=np.float32).reshape(image.height, image.width)
        elif image.encoding == '16UC1':
            return self.bridge.imgmsg_to_cv2(image, desired_encoding="16UC1")
        elif image.encoding == 'bayer_grbg8':
            return self.bridge.imgmsg_to_cv2(image, desired_encoding="32FC1")
        else:
            raise Exception

    def listen(self, init_node=False):

        if init_node:
            rospy.init_node('listener', anonymous=True)

        rospy.Subscriber(self.depth_topic, Image, self.depth_image_callback, queue_size=1)
        rospy.Subscriber(self.rgb_topic, Image, self.rgb_image_callback, queue_size=1)
