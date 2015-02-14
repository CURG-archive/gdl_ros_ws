#!/usr/bin/env python
import matplotlib
import sys
matplotlib.use('TkAgg')
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from matplotlib import pylab as plt
import numpy as np
import roslib
import rospy
from scipy.ndimage import gaussian_filter

from cloud_mesher import CloudMesher
from heatmap_generator import HeatmapGenerator

from rgbd_listener import RGBDListener
from grasp_publisher import GraspPublisher

from skimage.segmentation import mark_boundaries

from graspit_msgs.msg import Grasp


if sys.version_info[0] < 3:
    import Tkinter as Tk
else:
    import tkinter as Tk


class GUI():

    def __init__(self):

        #show live stream until capture button is pressed
        self._still_captured = False

        #different service providers
        self.rgbd_listener = RGBDListener()
        self.cloud_mesher = CloudMesher()
        self.grasp_publisher = GraspPublisher()
        self.heatmap_generator = HeatmapGenerator()

        self.root = Tk.Tk()
        self.root.wm_title("Image Capture GUI")
        self.root.protocol('WM_DELETE_WINDOW', self.quit_button_cb)
        self.root.after(1000, self.update_image)

        fig = plt.figure(figsize=(8, 8))

        self.canvas = FigureCanvasTkAgg(fig, master=self.root)
        pkg_dir = roslib.packages.get_pkg_dir('ui')
        self.image = plt.imread(pkg_dir + '/san_jacinto.jpg')
        self.depth_image = np.zeros((480, 640))
        self.mask = np.zeros((480, 640))

        self.set_capture_image(self.image)
        self.set_depth_image(self.depth_image)

        self.set_mask_image(self.mask)

        self.draw()

        fig.canvas.mpl_connect('button_press_event', self.on_click)

        button_capture = Tk.Button(master=self.root, text='Capture', command=self.capture_button_cb)
        button_reset_segmentation = Tk.Button(master=self.root, text='Reset Segmentation', command=self.reset_seg_button_cb)
        button_quit = Tk.Button(master=self.root, text='Quit', command=self.quit_button_cb)
        self.button_run_grasp_server = Tk.Button(master=self.root, text='get grasps', command=self.get_heatmaps_button_cb)
        self.button_run_grasp_server.config(state="disabled")

        button_quit.pack(side=Tk.LEFT)
        button_capture.pack()
        button_reset_segmentation.pack()
        self.button_run_grasp_server.pack()

        self.canvas.get_tk_widget().pack(side=Tk.TOP, fill=Tk.BOTH, expand=1)

        self.rgbd_listener.listen()
        self.cloud_mesher.listen()

        Tk.mainloop()

    def quit_button_cb(self, *args):

        self.root.quit()
        self.root.destroy()

    def capture_button_cb(self, *args):

        self._still_captured = True
        self.cloud_mesher.is_capturing = False

        self.cloud_mesher.run_service()

        self.button_run_grasp_server.config(state="active")

    def reset_seg_button_cb(self, *args):

        self.rgbd_listener.resetSlic()

    def get_heatmaps_button_cb(self, *args):
        self.heatmap_generator.get_heatmaps(self.image, self.mask, self.cloud_mesher.time_dir_full_filepath)

    def update_image(self):
        self.set_capture_image(self.rgbd_listener.rgbd_image,
                               segments_slic=self.rgbd_listener.slic)
        self.set_depth_image(self.rgbd_listener.rgbd_image[:, :, 3],
                               segments_slic=self.rgbd_listener.slic)
        self.draw()
        if not self._still_captured:
            self.root.after(1000, self.update_image)

    def set_capture_image(self, img, segments_slic=None):
        self.image = img
        plt.subplot(221)
        plt.title("capture")

        # bgr8 to rgb8 and throw out depth
        convertedImg = img[:, :, 0:3][:, :, ::-1]

        if segments_slic is None:
            self.plt_image = plt.imshow(convertedImg)
        else:
            img_with_boundaries = mark_boundaries(convertedImg, segments_slic)
            self.plt_image = plt.imshow(img_with_boundaries)

    def set_depth_image(self, img, segments_slic=None):
        self.depth_image = img
        plt.subplot(223)
        plt.title("depth")

        if segments_slic is None:
            self.depth_plt_image = plt.imshow(self.depth_image)
        else:
            img_with_boundaries = mark_boundaries(self.depth_image, segments_slic)
            self.depth_plt_image = plt.imshow(img_with_boundaries)

    def set_mask_image(self, img):
        self.mask = img
        plt.subplot(224)
        plt.title("mask")
        self.plt_mask_image = plt.imshow(img)

    def draw(self):
        self.root.update()
        self.canvas.draw()
        self.canvas.show()

    def on_click(self, event):
        if event.xdata is None:
            return
        rospy.loginfo('button=%d, x=%d, y=%d, xdata=%f, ydata=%f'%(
            event.button, event.x, event.y, event.xdata, event.ydata))

        rospy.loginfo(self.image[event.ydata, event.xdata, 3])

        x_pos = event.ydata
        y_pos = event.xdata

        self.mask = np.zeros((480, 640))
        self.mask[x_pos, y_pos] = 100
        self.mask = gaussian_filter(self.mask, sigma=10)

        self.set_mask_image(self.mask)
        self.draw()

if __name__ == "__main__":
    rospy.init_node('ui_node')
    gui = GUI()