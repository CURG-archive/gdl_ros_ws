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

from rgbd_listener import RGBDListener


if sys.version_info[0] < 3:
    import Tkinter as Tk
else:
    import tkinter as Tk


class GUI():
    def __init__(self):

        #show live stream until capture button is pressed
        self._still_captured = False
        self.rgbd_listener = RGBDListener()

        try:
            rospy.wait_for_service('grasp_service', timeout=5)
        except Exception, e:
            rospy.logerr("Service call failed: %s  UI will work fine, but Grasp Server is not running "%e)

        self.root = Tk.Tk()
        self.root.wm_title("Image Capture GUI")
        self.root.protocol('WM_DELETE_WINDOW', self.quit_button_cb)
        self.root.after(1000, self.update_image)

        fig = plt.figure(figsize=(8, 8))

        self.canvas = FigureCanvasTkAgg(fig, master=self.root)
        pkg_dir = roslib.packages.get_pkg_dir('ui')
        self.image = plt.imread(pkg_dir + '/san_jacinto.jpg')
        self.mask = np.zeros((480, 640))

        self.set_capture_image(self.image)
        self.set_mask_image(self.mask)

        self.draw()

        fig.canvas.mpl_connect('button_press_event', self.on_click)

        button_capture = Tk.Button(master=self.root, text='Capture', command=self.capture_button_cb)
        button_quit = Tk.Button(master=self.root, text='Quit', command=self.quit_button_cb)
        button_run_grasp_server = Tk.Button(master=self.root, text='get grasps', command=self.get_grasps_button_cb)

        button_quit.pack(side=Tk.LEFT)
        button_capture.pack()
        button_run_grasp_server.pack()

        self.canvas.get_tk_widget().pack(side=Tk.TOP, fill=Tk.BOTH, expand=1)

        self.rgbd_listener.listen()

        Tk.mainloop()

    def quit_button_cb(self, *args):
        rospy.loginfo('quit button press...')
        self.root.quit()
        self.root.destroy()

    def capture_button_cb(self, *args):
        rospy.loginfo("capture press...")
        self._still_captured = True


    def get_grasps_button_cb(self, *args):

        rospy.wait_for_service('grasp_service')
        try:
            calculate_grasps = rospy.ServiceProxy('calculate_grasps_service', CalculateGraspsService)
            self.grasps = calculate_grasps(self.image)
        except rospy.ServiceException, e:
            rospy.loginfo("Service call failed: %s"%e)
        rospy.loginfo(self.grasps)

    #this is called from within TK.mainloop()
    #it updates the image to be the most recently captured from the kinect.
    def update_image(self):
        #print("updating image")
        self.set_capture_image(self.rgbd_listener.rgbd_image[:, :, 0:3])
        self.draw()
        if not self._still_captured:
            self.root.after(1000, self.update_image)

    def set_capture_image(self, img):
        self.image = img
        plt.subplot(211)
        plt.title("capture")
        self.plt_image = plt.imshow(img)

    def set_mask_image(self, img):
        self.mask = img
        plt.subplot(212)
        plt.title("mask")
        self.plt_mask_image = plt.imshow(img)

    def draw(self):
        self.root.update()
        self.canvas.draw()
        self.canvas.show()

    def on_click(self, event):
        rospy.loginfo('button=%d, x=%d, y=%d, xdata=%f, ydata=%f'%(
            event.button, event.x, event.y, event.xdata, event.ydata))

        x_pos = event.xdata
        y_pos = event.ydata

        self.mask = np.zeros((480, 640))
        self.mask[y_pos, x_pos] = 100
        self.mask = gaussian_filter(self.mask, sigma=10)

        self.set_mask_image(self.mask)
        self.draw()

if __name__ == "__main__":
    rospy.init_node('ui_node')
    gui = GUI()