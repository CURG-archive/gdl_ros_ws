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
from pylearn_classifier_gdl.srv import  CalculateGraspsService
from pylearn_classifier_gdl.srv import CalculateGraspsServiceRequest
from collections import namedtuple

from rgbd_listener import RGBDListener


if sys.version_info[0] < 3:
    import Tkinter as Tk
else:
    import tkinter as Tk


# grasp named tuple with score, dof_values, joint_values, pose
# pose named tuple
grasp = namedtuple("grasp", "score dof_values joint_values pose")
pose = namedtuple("pose", "trans_x trans_y trans_z rot_x rot_y rot_z rot_w")

class GUI():
    def __init__(self):

        #show live stream until capture button is pressed
        self._still_captured = False
        self.rgbd_listener = RGBDListener()

        #grasp list
        self.grasp_list = []
        #current grasp id
        self.current_grasp = 0


        try:
            rospy.wait_for_service('calculate_grasps_service', timeout=60)
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


        # Grasp navigation

        # Current Grasp

        self.current_grasp_label_text = Tk.StringVar()
        self.current_grasp_label_text.set("0 / 0")
        current_grasp_label = Tk.Label(master=self.root, textvariable=self.current_grasp_label_text)
        #next button
        self.button_next_grasp = Tk.Button(master=self.root, text="Next Grasp", command=self.goto_next_grasp)
        #prev button
        self.button_prev_grasp = Tk.Button(master=self.root, text="Prev Grasp", command=self.goto_prev_grasp)

        self.button_next_grasp.config(state="disabled")
        self.button_prev_grasp.config(state="disabled")

        current_grasp_label.pack(side=Tk.RIGHT)
        self.button_next_grasp.pack(side=Tk.RIGHT)
        self.button_prev_grasp.pack(side=Tk.RIGHT)


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

    def goto_next_grasp(self, *args):
        rospy.loginfo('next button pressed...')
        self.current_grasp += 1
        if self.current_grasp > len(self.grasp_list):
            self.current_grasp = 1

        self.current_grasp_label_text.set("%s / %s" % (self.current_grasp, len(self.grasp_list)))

        # Publish grasp info on rostopics "gdl_joint_states" and "gdl_robot_pose"
        grasp = self.grasp_list[self.current_grasp-1]

    def goto_prev_grasp(self, *args):
        rospy.loginfo('prev button pressed...')
        self.current_grasp -= 1
        if self.current_grasp == 0:
            self.current_grasp = 1

        self.current_grasp_label_text.set("%s / %s" % (self.current_grasp, len(self.grasp_list)))

        # Publish grasp info on rostopics "gdl_joint_states" and "gdl_robot_pose"
        grasp = self.grasp_list[self.current_grasp-1]


    def get_grasps_button_cb(self, *args):

        rospy.wait_for_service('calculate_grasps_service')
        try:
            calculate_grasps = rospy.ServiceProxy('calculate_grasps_service', CalculateGraspsService)

            req = CalculateGraspsServiceRequest(self.image.flatten(), self.mask.flatten())

            self.grasps = calculate_grasps(req)
        except rospy.ServiceException, e:
            rospy.loginfo("Service call failed: %s" % e)
        rospy.loginfo(self.grasps)

    #this is called from within TK.mainloop()
    #it updates the image to be the most recently captured from the kinect.
    def update_image(self):
        #print("updating image")
        self.set_capture_image(self.rgbd_listener.rgbd_image)
        self.draw()
        if not self._still_captured:
            self.root.after(1000, self.update_image)

    def set_capture_image(self, img):
        self.image = img
        plt.subplot(211)
        plt.title("capture")
        self.plt_image = plt.imshow(img[:, :, 0:3])

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