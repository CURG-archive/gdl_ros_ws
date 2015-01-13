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

from rgbd_listener import RGBDListener
from grasp_publisher import GraspPublisher


if sys.version_info[0] < 3:
    import Tkinter as Tk
else:
    import tkinter as Tk


class GUI():
    def __init__(self):

        #show live stream until capture button is pressed
        self._still_captured = False
        self.rgbd_listener = RGBDListener()

        self.grasp_publisher = GraspPublisher()

        #grasp list
        self.grasp_list = []
        #current grasp id
        self.current_grasp = 0


        # try:
        #     rospy.wait_for_service('calculate_grasps_service', timeout=60)
        # except Exception, e:
        #     rospy.logerr("Service call failed: %s  UI will work fine, but Grasp Server is not running "%e)

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
        self.button_run_grasp_server = Tk.Button(master=self.root, text='get grasps', command=self.get_grasps_button_cb)
        self.button_run_grasp_server.config(state="disabled")

        button_quit.pack(side=Tk.LEFT)
        button_capture.pack()
        self.button_run_grasp_server.pack()


        # Grasp navigation

        # Current Grasp
        sideFrame = Tk.Frame(self.root)
        sideFrame.pack(side=Tk.RIGHT)

        self.current_grasp_label_text = Tk.StringVar()
        self.current_grasp_label_text.set("0 / 0")
        self.current_grasp_energy_text = Tk.StringVar()
        self.current_grasp_energy_text.set("Energy: n/a")
        self.current_grasp_xyz_text = Tk.StringVar()
        self.current_grasp_xyz_text.set("x: n/a, y: n/a, z: n/a")
        current_grasp_label = Tk.Label(master=sideFrame, textvariable=self.current_grasp_label_text)
        current_grasp_energy = Tk.Label(master=sideFrame, textvariable=self.current_grasp_energy_text)
        current_grasp_xyz = Tk.Label(master=sideFrame, textvariable=self.current_grasp_xyz_text)
        #next button
        self.button_next_grasp = Tk.Button(master=sideFrame, text="Next Grasp", command=self.goto_next_grasp_cb)
        #prev button
        self.button_prev_grasp = Tk.Button(master=sideFrame, text="Prev Grasp", command=self.goto_prev_grasp_cb)

        self.button_next_grasp.config(state="disabled")
        self.button_prev_grasp.config(state="disabled")

        current_grasp_label.pack(side=Tk.TOP)
        current_grasp_energy.pack(side=Tk.TOP)
        current_grasp_xyz.pack(side=Tk.BOTTOM)
        self.button_next_grasp.pack(side=Tk.BOTTOM)
        self.button_prev_grasp.pack(side=Tk.BOTTOM)


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
        self.button_run_grasp_server.config(state="active")

    def goto_next_grasp_cb(self, *args):
        rospy.loginfo('next button pressed...')
        self.current_grasp += 1
        if self.current_grasp > len(self.grasp_list):
            self.current_grasp = 1

        grasp = self.grasp_list[self.current_grasp - 1]
        self.grasp_publisher.publish_grasp(grasp)

        self.current_grasp_label_text.set("%s / %s" % (self.current_grasp, len(self.grasp_list)))
        self.current_grasp_energy_text.set("Energy = %s" % grasp.grasp_energy)
        self.current_grasp_xyz_text.set("x: %s, y: %s, z:%s" % (grasp.pose.position.x, grasp.pose.position.y, grasp.pose.position.z))

    def goto_prev_grasp_cb(self, *args):
        rospy.loginfo('prev button pressed...')
        self.current_grasp -= 1
        if self.current_grasp == 0:
            self.current_grasp = len(self.grasp_list)

        grasp = self.grasp_list[self.current_grasp - 1]
        self.grasp_publisher.publish_grasp(grasp)

        self.current_grasp_label_text.set("%s / %s" % (self.current_grasp, len(self.grasp_list)))
        self.current_grasp_energy_text.set("Energy = %s" % grasp.grasp_energy)
        self.current_grasp_xyz_text.set("x: %s, y: %s, z:%s" % (grasp.pose.position.x, grasp.pose.position.y, grasp.pose.position.z))

    def get_grasps_button_cb(self, *args):

        rospy.wait_for_service('calculate_grasps_service')
        try:
            calculate_grasps = rospy.ServiceProxy('calculate_grasps_service', CalculateGraspsService)

            req = CalculateGraspsServiceRequest(self.image.flatten(), self.mask.flatten())

            self.grasp_list = calculate_grasps(req).grasps
        except rospy.ServiceException, e:
            rospy.loginfo("Service call failed: %s" % e)
        rospy.loginfo(self.grasp_list)

        if len(self.grasp_list) > 0:
            self.button_next_grasp.config(state="active")
            self.button_prev_grasp.config(state="active")
            self.current_grasp = 1

            grasp = self.grasp_list[0]
            self.current_grasp_energy_text.set("Energy = %s" % grasp.grasp_energy)
            self.grasp_publisher.publish_grasp(grasp)

        else:
            self.button_next_grasp.config(state="disabled")
            self.button_prev_grasp.config(state="disabled")
            self.current_grasp = 0


        self.current_grasp_label_text.set("%s / %s" % (self.current_grasp, len(self.grasp_list)))


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
        self.plt_image = plt.imshow(img[:, :, 0:3][:, :, ::-1])

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

        rospy.loginfo(self.image[event.ydata, event.xdata, 3])

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