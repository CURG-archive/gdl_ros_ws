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
from pylearn_classifier_gdl.srv import CalculateGraspsService
from pylearn_classifier_gdl.srv import CalculateGraspsServiceRequest
from cloud_mesher import CloudMesher

from rgbd_listener import RGBDListener
from grasp_publisher import GraspPublisher

from skimage.segmentation import mark_boundaries

from graspit_msgs.msg import Grasp

import os

if sys.version_info[0] < 3:
    import Tkinter as Tk
else:
    import tkinter as Tk

class MockGrasp():

    def __init__(self, pose, joint_values=(0, 0, 0, 0, 0, 0, 0, 0)):
        self.pose = pose
        self.joint_values = joint_values


class GUI():
    def __init__(self):

        #show live stream until capture button is pressed
        self._still_captured = False
        self.rgbd_listener = RGBDListener()
        self.cloud_mesher = CloudMesher()

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
        self.button_run_grasp_server = Tk.Button(master=self.root, text='get grasps', command=self.get_grasps_button_cb)
        self.button_run_grasp_server.config(state="disabled")

        button_quit.pack(side=Tk.LEFT)
        button_capture.pack()
        button_reset_segmentation.pack()
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

        #Execute grasp button
        self.button_execute_grasp = Tk.Button(master=sideFrame, text="Exec Grasp", command=self.exec_grasp_cb)
        self.grasp_pub = rospy.Publisher('/graspit/grasps', Grasp)


        self.button_next_grasp.config(state="disabled")
        self.button_prev_grasp.config(state="disabled")

        current_grasp_label.pack(side=Tk.TOP)
        current_grasp_energy.pack(side=Tk.TOP)
        self.button_execute_grasp.pack(side=Tk.BOTTOM)
        current_grasp_xyz.pack(side=Tk.BOTTOM)
        self.button_next_grasp.pack(side=Tk.BOTTOM)
        self.button_prev_grasp.pack(side=Tk.BOTTOM)



        self.canvas.get_tk_widget().pack(side=Tk.TOP, fill=Tk.BOTH, expand=1)

        self.rgbd_listener.listen()
        self.cloud_mesher.listen()

        Tk.mainloop()

    def quit_button_cb(self, *args):
        rospy.loginfo('quit button press...')
        self.root.quit()
        self.root.destroy()

    def capture_button_cb(self, *args):
        rospy.loginfo("capture press...")
        self._still_captured = True
        self.cloud_mesher.is_capturing = False
        #self.pc_manager.publish_cloud()
        self.cloud_mesher.run_service()

        self.button_run_grasp_server.config(state="active")

    def reset_seg_button_cb(self, *args):
        rospy.loginfo("Resetting segmentation...")
        self.rgbd_listener.resetSlic()

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

    def exec_grasp_cb(self, *args):
        grasp_msg = Grasp()

        grasp_msg.grasp_type = Grasp.TYPE_FINGERTIP

        grasp_msg.pre_grasp_pose = self.grasp_list[self.current_grasp - 1].pose
        grasp_msg.final_grasp_pose = self.grasp_list[self.current_grasp - 1].pose

        self.grasp_pub.publish(grasp_msg)


    def get_grasps_button_cb(self, *args):

        rospy.wait_for_service('calculate_grasps_service')
        try:
            calculate_grasps = rospy.ServiceProxy('calculate_grasps_service', CalculateGraspsService)

            req = CalculateGraspsServiceRequest(self.image.flatten(), self.mask.flatten())

            response = calculate_grasps(req)

            heatmaps = np.array(response.heatmaps)
            heatmaps = heatmaps.reshape(response.heatmap_dims)

            save_path = self.cloud_mesher.time_dir_full_filepath
            save_path += "heatmaps/"

            if not os.path.exists(save_path):
                os.mkdir(save_path)

            for i in range(heatmaps.shape[0]):
                np.savetxt(save_path + str(i) + '.txt', heatmaps[i])

            rospy.loginfo("Heatmaps saved")

        except rospy.ServiceException, e:
            rospy.loginfo("Service call failed: %s" % e)



        #     self.grasp_list = response.grasps
        #
        #     for grasp in self.grasp_list:
        #         old_x = grasp.pose.position.x
        #         old_y = grasp.pose.position.y
        #         old_z = grasp.pose.position.z
        #
        #         grasp.pose.position.x = old_z - 0.05
        #         grasp.pose.position.y = -old_x
        #         grasp.pose.position.z = -old_y
        #
        # except rospy.ServiceException, e:
        #     rospy.loginfo("Service call failed: %s" % e)
        # rospy.loginfo(self.grasp_list)
        #
        # if len(self.grasp_list) > 0:
        #     self.button_next_grasp.config(state="active")
        #     self.button_prev_grasp.config(state="active")
        #     self.current_grasp = 1
        #
        #     grasp = self.grasp_list[0]
        #     self.current_grasp_energy_text.set("Energy = %s" % grasp.grasp_energy)
        #     self.grasp_publisher.publish_grasp(grasp)
        #
        # else:
        #     self.button_next_grasp.config(state="disabled")
        #     self.button_prev_grasp.config(state="disabled")
        #     self.current_grasp = 0


        self.current_grasp_label_text.set("%s / %s" % (self.current_grasp, len(self.grasp_list)))


    #this is called from within TK.mainloop()
    #it updates the image to be the most recently captured from the kinect.
    def update_image(self):
        #print("updating image")
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

        convertedImg = img[:, :, 0:3][:, :, ::-1]
        # bgr8 to rgb8 and throw out depth

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


        # slic = self.rgbd_listener.getSlic()
        # rospy.loginfo("seg # =%s" % slic[x_pos, y_pos])
        # self.mask = np.in1d(slic.ravel(), [slic[x_pos, y_pos]]).reshape(slic.shape)

        self.mask = np.zeros((480, 640))
        self.mask[x_pos, y_pos] = 100
        self.mask = gaussian_filter(self.mask, sigma=10)

        self.set_mask_image(self.mask)
        self.draw()

if __name__ == "__main__":
    rospy.init_node('ui_node')
    gui = GUI()