#!/usr/bin/env python
import matplotlib
import sys
import rospy
import numpy as np

matplotlib.use('TkAgg')
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from matplotlib import pylab as plt

from cloud_mesher import CloudMesher
from heatmap_generator import HeatmapGenerator
from rgbd_listener import RGBDListener
from grasp_publisher import GraspPublisher

from moveit_msgs.msg import PlanningScene
if sys.version_info[0] < 3:
    import Tkinter as Tk
else:
    import tkinter as Tk


class GUI():

    def __init__(self):

        #show live stream until capture button is pressed
        self._still_captured = False

        self.moveit_planning_scene_msg = None
        self.mesh_list_vars = {}
        self.check_buttons = []
        
        self.refined_planning_scene_publisher = rospy.Publisher("/planning_scene", PlanningScene)

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

        self.image = np.zeros((480, 640, 3))
        self.depth_image = np.zeros((480, 640))

        self.set_capture_image(self.image)
        self.set_depth_image(self.depth_image)

        self.draw()

        button_capture = Tk.Button(master=self.root, text='Capture', command=self.capture_button_cb)
        button_quit = Tk.Button(master=self.root, text='Quit', command=self.quit_button_cb)
        self.button_send_meshes = Tk.Button(master=self.root, text='Send Selected Meshes', command=self.send_meshes_cb)

        self.world_name_text_box = Tk.Entry(master=self.root)
        self.world_name_text_box.pack()
        self.world_name_text_box.insert(0, "world")

        button_quit.pack(side=Tk.LEFT)
        button_capture.pack()
        self.button_send_meshes.pack()

        # List of 'models'
        self.meshListFrame = Tk.Frame(self.root)
        self.meshListFrame.pack(side=Tk.RIGHT)

        self.canvas.get_tk_widget().pack(side=Tk.TOP, fill=Tk.BOTH, expand=1)

        self.rgbd_listener.listen()
        self.cloud_mesher.listen()

        Tk.mainloop()

    def quit_button_cb(self, *args):

        self.root.quit()
        self.root.destroy()

    def send_meshes_cb(self, *args):
        planning_scene_msg = PlanningScene()
        planning_scene_msg.is_diff = self.moveit_planning_scene_msg.is_diff
        planning_scene_msg.allowed_collision_matrix = self.moveit_planning_scene_msg.allowed_collision_matrix
        planning_scene_msg.name = self.moveit_planning_scene_msg.name

        for collision_object in self.moveit_planning_scene_msg.world.collision_objects:
            mesh_id = collision_object.id
            if self.mesh_list_vars[mesh_id].get() == '1':
                planning_scene_msg.world.collision_objects.append(collision_object)

        self.refined_planning_scene_publisher.publish(planning_scene_msg)

    def capture_button_cb(self, *args):

        self._still_captured = True
        self.cloud_mesher.is_capturing = False

        world_name = self.world_name_text_box.get()
        self.moveit_planning_scene_msg, mesh_names = self.cloud_mesher.run_service(world_name)

        self.mesh_list_vars = dict([mesh_name, Tk.Variable()] for mesh_name in mesh_names)

        #remove old check buttons
        for check_button in self.check_buttons:
            check_button.pack_forget()
            check_button.destroy()

        #add new check buttons
        for meshName in self.mesh_list_vars:
            check_button = Tk.Checkbutton(self.meshListFrame, text=meshName, variable=self.mesh_list_vars[meshName])
            check_button.pack()
            self.check_buttons.append(check_button)

        self.heatmap_generator.get_heatmaps(self.image, self.cloud_mesher.time_dir_full_filepath)

        self.button_send_meshes.config(state="active")

    def update_image(self):
        self.set_capture_image(self.rgbd_listener.rgbd_image,
                               segments_slic=self.rgbd_listener.slic)
        self.set_depth_image(self.rgbd_listener.rgbd_image[:, :, 3],
                               segments_slic=self.rgbd_listener.slic)
        self.draw()
        if not self._still_captured:
            self.root.after(1000, self.update_image)

    def set_capture_image(self, img, segments_slic=None):
        self.image = np.copy(img)
        plt.subplot(211)
        plt.title("capture")

        # bgr8 to rgb8 and throw out depth
        convertedImg = img[:, :, 0:3][:, :, ::-1]

        self.plt_image = plt.imshow(convertedImg)


    def set_depth_image(self, img, segments_slic=None):
        self.depth_image = np.copy(img)
        plt.subplot(212)
        plt.title("depth")

        self.depth_plt_image = plt.imshow(self.depth_image)

    def draw(self):
        self.root.update()
        self.canvas.draw()
        self.canvas.show()


if __name__ == "__main__":
    rospy.init_node('ui_node')
    gui = GUI()