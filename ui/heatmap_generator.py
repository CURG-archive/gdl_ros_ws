
import rospy
import numpy as np
import os

from pylearn_classifier_gdl.srv import CalculateGraspsService
from pylearn_classifier_gdl.srv import CalculateGraspsServiceRequest

import grasp_priors

import cPickle
import matplotlib.pyplot as plt
import cv2

GRASP_PRIORS_DIR = os.path.expanduser("~/grasp_deep_learning/data/grasp_datasets/5_grasp_priors/")

class HeatmapGenerator():

    def __init__(self):
        pass

    def get_heatmaps(self, image, save_path, mask=np.zeros((480, 640))):
        rospy.wait_for_service('calculate_grasps_service')
        try:
            calculate_grasps = rospy.ServiceProxy('calculate_grasps_service', CalculateGraspsService)

            req = CalculateGraspsServiceRequest(image.flatten())

            response = calculate_grasps(req)

            heatmaps = np.array(response.heatmaps)
            heatmaps = heatmaps.reshape(response.heatmap_dims)
            model_name = response.model_name

            grasp_priors_list_path = None
            for path in os.listdir(GRASP_PRIORS_DIR):
                if path in model_name:
                    grasp_priors_list_path = GRASP_PRIORS_DIR + path + '/grasp_priors_list.pkl'
                    break

            if not path:
                print "Error! Grasp priors list not found for current model <%s>" % model_name
                assert False


            f = open(grasp_priors_list_path)
            grasp_priors_list = cPickle.load(f)

            self._save_heatmaps_config(save_path, response.heatmap_dims)
            self._save_heatmaps(save_path, heatmaps)

            self._save_grasp_priors_config(save_path, grasp_priors_list)
            self._save_grasp_priors(save_path, grasp_priors_list)

            self._save_rgbd(save_path, image.flatten())

        except rospy.ServiceException, e:
            rospy.loginfo("Service call failed: %s" % e)

    def _save_heatmaps(self, save_path, heatmaps ):

        heatmaps_save_path = save_path + "heatmaps/"

        if not os.path.exists(heatmaps_save_path):
            os.mkdir(heatmaps_save_path)

        if not os.path.exists(save_path + "imgs/"):
            os.mkdir(save_path + "imgs/")
        save_image_path = save_path + "imgs/"

        for i in range(heatmaps.shape[-1]):
            np.savetxt(heatmaps_save_path + str(i) + '.txt', heatmaps[:, :, i])
            plt.imsave(save_image_path + str(i) + '.png', heatmaps[:, :, i])

        rospy.loginfo("Heatmaps saved")

    def _save_rgbd(self, save_path, rgbd):
        np.savetxt(save_path + "rgbd.txt", rgbd)

        rgbd_shaped = rgbd.reshape((480, 640, 4))
        if not os.path.exists(save_path + "imgs/"):
            os.mkdir(save_path + "imgs/")
        save_image_path = save_path + "imgs/"

        cv2.imwrite(save_image_path + 'rgb.png', rgbd_shaped[:, :, 0:3]*255)
        plt.imsave(save_image_path + "depth.png", rgbd_shaped[:, :, 3])


    def _save_heatmaps_config(self, save_path, heatmap_dims):

        num_heatmaps = heatmap_dims[-1]
        height = heatmap_dims[0]
        width = heatmap_dims[1]

        f = open(save_path + "heatmapsConfig.txt", 'w+')

        f.write(str(num_heatmaps) + "\n")
        f.write(str(height) + "\n")
        f.write(str(width) + "\n")

        f.close()

    def _save_grasp_priors(self, save_path, grasp_priors_list):
        grasp_priors_save_path = save_path + "grasp_priors/"

        if not os.path.exists(grasp_priors_save_path):
            os.mkdir(grasp_priors_save_path)

        for i in range(len(grasp_priors_list.grasp_priors_list)):
            grasp_prior = grasp_priors_list.get_grasp_prior(i)
            jv = grasp_prior.joint_values
            np.savetxt(grasp_priors_save_path + str(i) + ".txt", jv)

    def _save_grasp_priors_config(self, save_path, grasp_priors_list):
        num_grasp_priors = len(grasp_priors_list.grasp_priors_list)
        num_dof = len(grasp_priors_list.get_grasp_prior(0).joint_values)

        f = open(save_path + "graspPriorsConfig.txt", 'w+')

        f.write(str(num_grasp_priors) + "\n")
        f.write(str(num_dof) + "\n")

        f.close()