
import rospy
import numpy as np
import os

from pylearn_classifier_gdl.srv import CalculateGraspsService
from pylearn_classifier_gdl.srv import CalculateGraspsServiceRequest

import grasp_priors

import cPickle


class HeatmapGenerator():

    def __init__(self):
        pass

    def get_heatmaps(self, image, mask, save_path):
        rospy.wait_for_service('calculate_grasps_service')
        try:
            calculate_grasps = rospy.ServiceProxy('calculate_grasps_service', CalculateGraspsService)

            req = CalculateGraspsServiceRequest(image.flatten(), mask.flatten())

            response = calculate_grasps(req)

            heatmaps = np.array(response.heatmaps)
            heatmaps = heatmaps.reshape(response.heatmap_dims)
            model_name = response.model_name

            self._save_heatmaps_config(save_path, response.heatmap_dims)
            self._save_heatmaps(save_path, heatmaps)

            self._save_grasp_priors_config(save_path)
            self._save_grasp_priors(save_path, model_name)

            self._save_rgbd(save_path, image.flatten())

        except rospy.ServiceException, e:
            rospy.loginfo("Service call failed: %s" % e)

    def _save_heatmaps(self, save_path, heatmaps ):

        heatmaps_save_path = save_path + "heatmaps/"

        if not os.path.exists(heatmaps_save_path):
            os.mkdir(heatmaps_save_path)

        for i in range(heatmaps.shape[-1]):
            np.savetxt(heatmaps_save_path + str(i) + '.txt', heatmaps[:, :, i])

        rospy.loginfo("Heatmaps saved")

    def _save_rgbd(self, save_path, rgbd):
         np.savetxt(save_path + "rgbd.txt", rgbd)

    def _save_heatmaps_config(self, save_path, heatmap_dims):

        num_heatmaps = heatmap_dims[-1]
        height = heatmap_dims[0]
        width = heatmap_dims[1]

        f = open(save_path + "heatmapsConfig.txt")

        f.write(str(num_heatmaps) + "\n")
        f.write(str(height) + "\n")
        f.write(str(width) + "\n")

        f.close()

    def _save_grasp_priors(self, save_path, model_name):
        grasp_priors_save_path = save_path + "grasp_priors/"

        if not os.path.exists(grasp_priors_save_path):
            os.mkdir(grasp_priors_save_path)

        f = open(os.path.expanduser("~/grasp_deep_learning/data/grasp_priors/" + str(model_name) + "/grasp_priors_list.pkl"))

        grasp_priors_list = cPickle.load(f)

        for i in range(len(grasp_priors_list.grasp_priors_list)):
            grasp_prior = grasp_priors_list.get_grasp_prior(i)
            jv = grasp_prior.joint_values
            np.savetxt(grasp_priors_save_path + str(i) + ".txt", jv)


    def _save_grasp_priors_config(self, save_path):
        num_grasp_priors = 18
        num_dof = 4

        f = open(save_path + "heatmapsConfig.txt")

        f.write(str(num_grasp_priors) + "\n")
        f.write(str(num_dof) + "\n")

        f.close()