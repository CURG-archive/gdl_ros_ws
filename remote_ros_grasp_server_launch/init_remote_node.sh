#!/bin/bash
export PATH=$PATH:/usr/local/cuda/bin;
export PYLEARN2_DATA_PATH=~/grasp_deep_learning/data/;
source ~/grasp_deep_learning/setup.bash;
source ~/grasp_deep_learning/gdl/devel/setup.bash;
export ROS_MASTER_URI=http://captiva.cs.columbia.edu:11311;
cd ~/grasp_deep_learning/gdl/src/pylearn_classifier_gdl;
rosrun pylearn_classifier_gdl ros_grasp_server.py;
