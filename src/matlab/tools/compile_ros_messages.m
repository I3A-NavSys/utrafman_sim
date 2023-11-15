% This file contains the necessary code to compile ROS custom defined messages to be used in MATLAB. 

clc; clear;

UTRAFMAN_init;

rosgenmsg(fullfile(UTRAFMAN_DIR,'gazebo-ros/src/'));

addpath(fullfile(UTRAFMAN_DIR,'gazebo-ros/src/matlab_msg_gen_ros1/glnxa64/install/m'));

clear classes

rehash toolboxcache
