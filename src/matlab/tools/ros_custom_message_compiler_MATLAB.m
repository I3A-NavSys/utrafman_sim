% This file contains the necessary code to compile ROS custom defined messages to be used in MATLAB. 

%Replace the commented lines to configure your MATLAB ROS compiler environment

clc; clear;

UTRAFMAN_init;

%Unix configuration
if isunix

    setenv("MY_PYTHON_VENV", "/tmp/venv");
    %ros.internal.createOrGetLocalPython(true);
    %py = pyenv('Version', '/usr/bin/python3.8');        %Set it with your python path (3.8 or higher)
    rosgenmsg(strcat(UTRAFMAN_DIR, 'gazebo-ros/src/'));
    addpath(  strcat(UTRAFMAN_DIR, 'gazebo-ros/src/matlab_msg_gen_ros1/glnxa64/install/m'));

%Windows configuration
elseif ispc

    %py = pyenv('Version', 'C:\Users\Rafael.Casado\AppData\Local\Programs\Python\Python311\python.exe');  %Set it with your python path (3.8 or higher)
    %ros.internal.createOrGetLocalPython(true);
    rosgenmsg(strcat(UTRAFMAN_DIR, 'gazebo-ros\src\'));
    addpath(  strcat(UTRAFMAN_DIR, 'gazebo-ros\src\matlab_msg_gen_ros1\win64\install\m\'));

end

savepath ./pathdef.m
clear classes
rehash toolboxcache


