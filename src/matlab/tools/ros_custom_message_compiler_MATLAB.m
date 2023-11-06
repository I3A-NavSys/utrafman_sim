% This file contains the necessary code to compile ROS custom defined messages to be used in MATLAB. 

%Replace the commented lines to configure your MATLAB ROS compiler environment

clc; clear;

%Unix configuration
if isunix
    %Set it with your repo installation path
    repo_path = '/opt/ros/noetic/share/utrafman_sim'; 

    setenv("MY_PYTHON_VENV", "/tmp/venv");
    ros.internal.createOrGetLocalPython(true);
    py = pyenv('Version', '/usr/bin/python3.8');        %Set it with your python path (3.8 or higher)
    addpath(strcat(repo_path, '/src/gazebo-ros/src/utrafman_main/'));
    rosgenmsg(strcat(repo_path, '/src/gazebo-ros/src/'));
    addpath(strcat(repo_path, '/src/gazebo-ros/src/matlab_msg_gen_ros1/glnxa64/install/m'));

%Windows configuration
elseif ispc
    %Set it with your repo installation path
    repo_path = strcat(['c:\Users\',...
                       getenv("USERNAME"),...
                       '\OneDrive - Universidad de Castilla-La Mancha\NavSys\code\utrafman_sim']);



    addpath(strcat(repo_path, '\src\gazebo-ros\src\utrafman_main')); 
    %py = pyenv('Version', 'C:\Users\Rafael.Casado\AppData\Local\Programs\Python\Python311\python.exe');  %Set it with your python path (3.8 or higher)
    rosgenmsg(strcat(repo_path, '\src\gazebo-ros\src\'));
    addpath(strcat(repo_path, '\src\gazebo-ros\src\matlab_msg_gen_ros1\win64\install\m'));

end

savepath ./pathdef.m
clear classes
rehash toolboxcache


