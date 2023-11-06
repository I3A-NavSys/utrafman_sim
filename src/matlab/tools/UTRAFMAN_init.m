% This file allows you to configure ROS master IP address. 
% It is useful if you work with different machines.

% ROS master IP configuration
global ROS_IP
switch  getenv("USERNAME")
    case 'Rafa'
        ROS_IP = '192.168.17.128';
    otherwise
        %Default value
        ROS_IP = '127.0.0.1';
end


% MATLAB PATH configuration
global UTRAFMAN_DIR

if isunix  %Unix computer

    UTRAFMAN_DIR = '/opt/ros/noetic/share/utrafman_sim/src'; %Set it with your repo installation path
    addpath(strcat(UTRAFMAN_DIR, '/gazebo-ros/src/utrafman_main/'));
    addpath(strcat(UTRAFMAN_DIR, '/gazebo-ros/src/matlab_msg_gen_ros1/glnxa64/install/m'));
    addpath(strcat(UTRAFMAN_DIR, '/matlab/classes'));


elseif ispc %Windows computer

    switch getenv("USERNAME")
        case 'Rafa'
            UTRAFMAN_DIR = 'c:\Users\Rafa\OneDrive - Universidad de Castilla-La Mancha\NavSys\code\utrafman_sim\src';
        case 'Rafael.Casado'
            UTRAFMAN_DIR = 'c:\Users\Rafael.Casado\OneDrive - Universidad de Castilla-La Mancha\NavSys\code\utrafman_sim\src';
        otherwise
            error('Windows user not defined in file UTRAFMAN_init.m');
    end

    addpath(strcat(UTRAFMAN_DIR, '\gazebo-ros\src\utrafman_main')); 
    addpath(strcat(UTRAFMAN_DIR, '\gazebo-ros\src\matlab_msg_gen_ros1\win64\install\m'));
    addpath(strcat(UTRAFMAN_DIR, '\matlab\classes'));

end
