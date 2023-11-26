% This file allows you to configure ROS master IP address. 
% It is useful if you work with different machines.

% ROS master IP configuration
% %global ROS_MASTER_IP
% switch  getenv("USERNAME")
%     case 'Rafael.Casado'
%         ROS_MASTER_IP = '192.168.225.128';
%     case 'Rafa'
%         ROS_MASTER_IP = '192.168.17.128';
%     otherwise
%         %Default value
%         ROS_MASTER_IP = '127.0.0.1';
% end


% MATLAB PATH configuration
% global UTRAFMAN_DIR

if isunix  %Unix computer

    UTRAFMAN_DIR = '/opt/ros/humble/share/utrafman/src/'; %Set it with your repo installation path
    addpath(fullfile(UTRAFMAN_DIR,'gazebo-ros/src/matlab_msg_gen_ros1/glnxa64/install/m/'));

elseif ispc %Windows computer

    switch getenv("USERNAME")
        case 'Rafael.Casado'
            UTRAFMAN_DIR = 'c:\Users\Rafael.Casado\OneDrive - Universidad de Castilla-La Mancha\NavSys\code\utrafman_sim\src\';
        case 'Rafa'
            UTRAFMAN_DIR = 'c:\Users\Rafa\OneDrive - Universidad de Castilla-La Mancha\NavSys\code\utrafman_sim\src\';
        otherwise
            error('Windows user not defined in file UTRAFMAN_init.m');
    end
    addpath(fullfile(UTRAFMAN_DIR,'gazebo-ros\src\matlab_msg_gen_ros1\win64\install\m\'));

end

addpath(genpath(fullfile(UTRAFMAN_DIR,'matlab/')));
