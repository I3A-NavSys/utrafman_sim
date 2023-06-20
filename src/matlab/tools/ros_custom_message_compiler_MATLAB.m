%This file contains the necessary code to compile ROS custom defined
%messages to be used in MATLAB. If you use different machines to simulate with different configurations,
%you could define different configuration adding IF clausules.

%Replace the commented lines to configure your MATLAB ROS compiler environment

%Dependendes: 
% - python3.X-venv (apt install python3.x-venv)
% - ROS Toolbox for MATLAB by MathWorks

%Unix configuration
if isunix
    repo_path = '/opt/ros/noetic/share/siam_sim'; %Set it with your repo installation path

    setenv("MY_PYTHON_VENV", "/tmp/venv");
    ros.internal.createOrGetLocalPython(true);
    py = pyenv('Version', '/usr/bin/python3.9');        %Set it with your python path (3.8 or higher)
    addpath(strcat(repo_path, '/src/gazebo-ros/src/utrafman_main/'));
    rosgenmsg(strcat(repo_path, '/src/gazebo-ros/src/'));
    addpath(strcat(repo_path, '/src/gazebo-ros/src/matlab_msg_gen_ros1/glnxa64/install/m'));

%Windows configuration
else ispc
    repo_path = 'C:\i3a_repos\utrafman_sim';             %Set it with your repo installation path

    addpath(strcat(repo_path, '\src\gazebo-ros\src\utrafman_main')); 
    py = pyenv('Version', 'C:\Users\jesus\AppData\Local\Programs\Python\Python39\python.exe');  %Set it with your python path (3.8 or higher)
    rosgenmsg(strcat(repo_path, '\src\gazebo-ros\src\'));
    addpath(strcat(repo_path, 'src\gazebo-ros\src\matlab_msg_gen_ros1\win64\install\m'));
end

savepath
clear classes
rehash toolboxcache


