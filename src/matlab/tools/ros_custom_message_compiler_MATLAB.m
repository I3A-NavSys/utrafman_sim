%This file contains the necessary code to compile ROS custom defined
%messages to be used in MATLAB. If you use different machines to simulate,
%you could define different configuration adding IF clausules.

%Windows computer (Jesus' personal computer)
if getenv('username') == "jesus"
    addpath('C:\i3a_repos\utrafman_sim\src\gazebo-ros\src\utrafman_main');
    py = pyenv('Version', 'C:\Users\jesus\AppData\Local\Programs\Python\Python39\python.exe');
    rosgenmsg('C:\i3a_repos\utrafman_sim\src\gazebo-ros\src\');
    addpath('C:\i3a_repos\utrafman_sim\src\gazebo-ros\src\matlab_msg_gen_ros1\win64\install\m')

%Windows computer (I3A computer)
elseif getenv('username') == "usuario"
    addpath('C:\Users\usuario\Documents\i3a_repos\siam_sim\src\gazebo-ros\src\utrafman_main');
    py = pyenv('Version', 'C:\Users\usuario\AppData\Local\Programs\Python\Python39\python.exe');
    rosgenmsg('C:\Users\usuario\Documents\i3a_repos\siam_sim\src\gazebo-ros\src\');
    addpath('C:\Users\usuario\Documents\i3a_repos\siam_sim\src\gazebo-ros\src\matlab_msg_gen_ros1\win64\install\m')

%Ubuntu VM
elseif getenv('USERNAME') == "siamsim" %Virtual Machine
    addpath('../gazebo-ros/src/utrafman_main/');
    py = pyenv('Version', '/usr/bin/python3.9');
    rosgenmsg('../gazebo-ros/src/');
    addpath('../../matlab_msg_gen_ros1\win64\install\m')

%Ubuntu (galadriel computer)
elseif getenv("USERNAME") == 'galadriel-ubuntu'
    setenv("MY_PYTHON_VENV", "/tmp/venv");
    ros.internal.createOrGetLocalPython(true);
    addpath('../gazebo-ros/src/utrafman_main/');
    py = pyenv('Version', '/usr/bin/python3.9');
    rosgenmsg('../gazebo-ros/src/');
    addpath('../gazebo-ros/src/matlab_msg_gen_ros1\win64\install\m')
end

savepath
clear classes
rehash toolboxcache


