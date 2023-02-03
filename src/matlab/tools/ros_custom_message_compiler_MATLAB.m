%This file contains the necessary code to compile ROS custom defined
%messages to be used in MATLAB. If you use different machines to simulate,
%you could define different configuration adding IF clausules.

%Windows computer
if getenv('username') == "jesus" %Jesus' personal computer
    addpath('C:\Users\usuario\Documents\i3a_repos\siam_sim\src\siam_main');
    py = pyenv('Version', 'C:\Users\jesus\AppData\Local\Programs\Python\Python39\python.exe');
    rosgenmsg('C:\i3a_repos\siam_sim\src\');
    addpath('C:\i3a_repos\siam_sim\src\matlab_msg_gen_ros1\win64\install\m')

%Windows computer
elseif getenv('username') == "usuario" %I3A computer
    addpath('C:\Users\usuario\Documents\i3a_repos\siam_sim\src\gazebo-ros\src\siam_main');
    py = pyenv('Version', 'C:\Users\usuario\AppData\Local\Programs\Python\Python39\python.exe');
    rosgenmsg('C:\Users\usuario\Documents\i3a_repos\siam_sim\src\gazebo-ros\src\');
    addpath('C:\Users\usuario\Documents\i3a_repos\siam_sim\src\gazebo-ros\src\matlab_msg_gen_ros1\win64\install\m')

%Ubuntu computer
elseif getenv('USERNAME') == "siamsim" %Virtual Machine
    addpath('../gazebo-ros/src/siam_main/');
    py = pyenv('Version', '/usr/bin/python3.9');
    rosgenmsg('../gazebo-ros/src/');
    addpath('../../matlab_msg_gen_ros1\win64\install\m')

%Ubuntu computer
elseif getenv("USERNAME") == 'galadriel-ubuntu'
    setenv("MY_PYTHON_VENV", "/tmp/venv");
    ros.internal.createOrGetLocalPython(true);
    addpath('../gazebo-ros/src/siam_main/');
    py = pyenv('Version', '/usr/bin/python3.9');
    rosgenmsg('../gazebo-ros/src/');
    addpath('../../gazebo-ros/src/matlab_msg_gen_ros1\win64\install\m')
end

savepath
clear classes
rehash toolboxcache


