if getenv('username') == "jesus"
    addpath('C:\Users\usuario\Documents\i3a_repos\siam_sim\src\siam_main');
    py = pyenv('Version', 'C:\Users\jesus\AppData\Local\Programs\Python\Python39\python.exe');
    rosgenmsg('C:\i3a_repos\siam_sim\src\');
    addpath('C:\i3a_repos\siam_sim\src\matlab_msg_gen_ros1\win64\install\m')

elseif getenv('username') == "usuario"
    %Updated to repo refactor
    addpath('C:\Users\usuario\Documents\i3a_repos\siam_sim\src\gazebo-ros\src\siam_main');
    py = pyenv('Version', 'C:\Users\usuario\AppData\Local\Programs\Python\Python39\python.exe');
    rosgenmsg('C:\Users\usuario\Documents\i3a_repos\siam_sim\src\gazebo-ros\src\');
    addpath('C:\Users\usuario\Documents\i3a_repos\siam_sim\src\gazebo-ros\src\matlab_msg_gen_ros1\win64\install\m')

elseif getenv('USERNAME') == "siamsim"
    addpath('..');
    py = pyenv('Version', '/usr/bin/python3.8');
    rosgenmsg('../..');
    addpath('../../matlab_msg_gen_ros1\win64\install\m')
end

savepath
clear classes
rehash toolboxcache


