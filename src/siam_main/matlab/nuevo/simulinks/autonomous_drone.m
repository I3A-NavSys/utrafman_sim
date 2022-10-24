if getenv('username') == "jesus"
    addpath('C:\Users\usuario\Documents\i3a_repos\siam_sim\src\siam_main');
    py = pyenv('Version', 'C:\Users\jesus\AppData\Local\Programs\Python\Python39\python.exe');
    rosgenmsg('C:\i3a_repos\siam_sim\src\');
    addpath('C:\i3a_repos\siam_sim\src\matlab_msg_gen_ros1\win64\install\m')
else
    addpath('C:\Users\usuario\Documents\i3a_repos\siam_sim\src\siam_main');
    py = pyenv('Version', 'C:\Users\usuario\AppData\Local\Programs\Python\Python39\python.exe');
    rosgenmsg('C:\Users\usuario\Documents\i3a_repos\siam_sim\src\');
    addpath('C:\Users\usuario\Documents\i3a_repos\siam_sim\src\matlab_msg_gen_ros1\win64\install\m')
end
savepath
clear classes
rehash toolboxcache


