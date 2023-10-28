# Installation


## Set up your machine

You can install U-TRAFMAN on either a computer or a virtual machine. We have successfully tested it with VMware Workstation Player 17. You can obtain it [here](https://www.vmware.com/es/products/workstation-player/workstation-player-evaluation.html).

Configure a computer with **Ubuntu 20.04.6 LTS (Focal Fossa)**. You can obtain a Desktop Image [here](https://releases.ubuntu.com/focal).

>:warning: U-TRAFMAN has been developed and tested in the specified Ubuntu version. While it might work in other versions, there are no guarantees of compatibility.


## ROS and Gazebo

U-TRAFMAN runs on **ROS** (Robot Operating System) **Noetic**. 
You can follow the official tutorials to install it [here](https://wiki.ros.org/noetic/Installation/Ubuntu).
Don't forget to perform a _full desktop_ installation and install the necessary dependencies for building packages.

As a part of _ROS Noetic_, the _Gazebo 11_ simulator will be installed. To check it, execute the command `gazebo` in a terminal. The graphical interface of Gazebo should open:

![Gazebo](./img/gazebo.png 'Gazebo simulator. :size=400px')

>:warning: U-TRAFMAN has been developed and tested in the specified ROS/Gazebo versions. While it might work in other versions, there are no guarantees of compatibility.


## U-TRAFMAN package

Once your ROS/Gazebo installation is complete, you can install U-TRAFMAN as a ROS package.
You need to clone the repository in your ROS installation:
```bash
cd /opt/ros/noetic/share
sudo git clone https://github.com/I3A-NavSys/utrafman_sim
```

After cloning, change the permissions of the `utrafman_sim` folder to your user executing the following commands (replace `username` with your actual _username_):
```bash
sudo chown -R username utrafman_sim
sudo chgrp -R username utrafman_sim
```

A ROS (Catkin) workspace containing the simulation environment is located in `src/gazebo-ros/`. 
You need to compile it with the following commands:
```bash
cd utrafman-sim/src/gazebo-ros
catkin_make
```
>:warning: Make sure that the `/opt/ros/noetic/setup.bash` file was sourced in your `.bashrc` file as instructed in the ROS installation step above. Failure to do so may cause the `catkin_make` command to not work properly.

You also need to source the compiled workspace into your `.bashrc` file for it to be accessible:
```bash
echo "source /opt/ros/noetic/share/utrafman_sim/devel/setup.bash" >> ~/.bashrc
source ~/.bashrc
```


## MATLAB

U-TRAFMAN provides several tools for launching and analyzing simulations. 
To manage these tools, you must install MATLAB by following the official tutorial [here](https://es.mathworks.com/help/install/ug/install-mathworks-software.html). 
Please ensure that you install at least the following components:
- MATLAB R2023a
- ROS Toolbox
- Parallel Computing Toolbox

>:warning:  U-TRAFMAN simulator has been tested in **MATLAB R2023a**. While it might work in other versions, there are no guarantees of compatibility.

>:warning:  The ideal option is to install ROS/Gazebo and MATLAB on the same computer. However, in the case of very large simulations where the resources of typical machines may not be sufficient, it could be advantageous to run them on different computers. In such cases, ensure that the computers are connected to the same network and can communicate with each other.


----------
### PlotCube
PLOTCUBE for MATLAB by Olivier (installed after MATLAB installation)
>:warning: ¿desde donde se instala?
>:warning: ¿donde se instala?


### Python
Python 3.8-venv (installed through the terminal using `apt install python3.x-venv`).
>:warning:  U-TRAFMAN simulator has been tested in **Python 3.8**. While it might work in other versions, there are no guarantees of compatibility.
---------


### Compiling ROS messages using MATLAB
Custom ROS messages are used by U-TRAFMAN. You need to compile them in order to be used in MATLAB. Use script file `/src/matlab/tools/ros-custom-message-compiler.m` to compile them. Edit the file to define where the working directory (repo) and python is installed on your computer and run the script. If everything is correct, you should see a message in the MATLAB console saying `Build succeeded`.

You could find more information about how to compile custom ROS messages [here](https://es.mathworks.com/help/ros/custom-message-support.html?s_tid=CRUX_lftnav). 




#### Setting up ROS_MASTER IP address in MATLAB
MATLAB must be configured with ROS_MASTER IP address, as explained in this [tutorial](https://es.mathworks.com/help/ros/ug/get-started-with-ros.html).
- If you use a one-computer setup, you can simply use `localhost` as ROS_MASTER IP address.
- If you use a two-computer setup, you must configure MATLAB with the IP address of the computer where ROS is running, in the file `/src/matlab/config/ros.m`. 

-------------
Once you have done all previous steps, you are ready to run simulations.

