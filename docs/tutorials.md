# 5. Tutorials
This section contains tutorials to install and use SiAM Simulator.

## 5.1. Setup
As you discover in the previous section, SiAM Simulator uses ROS, Gazebo and MATLAB. ROS and Gazebo must be installed on the same computer, but MATLAB could work on a different computer. This is because, in very large simulations, the resources of typical machines may not be enough. By dividing the load into two independent machines, larger simulations can be performed.

If you use a two-computer setup, both of them must be connected to the same network and be able to communicate with each other. If you want to avoid ROS network problems, you can use the same computer for ROS, Gazebo and MATLAB. 

Either way, MATLAB must be configured with ROS_MASTER IP address, as explained in [this tutorial](https://es.mathworks.com/help/ros/ug/get-started-with-ros.html). If you use a two-computer setup, you must configure MATLAB with the IP address of the computer where ROS is running, in the file `/src/matlab/config/ros.m`. If you use a one-computer setup, you can simply use `localhost` as ROS_MASTER IP address.


## 5.2. Installation
### ROS and Gazebo
>:warning:  SiAM Simulator is only tested in **Ubuntu 20.04 and ROS Noetic**. Maybe it could work in other versions, but it is not guaranteed.

To install SiAM Simulator, you need to install first ROS and Gazebo. You can follow the official tutorials to install ROS and Gazebo [here](http://wiki.ros.org/noetic/Installation). Once you are  able to run a simulation in Gazebo and ROS, you can install SiAM Simulator. Remember to define the ROS environment variables, as explained in the ROS installation tutorial!

To install SiAM Simulator, you need to clone the repository in your workspace. To do that, you can use the following command:

```bash
cd /path-to-install-siam-simulator
git clone https://github.com/I3A-NavSys/siam_sim
```
As you could see, `src/gazebo_ros/` is a ROS (_catkin_) workspace and contains all the simulation environment. On the other hand, `src/matlab/` includes simulator launch code, tools and telemetry viewer. Once you have cloned the repository, you need to compile the code. To do that, you can use the following commands:

```bash
cd /siam-simulator/src/gazebo_ros
catkin_make
```
Once the workspace is compiled, you should be ready to run simulations.

### MATLAB
>:warning:  SiAM Simulator is only tested with **MATLAB R2022a and newer versions, and Python 3.9**. Maybe it could work in other versions, but it is not guaranteed.

To install MATLAB, you can follow the official tutorial [here](https://es.mathworks.com/help/install/ug/install-mathworks-software.html). Once you have installed MATLAB, you need to install the ROS Toolbox. To do that, you can follow the official tutorial [here](https://es.mathworks.com/help/ros/ug/install-ros-toolbox.html). When MATLAB and all the dependencies are installed, you only need to open the `/src/matlab/` folder in MATLAB. 

Finally, as custom ROS messages are used, you need to compile them. Use script file `/src/matlab/tools/ros-custom-message-compiler.m` to compile them. Edit the file to define where python is installed on your computer. Once you have compiled the messages, you are ready to run simulations. You could find more information about how to compile custom ROS messages [here](https://es.mathworks.com/help/ros/custom-message-support.html?s_tid=CRUX_lftnav). Once you have done all previous steps, you are ready to run simulations.

## 5.3. Running a test simulation
SiAM Sim comes with a test simulation to test if ROS, Gazebo and MATLAB are working properly. Open MATLAB with `/src/matlab/` as current directory and, in a new terminal, run the following command:
```bash
roslaunch siam_main test.launch
```
You should see how ROS is launched in the terminal, and Gazebo is launched in a new window. In the Gazebo window, you should see an empty world.

![Empty World](./img/tutorials/test-simulation-world-1.png 'Gazebo Viewer. Empty world.  :size=800px')

Now is the time to add UAVs to the world and send flight plans to them. To do that, you must run in MATLAB the following script: `/src/matlab/simulations/test_simulation.m`. This script will add five UAVs to the world, and send flight plans to them. A few seconds after, you should see how the UAV moves in the Gazebo window.

![UAVs](./img/tutorials/test-simulation-world-2.png 'Gazebo Viewer. Five UAV flying. :size=800px')

If you want to see the telemetry data, you can run ´/src/tools/telemetry-viewer.m´ script in MATLAB. This script will open a new window with the telemetry viewer.

![Telemetry Viewer](./img/tutorials/test-simulation-telemetry-viewer-1.jpg 'MATLAB Telemetry Viewer window :size=800px')