# 5. Tutorials
This section contains tutorials to install and use U-TRAFMAN Simulator.

## 5.1. Setup
As you discover in the previous section, U-TRAFMAN Simulator uses ROS, Gazebo and MATLAB. ROS and Gazebo must be installed on the same computer, but MATLAB could work on a different computer. This is because, in very large simulations, the resources of typical machines may not be enough. By dividing the load into two independent machines, larger simulations can be performed.

If you use a two-computer setup, both of them must be connected to the same network and be able to communicate with each other. If you want to avoid ROS network problems, you can use the same computer for ROS, Gazebo and MATLAB. 

Either way, MATLAB must be configured with ROS_MASTER IP address, as explained in [this tutorial](https://es.mathworks.com/help/ros/ug/get-started-with-ros.html). If you use a two-computer setup, you must configure MATLAB with the IP address of the computer where ROS is running, in the file `/src/matlab/config/ros.m`. If you use a one-computer setup, you can simply use `localhost` as ROS_MASTER IP address.


## 5.2. Installation
### ROS and Gazebo
>:warning:  U-TRAFMAN Simulator is only tested in **Ubuntu 20.04 and ROS Noetic**. Maybe it could work in other versions, but it is not guaranteed.

To install U-TRAFMAN Simulator, you need to install first ROS and Gazebo. You can follow the official tutorials to install ROS and Gazebo [here](http://wiki.ros.org/noetic/Installation). Once you are  able to run a simulation in Gazebo and ROS, you can install U-TRAFMAN Simulator. Remember to define the ROS environment variables, as explained in the ROS installation tutorial!

To install U-TRAFMAN Simulator, you need to clone the repository in your ROS installation. To do that, you can use the following command:

```bash
cd /opt/ros/noetic/share
git clone https://github.com/I3A-NavSys/utrafman_sim
```
You could install the simulator in other locations, but additional steps are needed. As you could see, `src/gazebo-ros/` is a ROS (_catkin_) workspace and contains all the simulation environment. On the other hand, `src/matlab/` includes simulator launch code, tools and telemetry viewer. Once you have cloned the repository, you need to compile the code. To do that, you can use the following commands:

```bash
cd /utrafman-sim/src/gazebo-ros #(or just cd utrafman_sim)
catkin_make
```
>:warning:  _catkin\_make_ will not work if you have not sourced ROS `/opt/ros/noetic/setub.bash` file. Add the source in your `.bashrc` or source it in the terminal before compiling the workspace!

Once the workspace is compiled, you also need to source the workspace into your `.bashrc` file. To do that, include the following line in your `.bashrc` file:

```bash
source /opt/ros/noetic/share/utrafman_sim/devel/setup.bash
```

Now you should be ready to run simulations.

### MATLAB
>:warning:  U-TRAFMAN Simulator is only tested with **MATLAB R2022a and newer versions, and Python 3.9**. Maybe it could work in other versions, but it is not guaranteed.

To install MATLAB, you can follow the official tutorial [here](https://es.mathworks.com/help/install/ug/install-mathworks-software.html). Once you have installed MATLAB, you need to install the ROS Toolbox. To do that, you can follow the official tutorial [here](https://es.mathworks.com/help/ros/ug/install-ros-toolbox.html). When MATLAB and all the dependencies are installed, you only need to open the `/src/matlab/` folder in MATLAB. 

Finally, as custom ROS messages are used, you need to compile them. Use script file `/src/matlab/tools/ros-custom-message-compiler.m` to compile them. Edit the file to define where python is installed on your computer. Once you have compiled the messages, you are ready to run simulations. You could find more information about how to compile custom ROS messages [here](https://es.mathworks.com/help/ros/custom-message-support.html?s_tid=CRUX_lftnav). Once you have done all previous steps, you are ready to run simulations.

## 5.3. Running a simple simulation
U-TRAFMAN Sim comes with a simple simulation to test if ROS, Gazebo and MATLAB are working properly. Open MATLAB with `/src/matlab/` as current directory and, in a new terminal, run the following command:
```bash
roslaunch utrafman_main generated_city.launch
```
You should see how ROS is launched in the terminal, and Gazebo is launched in a new window. In the Gazebo window, you should see a world with simples buildings placed on it.

![Empty World](./img/tutorials/simple-simulation-1.png 'Gazebo Viewer. Simple generated city world.  :size=800px')

Now is the time to add some UAVs to the world and send flight plans to them. To do that, you must run in MATLAB the following script: `/src/matlab/simulations/simple_simulation.m`. This script will connect with ROS master from MATLAB, load the world definition file in MATLAB, create a single operator, create 10 UAVs and register both using the Registry service. Using the world definition file, the script will generate a flight plan for each UAV with a random route of 500 meters. Finally, the script will send the flight plans to the UAVs. 

![UAVs](./img/tutorials/simple-simulation-2.png 'Gazebo Viewer. Ten UAVs flying in the world. :size=800px')

![UAVs](./img/tutorials/simple-simulation-3.png 'Gazebo Viewer. Ten UAVs flying in the world. :size=800px')

Every time a random route is generated for a UAV, the script will print the route in a 3D figure showing the route and the buildings in the world.

![Routes generated](./img/tutorials/simple-simulation-random-routes.png 'Random routes generated :size=800px')

Once the simulation has finised, an _SimulationProcesser_ object which variable name is `SP` will be available in the MATLAB workspace, allowing you to get and analyze the data. For example, if you want to visualize the telemetry data of a flight plan, you can run the following command in MATLAB:
```matlab
    SP.telemetryViewer(fp_id);
```

![Telemetry Viewer](./img/tutorials/simple-simulation-telemetry-viewer.png 'MATLAB Telemetry Viewer :size=800px')