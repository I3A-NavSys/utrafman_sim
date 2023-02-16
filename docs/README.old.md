# U-TRAFMAN Sim Documentation

## 1. Introduction
The main objective of this simulator is allowing users to develop functionality and realistically simulate flight plans and navigations of Unmanned Aerial Vehicles (UAVs) in urban scenarios. Inside this main objective, there are some sub-objectives:
- **Develop and test UAV autonomous control software and services implementations:** control systems of the Unmanned Aerial Vehicles could be modified, reimplemented or amplified.
- **Develop and check UTM services:** services and frameworks could be developed and tested to validate if proposals are valid. Information between aircrafts and services can be exchanged through topics, stored and analysed after simulation to debug. Services could be implemented in whatever platform you want if the platform is compatible with ROS.
- **Provide realistic dynamics simulations of hundreds of UAV inside an urban area:** due to the physics engine of Gazebo simulator and its possibilities, UAV behaviour could be implemented, making the simulations close to real world behaviour.

### How the simulator is composed?
Nowadays, the simulator is composed of 3 main and essential components:
-	**ROS**: ROS Noetic could be the most critical part of the simulator. Provide communication between UAV and service/software implemented in other platforms. Topics and services are used to abstract communication systems from the development point-of-view. 
-	**Gazebo**: as is said before, Gazebo offers the entire physics and dynamics engine simulator, as well as the environment and collision simulator. Moreover, Gazebo allows the use of LiDARs and cameras onboard, where behaviour of these sensors could be implemented through Gazebo plugins.
-	**Services, analystics tools and gathering data**: a simulator launcher and telemetry viewer are provided in order to offer an example of how UAV are placed in the world, how to send a flight plan and a way to view afterward the route flied by the aircraft. These parts have been constructed using MATLAB.

### Previous knowledge
As you would read in the future section, the simulator is built using ROS and Gazebo. So, it is necessary to have some knowledge about these tools. Nowadays advanced concepts are not used, but know how ROS network works, use of packages, how to create a topic, how to create a service, etc **is a must**. If you are not familiar with ROS, you can read the [ROS wiki](http://wiki.ros.org/ROS/Tutorials) to learn the basics. If you are not familiar with Gazebo, you can read the [Gazebo tutorials](http://gazebosim.org/tutorials) to learn the basics.

## 2. Simulator environment
U-TRAFMAN Simulator has been built using Robot Operating System (ROS) and Gazebo, as lot of simulator do. The contribution of ROS and Gazebo to the simulator will be discuted in followed sections. Our objective with U-TRAFMAN is produce a widely-use simulator that help us to develop and test our UAV control software, UTM services and other software designet to support Urban Air Mobility (UAM). 

### 2.1. Robotic Operating System (ROS 1)
Robotic Operating System (ROS) is an open-source robotics middleware that provides tools and libraries for building complex robot applications. It enables quick development and integration of robotic components, such as sensors, actuators, and algorithms. ROS is designed to be a flexible and modular platform that can be used on a wide range of robots and provide a wide range of services. ROS is used in many robotics applications, such as autonomous vehicles, robotic manipulation, and service robots.

In SiAM Sim, ROS is used to provide communication between UAV and services/software implemented in other platforms. Topics and services are used to abstract communication systems from the development point-of-view. To do that, ROS use nodes, which are processes that communicate with each other through topics and services. Moreover, ROS is also used to launch the simulator through a launch file. 

#### Packages
Packages are the main way to organize ROS code. A package is a directory containing all the files you need for a ROS package to be built. Packages are grouped in workspaces, which are directories that contain ROS packages. U-TRAFMAN Sim is organized in one workspace, which contains two following packages:
- **siam_main**: package that contains the main parts of the simulator. It contains the launch files, the world files, the UAV sdf models, the UAV control software, etc.
- **siam_god**: package that contains the God code and dependences. This contains the code for a Gazebo plugin that allows to spawn and remove UAVs from the environment. This plugin is present in each world file of the simulator.

#### Messages
Message sent through topics and services are defined with present or custom ROS messages. Some ROS messages that are used in SiAM Sim are (_package_/_message_)):
- **siam_main/[Uplan](../src/gazebo-ros/src/siam_main/msg/Uplan.msg)**: message that contains the flight plan of the UAV.
- **siam_main/[Telemetry](../src/gazebo-ros/src/siam_main/msg/Telemetry.msg)**: message that contains the telemetry of the UAV, like position, velocity, acceleration, etc.
- **siam_main/[Waypoint](../src/gazebo-ros/src/siam_main/msg/Waypoint.msg)**: message that contains the position of a 4D waypoint.

#### Topics
The topics implemented in U-TRAFMAN Sim are (_namespace_/_topic_):
- **/god/insert**: topic that allows to insert a UAV in the environment. To insert a drone, it is necessary to send a message with the sdf model of the UAV to be inserted.
- **/god/remove**: topic that allows to remove a UAV from the environment. To remove a drone, it is necessary to send the drone ID to remove.
- **/drone/%droneid/uplan**: topic that allows to send a flight plan to a UAV. 
- **/drone/%droneid/telemetry**: topic that allows to receive the telemetry of a UAV.
- **/drone/%droneid/kill**: topic that allows to kill a UAV.

As you could know, this is an open-source project, so you can contribute to it. You can create new packages, modify messages definitions, add new topics, etc. 


### 2.2. Gazebo
Gazebo is a 3D dynamic simulator with the ability to accurately and efficiently simulate populations of robots in complex indoor and outdoor environments. Gazebo offers the entire physics and dynamics engine simulator, as well as the environment and collision simulator. Moreover, Gazebo allows the use of LiDARs and cameras onboard, where behaviour of these sensors could be implemented through Gazebo plugins.

Gazebo is used inside U-TRAFMAN Sim to simulate the environment and the UAVs. The environment is composed of a world, where drones are placed. The world is composed of models, which are the objects that are placed in the world. For example, in world files exist a small city with roads, buildings, trees, etc. Gazebo is launched with ROS launch file, which is located in _siam_main_ package.

Gazebo simulations are running in a separated thread (_gzserver_) that simulation viewer (_gzclient_), so the simulator could be paused, resumed, stopped, etc. This is useful to debug the simulation, or if you want to save computational resources, as _gzclient_ is not necessary to run the simulation.

Simulations in Gazebo could be extended with plugins. Plugins are libraries that are loaded in Gazebo and allow to extend the simulator with new functionalities. For example, in U-TRAFMAN Sim, a plugin is used to spawn and remove UAVs from the environment. Drone autonomous control software is implemented as a plugin as well, so it is possible to implement new control software without modifying the simulator.

### 2.3. MATLAB
> :warning: **MATLAB** is a registered trademark of The MathWorks, Inc. Moreover, is a commercial software, so you need to have a license to use it. However, you can use the free trial version of MATLAB for 30 days.

> :heavy_check_mark: As is said before, this simulator is flexible enougt to be used with other languages. You can use the MATLAB part done before, and implement your work on the simulator in the language you want.

MATLAB is a high-level language and interactive environment that enables you to perform computationally intensive tasks. MATLAB allows matrix manipulations, plotting of functions and data, implementation of algorithms, creation of user interfaces, and interfacing with programs written in other languages, including C, C++, C#, Java, Fortran and Python.

In U-TRAFMAN Simulator, MATLAB is used to define everithing that involves the simulation. This includes the simulation definition, UTM services, telemetry data processing, etc. The main reason to use MATLAB is that it is a widely-used language in the aerospace industry, so it is easy to find people that know how to use it. Moreover, it is easy to implement new functionalities in MATLAB, as it is a high-level language. And of course, with ROS Toolbox, it is easy to communicate with ROS network, node and more.

Nowadays, MATLAB is used to define the simulation, insert and remove UAVs from the environment, send flight plans to UAVs, receive telemetry data from UAVs, analyze telemetry data, etc. MATLAB part in the repository shows the following repository structure:
- **classes**: folder that contains definitios and methods of the classes. Documentation is included in the code. If you are using MATLAB, you can use the `doc` command to see the documentation of the class.
- **config**: folder that contains configuration files.
    - **ros.m**: file to configure ROS network in MATLAB.
- **simulations**: folder that contains simulation definitions files.
    - **test_simulation.m**: file that defines a test simulation.
- **tools**: folder that contains tools, like telemetry viewer or error.
    - **telemetry_viewer.m**: Telemetry data tool. Telemetry viewer was built to check if the autonomous control software is working correctly, and how precise is compared to the flight plan. The telemetry viewer window is divided in two parts. In the left part, you can see the flight plan route and the UAV route, and in the right part, you can see the telemetry data with the position and velocity the UAV compared to the flight plan. You can change the UAV under analysis in the line 15, `drone = 1`.
    - **error.m**: Error computation tool. THis tool was built to compute the error between the UAV position and the flight plan position through time. Min, mean and max error are computed. Error is computed for all the UAVs in the simulation.


## 3. Drone Control Software
The UAV control plugin takes UAV positions / velocities / accelerations from Gazebo using the C++ API, computes the difference between UAV position and reference from the FlightPlan, computes the control action and computes the forces and moments to apply to the UAV in Gazebo. Moreover, this plugin also send telemetry data throgh topics with the frecuency selected. You could found the code of the plugin in _siam\_main_ package, in _src_ folder. The plugin has been implemented the control present [in this paper](https://journals.sagepub.com/doi/10.1177/1729881418820425).

## 4. UTM Services
Unmanned Traffic Management (UTM) is an emerging technology that will be used to manage the increasing number of unmanned aerial vehicles (UAVs) in the airspace. UTM will enable the safe, efficient, and secure integration of UAVs into the existing air traffic management system. The UTM system will facilitate the safe exchange of information between UAVs, pilots, and other airspace users to ensure that UAVs operate in a safe and secure manner. To archieve that, UTM will offer something called "_services_", that are information exchange systems, provided by different sercices providers, such as goverment, private companies, etc. 

Some of these services are:
- Auspace authorization.
- Airspace management.
- Flight coordination.
- Real-time traffic information.
- Weather information.
- Conflict detection and resolution.
- etc.

As UTM technology advances, the services it offers will become more reliable, secure, and efficient, enabling the safe integration of UAVs into the airspace. But nowaday, UTM is still in its infancy, and there are not many services available. Services in development must be tested, and this is where U-TRAFMAN Simulator comes in. U-TRAFMAN Simulator is a flexible simulator that allows to develop and test UTM services. These services could be implemented in MATLAB, as it is a widely-used language in the aerospace industry and mainly language in this project, but **you can use any other language if you want**.

In this 1.0 version, U-TRAFMAN does not offer any UTM service. It just offer a way to implement them. However, in the next versions, some UTM services will be implemented. 

---

# 5. Tutorials
This section contains tutorials to install and use U-TRAFMAN Simulator.

## 5.1. Setup
As you discover in the previous section, U-TRAFMAN Simulator use ROS, Gazebo and MATLAB. ROS and Gazebo must be installed in the same computed, but MATLAB could workd in a different computer. This is because, in very large simulations, the resources of typical machines may not be enough. By dividing the load into two independent machines, larger simulations can be performed.

If you use two-computer setup, both of then must be connected to the same network and be able to communicate with each other. If you want to avoid ROS network problems, you can use the same computer for ROS, Gazebo and MATLAB. 

Either way, MATLAB must be configured with ROS_MASTER IP address, as explained in [this tutorial](https://es.mathworks.com/help/ros/ug/get-started-with-ros.html). If you use two-computer setup, you must configure MATLAB with the IP address of the computer where ROS is running, in the file `/src/matlab/config/ros.m`. If you use one-computer setup, you can simply use _localhost_ as ROS_MASTER IP address.


## 5.2. Installation
### ROS and Gazebo
>:warning:  U-TRAFMAN Simulator is only tested in **Ubuntu 20.04 and ROS Noetic**. Maybe it could work in other versions, but it is not guaranteed.

To install U-TRAFMAN Simulator, you need to install first ROS and Gazebo. You can follow the official tutorials to install ROS and Gazebo [here](http://wiki.ros.org/noetic/Installation). Once you are be able to run a simulation in Gazebo and ROS, you can install U-TRAFMAN Simulator. Remmember to define the ROS environment variables, as explained in the ROS installation tutorial!

To install U-TRAFMAN Simulator, you need to clone the repository in your workspace. To do that, you can use the following command:

```shell
cd /path-to-install-siam-simulator
git clone https://github.com/I3A-NavSys/siam_sim
```
As you could see, `src/gazebo_ros/` is a ROS (_catkin_) workspace, and contains all the simulation environment. In the other hand, `src/matlab/` includes simulator launch code, tools and telemetry viewer. Once you have cloned the repository, you need to compile the code. To do that, you can use the following commands:

```shell
cd /siam-simulator/src/gazebo_ros
catkin_make
```
Once the workspace is compiled, you should be ready to run simulations.

### MATLAB
>:warning:  U-TRAFMAN Simulator is only tested with **MATLAB R2022a and newer versions, and Python 3.9**. Maybe it could work in other versions, but it is not guaranteed.

To install MATLAB, you can follow the official tutorial [here](https://es.mathworks.com/help/install/ug/install-mathworks-software.html). Once you have installed MATLAB, you need to install the ROS Toolbox. To do that, you can follow the official tutorial [here](https://es.mathworks.com/help/ros/ug/install-ros-toolbox.html). When MATLAB and all the dependencies are installed, you only need to open the `/src/matlab/` folder in MATLAB. 

Finally, as custom ROS messages are used, you need to compile them. Use script file `/src/matlab/tools/ros-custom-message-compiler.m` to compile them. Edit the file to define where python is installed in your computer. Once you have compiled the messages, you are ready to run simulations. You could find more information about how to compile custom ROS messages [here](https://es.mathworks.com/help/ros/custom-message-support.html?s_tid=CRUX_lftnav). Once you have done all previous steps, you are ready to run simulations.

## 5.3. Running a test simulation
U-TRAFMAN Sim comes with a test simulation to test if ROS, Gazebo and MATLAB are working properly. Open MATLAB with `/src/matlab/` as current directory and, in a new terminal, run the following command:
```shell
roslaunch siam_main test.launch
```
You should see how ROS is launched in the terminal, and Gazebo is launched in a new window. In the Gazebo window, you should see empty world.

![Empty World](./img/tutorials/test-simulation-world-1.png ':size=800px')

Now is time to add UAVs to the world and send flight plans to them. To do that, you must run in MATLAB the following script: `/src/matlab/simulations/test_simulation.m`. This script will add five UAVs to the world, and send a flight plan to it. Few seconds after, you should see how the UAV moves in the Gazebo window.

![UAVs](./img/tutorials/test-simulation-world-2.png ':size=800px')

If you want to see the telemetry data, you can run ´/src/tools/telemetry-viewer.m´ script in MATLAB. This script will open a new window with the telemetry viewer.

![Telemetry Viewer](./img/tutorials/test-simulation-telemetry-viewer-1.jpg ':size=800px')