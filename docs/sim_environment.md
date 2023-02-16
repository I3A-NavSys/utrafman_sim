# 2. Simulator environment <!-- {docsify-ignore} -->
U-TRAFMAN Simulator has been built using Robot Operating System (ROS) and Gazebo, as a lot of simulators do. The contribution of ROS and Gazebo to the simulator will be discussed in followed sections. Our objective with U-TRAFMAN is to produce a widely-used simulator that helps us to develop and test our UAV control software, UTM services and other software designed to support Urban Air Mobility (UAM). 

## 2.1. Robotic Operating System (ROS 1)
Robotic Operating System (ROS) is an open-source robotics middleware that provides tools and libraries for building complex robot applications. It enables quick development and integration of robotic components, such as sensors, actuators, and algorithms. ROS is designed to be a flexible and modular platform that can be used on a wide range of robots and provide a wide range of services. ROS is used in many robotics applications, such as autonomous vehicles, robotic manipulation, and service robots.

In U-TRAFMAN Sim, ROS is used to provide communication between UAV and services/software implemented in other platforms. Topics and services are used to abstract communication systems from the development point of view. To do that, ROS uses nodes, which are processes that communicate with each other through topics and services. Moreover, ROS is also used to launch the simulator through a launch file. 

### Packages
Packages are the main way to organize ROS code. A package is a directory containing all the files you need for a ROS package to be built. Packages are grouped in workspaces, which are directories that contain ROS packages. U-TRAFMAN Sim is organized in one workspace, which contains two following packages:
- **utrafman_main**: a package that contains the main parts of the simulator. It contains the launch files, the world files, the UAV sdf models, the UAV control software, etc.
- **utrafman_god**: a package that contains the God code and dependencies. This contains the code for a Gazebo plugin that allows to spawn and remove UAVs from the environment. This plugin is present in each world file of the simulator.

### Messages
Messages sent through topics and services are defined with present or custom ROS messages. Some ROS messages that are used in U-TRAFMAN Sim are (_package_/_message_)):
- **utrafman_main/[Uplan](https://github.com/I3A-NavSys/utrafman_sim/tree/main/src/gazebo-ros/src/utrafman_main/msg/Uplan.msg)**: a message that contains the flight plan of the UAV.
- **utrafman_main/[Telemetry](https://github.com/I3A-NavSys/utrafman_sim/tree/main/src/gazebo-ros/src/utrafman_main/msg/Telemetry.msg)**: a message that contains the telemetry of the UAV, like position, velocity, acceleration, etc.
- **utrafman_main/[Waypoint](https://github.com/I3A-NavSys/utrafman_sim/tree/main/src/gazebo-ros/src/utrafman_main/msg/Waypoint.msg)**: a message that contains the position of a 4D waypoint.

### Topics
The topics implemented in U-TRAFMAN Sim are (_namespace_/_topic_):
- **/god/insert**: a topic that allows inserting a UAV in the environment. To insert a drone, it is necessary to send a message with the sdf model of the UAV to be inserted.
- **/god/remove**: a topic that allows to remove UAVs from the environment. To remove a drone, it is necessary to send the drone ID to remove.
- **/drone/%droneid/uplan**: a topic that allows sending a flight plan to a UAV. 
- **/drone/%droneid/telemetry**: a topic that allows receiving the telemetry of a UAV.
- **/drone/%droneid/kill**: a topic that allows killing a UAV.

As you could know, this is an open-source project, so you can contribute to it. You can create new packages, modify message definitions, add new topics, etc. 


## 2.2. Gazebo
Gazebo is a 3D dynamic simulator with the ability to accurately and efficiently simulate populations of robots in complex indoor and outdoor environments. Gazebo offers the entire physics and dynamics engine simulator, as well as the environment and collision simulator. Moreover, Gazebo allows the use of LiDARs and cameras onboard, where the behaviour of these sensors could be implemented through Gazebo plugins.

Gazebo is used inside U-TRAFMAN Sim to simulate the environment and the UAVs. The environment is composed of a world, where drones are placed. The world is composed of models, which are the objects that are placed in the world. For example, in world files exist a small city with roads, buildings, trees, etc. Gazebo is launched with ROS launch file, which is located in `utrafman_main` package.

Gazebo simulations are running in a separate thread (_gzserver_) that simulation viewer (_gzclient_), so the simulator could be paused, resumed, stopped, etc. This is useful to debug the simulation, or if you want to save computational resources, as _gzclient_ is not necessary to run the simulation.

Simulations in Gazebo could be extended with plugins. Plugins are libraries that are loaded in Gazebo and allow to extend the simulator with new functionalities. For example, in U-TRAFMAN Sim, a plugin is used to spawn and remove UAVs from the environment. Drone autonomous control software is implemented as a plugin as well, so it is possible to implement new control software without modifying the simulator.

## 2.3. MATLAB
> :warning: **MATLAB** is a registered trademark of The MathWorks, Inc. Moreover, is a commercial software, so you need to have a license to use it. However, you can use the free trial version of MATLAB for 30 days.

> :heavy_check_mark: As said before, this simulator is flexible enough to be used with other languages. You can use the MATLAB part done before, and implement your work on the simulator in the language you want.

MATLAB is a high-level language and interactive environment that enables you to perform computationally intensive tasks. MATLAB allows matrix manipulations, plotting of functions and data, implementation of algorithms, creation of user interfaces, and interfacing with programs written in other languages, including C, C++, C#, Java, Fortran and Python.

In U-TRAFMAN Simulator, MATLAB is used to define everything that involves the simulation. This includes the simulation definition, UTM services, telemetry data processing, etc. The main reason to use MATLAB is that it is a widely-used language in the aerospace industry, so it is easy to find people that know how to use it. Moreover, it is easy to implement new functionalities in MATLAB, as it is a high-level language. And of course, with ROS Toolbox, it is easy to communicate with ROS network, node and more.

Nowadays, MATLAB is used to define the simulation, insert and remove UAVs from the environment, send flight plans to UAVs, receive telemetry data from UAVs, analyze telemetry data, etc. MATLAB part in the repository shows the following repository structure:
- **classes**: a folder that contains definitions and methods of the classes. Documentation is included in the code. If you are using MATLAB, you can use the `doc` command to see the documentation of the class.
- **config**: a folder that contains configuration files.
    - **ros.m**: file to configure ROS network in MATLAB.
- **simulations**: a folder that contains simulation definitions files.
    - **test_simulation.m**: file that defines a test simulation.
- **tools**: a folder that contains tools, like telemetry viewer or error.
    - **telemetry_viewer.m**: Telemetry data tool. Telemetry viewer was built to check if the autonomous control software is working correctly, and how precise is compared to the flight plan. The telemetry viewer window is divided into two parts. In the left part, you can see the flight plan route and the UAV route, and in the right part, you can see the telemetry data with the position and velocity of the UAV compared to the flight plan. You can change the UAV under analysis in line 15, `drone = 1`.
    - **error.m**: Error computation tool. This tool was built to compute the error between the UAV position and the flight plan position through time. Min, mean and max errors are computed. Error is computed for all the UAVs in the simulation.