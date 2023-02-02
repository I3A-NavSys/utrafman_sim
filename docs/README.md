# SiAM Sim Documentation

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


## 2. Simulator environment
SiAM Simulator has been built using Robot Operating System (ROS) and Gazebo, as lot of simulator do. The contribution of ROS and Gazebo to the simulator will be discuted in followed sections. Our objective with SiAM is produce a widely-use simulator that help us to develop and test our UAV control software, UTM services and other software designet to support Urban Air Mobility (UAM). 

### 2.1. Robotic Operating System (ROS 1)
Robotic Operating System (ROS) is an open-source robotics middleware that provides tools and libraries for building complex robot applications. It enables quick development and integration of robotic components, such as sensors, actuators, and algorithms. ROS is designed to be a flexible and modular platform that can be used on a wide range of robots and provide a wide range of services. ROS is used in many robotics applications, such as autonomous vehicles, robotic manipulation, and service robots.

In SiAM Sim, ROS is used to provide communication between UAV and services/software implemented in other platforms. Topics and services are used to abstract communication systems from the development point-of-view. To do that, ROS use nodes, which are processes that communicate with each other through topics and services. Moreover, ROS is also used to launch the simulator through a launch file. 

#### Packages
Packages are the main way to organize ROS code. A package is a directory containing all the files you need for a ROS package to be built. Packages are grouped in workspaces, which are directories that contain ROS packages. SiAM Sim is organized in one workspace, which contains two following packages:
- **siam_main**: package that contains the main parts of the simulator. It contains the launch files, the world files, the UAV sdf models, the UAV control software, etc.
- **siam_god**: package that contains the God code and dependences. This contains the code for a Gazebo plugin that allows to spawn and remove UAVs from the environment. This plugin is present in each world file of the simulator.

#### Messages
Message sent through topics and services are defined with present or custom ROS messages. Some ROS messages that are used in SiAM Sim are (_package_/_message_)):
- **siam_main/FligjtPlan**: message that contains the flight plan of the UAV.
- **siam_main/Telemetry**: message that contains the telemetry of the UAV, like position, velocity, acceleration, etc.
- **siam_Waypoint**: message that contains the position of a 4D waypoint.

#### Topics
The topics implemented in SiAM Sim are (_namespace_/_topic_):
- **/god/insert**: topic that allows to insert a UAV in the environment. To insert a drone, it is necessary to send a message with the sdf model of the UAV to be inserted.
- **/god/remove**: topic that allows to remove a UAV from the environment. To remove a drone, it is necessary to send the drone ID to remove.
- **/drone/%droneid/uplan**: topic that allows to send a flight plan to a UAV. 
- **/drone/%droneid/telemetry**: topic that allows to receive the telemetry of a UAV.
- **/drone/%droneid/kill**: topic that allows to kill a UAV.

As you could know, this is an open-source project, so you can contribute to it. You can create new packages, modify messages definitions, add new topics, etc. 


### 2.2. Gazebo
Gazebo is a 3D dynamic simulator with the ability to accurately and efficiently simulate populations of robots in complex indoor and outdoor environments. Gazebo offers the entire physics and dynamics engine simulator, as well as the environment and collision simulator. Moreover, Gazebo allows the use of LiDARs and cameras onboard, where behaviour of these sensors could be implemented through Gazebo plugins.

Gazebo is used inside SiAM Sim to simulate the environment and the UAVs. The environment is composed of a world, where drones are placed. The world is composed of models, which are the objects that are placed in the world. For example, in world files exist a small city with roads, buildings, trees, etc. Gazebo is launched with ROS launch file, which is located in _siam_main_ package.

Gazebo simulations are running in a separated thread (_gzserver_) that simulation viewer (_gzclient_), so the simulator could be paused, resumed, stopped, etc. This is useful to debug the simulation, or if you want to save computational resources, as _gzclient_ is not necessary to run the simulation.

### 2.3. Extending Gazebo with plugins
Simulations in Gazebo could be extended with plugins. Plugins are libraries that are loaded in Gazebo and allow to extend the simulator with new functionalities. For example, in SiAM Sim, a plugin is used to spawn and remove UAVs from the environment. Drone autonomous control software is implemented as a plugin as well, so it is possible to implement new control software without modifying the simulator.


## 3. Drone Control Software
### 3.1. How is implemented?
## 4. UTM Services
## 5. Tutorials