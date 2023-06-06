# 2. Simulator environment <!-- {docsify-ignore} -->
U-TRAFMAN Simulator has been built using Robot Operating System (ROS) and Gazebo, as a lot of simulators do. The contribution of ROS and Gazebo to the simulator will be discussed in followed sections. Our objective with U-TRAFMAN is to produce a widely-used simulator that helps us to develop and test our UAV control software, UTM services and other software designed to support Urban Air Mobility (UAM). 

## 2.1. Robotic Operating System (ROS 1)
Robotic Operating System (ROS) is an open-source robotics middleware that provides tools and libraries for building complex robot applications. It enables quick development and integration of robotic components, such as sensors, actuators, and algorithms. ROS is designed to be a flexible and modular platform that can be used on a wide range of robots and provide a wide range of services. ROS is used in many robotics applications, such as autonomous vehicles, robotic manipulation, and service robots.

In U-TRAFMAN Sim, ROS is used to provide communication between UAV and services/software implemented in other platforms. Topics and services are used to abstract communication systems from the development point of view. To do that, ROS uses nodes, which are processes that communicate with each other through topics and services. Moreover, ROS is also used to launch the simulator through a launch file. 

### Packages
Packages are the main way to organize ROS code. A package is a directory containing all the files you need for a ROS package to be built. Packages are grouped in workspaces, which are directories that contain ROS packages. U-TRAFMAN Sim is organized in one workspace, which contains two following packages:
- **utrafman_main**: a package that contains the main parts of the simulator. It contains the launch files, the world files, the UAV sdf models, the UAV control software, etc.
- **utrafman_god**: a package that contains the God code and dependencies. This contains the code for a Gazebo plugin that allows to spawn and remove UAVs from the environment. This plugin is present in each world file of the simulator.

### Custom messages
Messages sent through topics and services are defined with present or custom ROS messages. Some ROS messages that are used in U-TRAFMAN Sim are (_package_/_message_):
- **utrafman_main/[Operator.msg](https://github.com/I3A-NavSys/utrafman_sim/tree/main/src/gazebo-ros/src/utrafman_main/msg/Operator.msg)**: a message that contains the information of the operator.
- **utrafman_main/[Telemetry.msg](https://github.com/I3A-NavSys/utrafman_sim/tree/main/src/gazebo-ros/src/utrafman_main/msg/Telemetry.msg)**: a message that contains the telemetry of the UAV, like position, velocity, acceleration, etc.
- **utrafman_main/[UAV.msg](https://github.com/I3A-NavSys/utrafman_sim/tree/main/src/gazebo-ros/src/utrafman_main/msg/UAV.msg)**: a message that contains the information of the UAV.
- **utrafman_main/[Uplan.msg](https://github.com/I3A-NavSys/utrafman_sim/tree/main/src/gazebo-ros/src/utrafman_main/msg/Uplan.msg)**: a message that contains the flight plan of the UAV.
- **utrafman_main/[Waypoint.msg](https://github.com/I3A-NavSys/utrafman_sim/tree/main/src/gazebo-ros/src/utrafman_main/msg/Waypoint.msg)**: a message that contains the position of a 4D waypoint.

### Custom services
- **utrafman_main/[insert_model.srv](https://github.com/I3A-NavSys/utrafman_sim/tree/main/src/gazebo-ros/src/utrafman_main/srv/insert_model.srv)**: a service message used to insert a UAV in the simulation.
- **utrafman_main/[mon_get_locs.srv](https://github.com/I3A-NavSys/utrafman_sim/tree/main/src/gazebo-ros/src/utrafman_main/srv/mon_get_locs.srv)**: a service message used to get telemetry of UAVs from the Monitoring service.
- **utrafman_main/[reg_get_fps.srv](https://github.com/I3A-NavSys/utrafman_sim/tree/main/src/gazebo-ros/src/utrafman_main/srv/reg_get_fps.srv)**: a service message used to get (a / a list of) flight plans of UAVs from the Register service.
- **utrafman_main/[reg_get_operators.srv](https://github.com/I3A-NavSys/utrafman_sim/tree/main/src/gazebo-ros/src/utrafman_main/srv/reg_get_operators.srv)**: a service message used to get (a / a list of) operators from the Register service.
- **utrafman_main/[reg_get_uavs.srv](https://github.com/I3A-NavSys/utrafman_sim/tree/main/src/gazebo-ros/src/utrafman_main/srv/reg_get_uavs.srv)**: a service message used to get (a / a list of) UAVs from the Register service.
- **utrafman_main/[reg_reg_flightplan.srv](https://github.com/I3A-NavSys/utrafman_sim/tree/main/src/gazebo-ros/src/utrafman_main/srv/reg_reg_flightplan.srv)**: a service message used to register a flight plan into the Register service.
- **utrafman_main/[reg_reg_operators.srv](https://github.com/I3A-NavSys/utrafman_sim/tree/main/src/gazebo-ros/src/utrafman_main/srv/reg_reg_operators.srv)**: a service message used to register a operator into the Register service.
- **utrafman_main/[reg_reg_uavs.srv](https://github.com/I3A-NavSys/utrafman_sim/tree/main/src/gazebo-ros/src/utrafman_main/srv/reg_get_uavs.srv)**: a service message used to register a UAV intos the Register service.
<!-- Add the teletransport service -->
- **utrafman_main/[teletransport.srv](https://github.com/I3A-NavSys/utrafman_sim/tree/main/src/gazebo-ros/src/utrafman_main/srv/teletransport.srv)**: a service message used to teletransport a UAV to a different position in the world.



### Topics
The topics genereted by each UAV in the simulation are:
<!-- - **/god/insert**: a topic that allows inserting a UAV in the environment. To insert a drone, it is necessary to send a message with the sdf model of the UAV to be inserted.
- **/god/remove**: a topic that allows to remove UAVs from the environment. To remove a drone, it is necessary to send the drone ID to remove. -->
- **/drone/%droneid/uplan**: a topic that allows sending a flight plan to a UAV. 
- **/drone/%droneid/telemetry**: a topic that allows receiving the telemetry of a UAV.
- **/drone/%droneid/kill**: a topic that allows killing a UAV.

Topics and services created by UTM services could be found in the [UTM services documentation](./utm_services.md).

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
- **classes**: a folder that contains definitions and methods of the classes. Documentation is included in the code. If you are using MATLAB, you can use the `doc` command to see the documentation of the class. A brief description of each class is shown below:
    - **UTMAirspace**: represents a UTM airspace. Inside it stores everything related to the airspace: the services (named `S_*`) and the simulation time (Gclock). It also stores information to locate the ROS master. It has methods to connect to the ROS network, update the simulation time and store simulation information.

    - **WorldModel**: represents the world model through an occupancy matrix. It has methods to read the world definition in `.wc` format, check the occupancy of a position in the world and generate random paths from the desired distance.

    - **Operator**: an operator represents the owner of an aircraft, and who establishes the flights to be performed in a timely manner. This operator has a unique identifier and can have several aircraft under his care. Among its methods is the functionality to register new UAVs, register new flight plans and send flight plans to UAVs for execution. In addition, in order to communicate with the aircraft, each operator has a node registered in the ROS network.

    - **UAVProperties**: represents the properties of a UAV, storing its unique identifier, the aircraft model and operator to which it belongs, the initial location and its status.

    - **FlightPlanProperties**: represents the properties of a flight plan. In this case, a flight plan is composed of a series of 4D waypoints, which we call oute, indicating that the UAV must navigate through specific coordinates in a specified time. Its properties include the UAV and assigned operator information, the unique identifier of the flight plan, the priority, the status and the DTTO (Desired Time To Take Off). Its methods include the parser that transforms this MATLAB object into a ROS message and a method that implements the abstraction layer mentioned above.

    - **SimulationProcesser**: this is a special class used to process and analyse simulation data after the simulation is finished. It converts the ROS messages stored in Registry and Monitoring service (see [UTM services documentation](./utm_services.md)) to MATLAB arrays, because MATLAB process the latter in a more efficient way. One the data is processed (basically, calling _SimulationProcesser()_ constructor method with the _UTMArispace_ object), offers method to get, filter and compute data from the simulation. Also offers some methods to analyze and plot the data, such as the conflict checkers and the telemetry viewer. This _SimulationProcesser_ object could be saved in a _.mat_ file, so it is possible to load it and continue the analysis later.
    

- **config**: a folder that contains configuration files.
    - **ros.m**: file to configure ROS network information for MATLAB.

- **simulations**: a folder that contains simulation control files.
    - **simple_simulation.m**: the most simple simulation, where a defined number of UAVs are allocated and each one perform a FP of 500 meters inside 'generated_city' world.
    - **longterm_simulation.m**: long-term simulation, where a defined number of UAVs are allocated and each one receives a FP each round, inside 'generated_city' world. Total number of rounds depends on defined total time and routes distance.


- **tools**: a folder that contains tools:
    - **ros_custom_message_compiler_MATLAB.m**: contains the necessary code to compile ROS custom defined messages to be used in MATLAB

In this image, you can see a class diagram of the classes defined in MATLAB:
![Class diagram](./diagrams/classes-diagram.png).