# Sustainable and Intelligent Air Mobility Simulator
The main objective of this simulator is allowing users to develop functionality and realistically simulate flight plans and navigations of Unmanned Aerial Vehicles (UAVs) in urban scenarios. Inside this main objective, there are some sub-objectives:
- **Develop and test UAV autonomous control software and services implementations:** control systems of the Unmanned Aerial Vehicles could be modified, reimplemented or amplified.
- **Develop and check UTM services:** services and frameworks could be developed and tested to validate if proposals are valid. Information between aircrafts and services can be exchanged through topics, stored and analysed after simulation to debug. Services could be implemented in whatever platform you want if the platform is compatible with ROS.
- **Provide realistic dynamics simulations of hundreds of UAV inside an urban area:** due to the physics engine of Gazebo simulator and its possibilities, UAV behaviour could be implemented, making the simulations close to real world behaviour.

## How the simulator is composed?
Nowadays, the simulator is composed of 3 main and essential components:
-	**ROS**: ROS Noetic could be the most critical part of the simulator. Provide communication between UAV and service/software implemented in other platforms. Topics and services are used to abstract communication systems from the development point-of-view. 
-	**Gazebo**: as is said before, Gazebo offers the entire physics and dynamics engine simulator, as well as the environment and collision simulator. Moreover, Gazebo allows the use of LiDARs and cameras onboard, where behaviour of these sensors could be implemented through Gazebo plugins.
-	**Simulations launcher and telemetry viewer**: a simulator launcher and telemetry viewer are provided in order to offer an example of how UAV are placed in the world, how to send a flight plan and a way to view afterward the route flied by the aircraft. These parts have been constructed using MATLAB.
