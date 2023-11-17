<!-- This file set the content for the main of the documentation (shown after the cover page). -->

Welcome to U-TRAFMAN Simulator documentation! 

Use the menu on the left to navigate through the documentation. You can use the search box at the top left to search for a specific topic. 

If you have any questions, please contact us at 
`[rafael.casado, aurelio.bermudez] (at) uclm.es`.


<p style="display:flex;justify-content:space-evenly;align-items:center">
	<a href="https://www.uclm.es/es/centros-investigacion/I3A/secciones-investigacion/RAAP"/> <img src="/img/RAAPlogo.png" width=200px/>
	<a href="https://www.uclm.es/centros-investigacion/i3a?sc_lang=en"/> <img src="/img/I3Alogo.png" width=200px/>
	<a href="https://www.uclm.es"/> <img src="./img/UCLMlogo.png" width=200px/>
</p>	


# U-TRAFMAN Simulator
The main purpose of this simulator is to allow the user to develop functionalities for Urban Air Mobility (UAM) and Advanced Air Mobility (AAM), as well as realistically simulate flight plans and navigations of Unmanned Aerial Vehicles (UAVs) in urban scenarios. Additionally, U-TRAFMAN offers the following features:

- **Development, testing, and implementation of UAV autonomous control software:** onboard and offboard control systems for UAVs can be implemented and tested.
- **Development and appraisal of Unmanned Traffic Management (UTM) services:** UTM services and frameworks can be developed and tested to validate if their proposals are valid. Information between aircraft and UTM services can be exchanged through topics, stored, and analyzed after simulation for further analysis and debugging. 
<!-- UTM services could be implemented in whatever platform you want if the platform is compatible with ROS. -->
- **Provide realistic dynamics simulations of hundreds of UAVs inside an urban area:** Due to the physics engine of the Gazebo simulator and its possibilities, UAV dynamic behavior can be implemented, making the simulations close to real-world behavior.

We offer a complete installation guide and a simple simulation to let the user know the basics of U-TRAFMAN, described in the [Install](https://i3a-navsys.github.io/utrafman_sim/#/install) section.

## How is the simulator composed?
The simulator works with the help of 3 essential components/technologies that appeal to the features described before:
- **Gazebo**: provides a complete physics engine, dynamics engine, and an environment and collision simulator. Using Gazebo, UAVs can be placed in a world populated with obstacles, buildings, etc. Using UAVs' plugins, the behavior of the UAVs can be adjusted to user preference, making the UAVs fly autonomously. This plugin system also implements communication channels between UAVs and UTM services. Moreover, Gazebo allows the use of different sensors and hardware to extend the UAVs' capabilities.
- **ROS**: Robotic Operating System, or ROS, is a middleware that provides a communication system between different processes. In U-TRAFMAN, ROS is used to communicate UAVs with UTM services and other software. Topics and services are used to abstract communication systems from the development point of view.
- **MATLAB**: used to provide a way to launch simulations, run UTM services, and analyze the data gathered during the simulation. MATLAB is employed due to its simplicity and potent data analysis tools. MATLAB is not mandatory to use the simulator, as communications between UAVs and UTM services are done using ROS, so other tools that can communicate with ROS could be used as a replacement.

 You can find more information about these tools in the [Architecture](https://i3a-navsys.github.io/utrafman_sim/#/architecture) section.

## Previous knowledge
The simulator is built using ROS and Gazebo, so knowing about these tools is necessary. Advanced concepts of these tools are not used, but knowing how ROS network works, the use of packages, how to create a topic, how to create a service, etc. **is a must**. If you are unfamiliar with ROS, read the [ROS wiki](http://wiki.ros.org/ROS/Tutorials) to learn the basics. If you are unfamiliar with Gazebo, read the [Gazebo tutorials](http://gazebosim.org/tutorials) to learn the basics. 

If you already have knowledge about these tools, and you have the simulator installed with all of its pre-requisites, you can proceed to the [ROS API](https://i3a-navsys.github.io/utrafman_sim/#/ROS_api) and the [MATLAB API](https://i3a-navsys.github.io/utrafman_sim/#/MATLAB_api) sections to study their implementations.