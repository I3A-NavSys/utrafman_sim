# Architecture

The U-TRAFMAN simulator is designed to enable the development of Urban Air Mobility (UAM) services. Specifically, it focuses on the following aspects:

- **Realistic Dynamics Simulation of Hundreds of UAVs in an Urban Area:** Leveraging the advanced physics engine of the Gazebo simulator, you can implement realistic dynamic behaviors for Unmanned Aerial Vehicles (UAVs), ensuring that the simulations closely resemble real-world scenarios.

- **Development and Testing of Autonomous UAV Control Software and Services:** The simulator provides support for the development and testing of both onboard and offboard control systems for UAVs, enabling the refinement of UAV control software.

- **Development and Testing of UTM Services:** U-TRAFMAN offers a platform for the development and testing of Unmanned Traffic Management (UTM) services and frameworks, allowing you to validate their effectiveness. Data generated during simulations can be stored and analyzed post-simulation to assist with further analysis and debugging.

The simulator has been developed using the following components and technologies:

## Robotic Operating System (ROS 1)

The Robotic Operating System (ROS) is an open-source robotics middleware that offers a comprehensive set of tools and libraries for developing complex robot applications. It enables rapid development and seamless integration of various robotic components, including sensors, actuators, and algorithms. ROS is designed to be a versatile and modular platform that is applicable across a diverse array of robots and provides a wide range of services. This middleware plays a pivotal role in numerous robotics applications, including autonomous vehicles, robotic manipulation, and service robots.

While advanced concepts are not extensively used, understanding the fundamentals of how the ROS network operates, working with packages, creating topics, services, and other basic ROS concepts is crucial. If you are unfamiliar with ROS, you can refer to the [ROS wiki](http://wiki.ros.org/ROS/Tutorials) to learn the basics. 

In U-TRAFMAN, communication between UAVs and Unmanned Traffic Management (UTM) services is streamlined through ROS topics and services. This abstraction simplifies the communication system from a development perspective, making it more manageable and accessible.


## Gazebo

Gazebo is a powerful 3D dynamic simulator known for its capability to accurately and efficiently simulate groups of robots within intricate indoor and outdoor environments. Gazebo provides a comprehensive physics and dynamics engine for simulation, along with environmental and collision simulation. Additionally, Gazebo supports the integration of LiDARs and cameras onboard robotic platforms, and the behavior of these sensors can be implemented using Gazebo plugins. This versatile simulator is a valuable tool for various robotic applications.

In U-TRAFMAN Sim, Gazebo is utilized to simulate both the environment and the UAVs. The environment is structured with a world where drones are placed. This world consists of models, which are the objects situated within it. For instance, world files may define a small city with roads, buildings, trees, and other elements.

Gazebo is initiated through a ROS launch file, which is located in the `utrafman_main` package. The Gazebo simulations are executed in separate threads, with one thread running the simulation server (_gzserver_), and another for the simulation viewer (_gzclient_). This separation allows you to pause, resume, stop, or interact with the simulation viewer as needed. It's particularly useful for debugging purposes or for conserving computational resources since _gzclient_ is not essential to run the simulation.

Gazebo simulations can be extended with plugins, which are libraries loaded into Gazebo to introduce new functionalities. For instance, in U-TRAFMAN Sim, a plugin is used to spawn and remove UAVs within the environment. Additionally, drone autonomous control software is implemented as a plugin, enabling the implementation of new control software without the need to modify the core simulator. This extensibility and modularity make Gazebo a powerful tool for enhancing the capabilities of U-TRAFMAN Sim.

If you are not familiar with Gazebo, you can explore the [Gazebo tutorials](http://gazebosim.org/tutorials) to acquire fundamental knowledge.



## MATLAB

MATLAB is a high-level programming language known for its support of matrix manipulations, data plotting, algorithm implementation, user interface creation, and interoperability with programs written in other languages such as C, C++, C#, Java, Fortran, and Python. Additionally, it offers seamless interaction with ROS networks.
MATLAB is extensively utilized in the aerospace industry, making it a popular choice for various applications.

In the context of the U-TRAFMAN Simulator, MATLAB serves the following purposes:
- Defining the simulation environment.
- Managing the insertion and removal of UAVs within the environment.
- Sending flight plans to UAVs.
- Receiving telemetry data from UAVs.
- Analyzing telemetry data to facilitate in-depth analysis.

>:heavy_check_mark: It's important to emphasize that MATLAB is not obligatory for utilizing the simulator. Communication between UAVs and UTM services, as well as among different UTM services, is established through ROS. Consequently, any other tool capable of communicating with ROS can be integrated with the simulator. U-TRAFMAN is designed to be adaptable and can be used with other programming languages, allowing you to take advantage of the provided MATLAB components and implement your work using a language of your preference.

> :warning: **MATLAB** is a registered trademark of The MathWorks, Inc. Furthermore, it is a commercial software, and a valid license is required for its use. However, a free trial version of MATLAB is available, offering a 30-day trial period for evaluation.

