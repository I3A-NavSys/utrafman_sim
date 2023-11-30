# MATLAB API

The MATLAB component in the repository is structured as the following directories:

## Classes

Here resides the main files used by MATLAB for representing, analyzing, and processing aspects and data of U-TRAFMAN:

>In MATLAB, you can utilize the `doc` command to access detailed documentation for each class. 

- **classes/FlightPlanProperties**: This class depicts the properties of a flight plan. A flight plan comprises a series of 4D waypoints (denoted as _route_), defining the specific coordinates the UAV must navigate within a specified timeframe.

- **classes/Operator**: The Operator class represents the owner of an aircraft and is responsible for scheduling flights. Each operator has a unique identifier and can manage multiple aircraft. The class includes methods for registering new UAVs, creating new flight plans, and dispatching flight plans to UAVs for execution. Each operator is associated with a node registered in the ROS network, facilitating communication with the aircraft.

- **classes/SimulationProcesser**: This specialized class is responsible for processing and analyzing simulation data once the simulation has concluded. It performs the conversion of ROS messages stored in the Registry and Monitoring service (refer to the [UTM services documentation](./utm_services.md)) into MATLAB arrays. MATLAB processes these arrays more efficiently. After processing the data (typically by calling the constructor method with the UTMAirspace object), the class provides methods for retrieving, filtering, and computing information from the simulation data. Additionally, it offers various data analysis and visualization methods, including conflict checkers and a telemetry viewer. The _SimulationProcesser_ object can be saved in a _.mat_ file, allowing for later retrieval and continuation of the analysis.

- **classes/UAVProperties**: This class is designed to represent the properties of a UAV. It stores essential information, including a unique identifier, the aircraft model, the operator to whom it belongs, the initial location, and its current status.

- **classes/UTMAirspace.m**: This class serves as a representation of a UTM airspace. It includes details about the services (referred to as `S_*`), the simulation time (Gclock), and information for locating the ROS master. The class also provides methods to establish connections to the ROS network, update simulation time, and store simulation-related data.

- **classes/WorldModel.m**: This class is responsible for portraying the world model using an occupancy matrix. It offers methods for reading the world definition in `.wc` format, checking the occupancy status of a specific position in the world, and generating random paths based on a desired distance.

This diagram shows the MATLAB classes defined in the U-TRAFMAN simulator:
![Class Diagram](./diagrams/classes-diagram.png)

Additionally, two services classes are located in this folder, as mentioned and used by some of the classes before:

- **classes/S_Monitoring.m**: Monitoring service class. Monitors telemetry data from UAVs working in airspace, allowing storing and retrieving of said data.

- **classes/S_Registry.m**: Registry service class. Maintains the state of airspace and an updated registry of the entities in said airspace

>For more information about these services, refer to the [UTM services documentation](https://i3a-navsys.github.io/utrafman_sim/#/utm_services.md))


## Simulation scripts

This folder contains scripts used to automatically run predefined simulations, allowing fast execution of a flight scenario:

- **simulations/simple_simulation.m**: This script represents the simplest simulation scenario. It allocates a specified number of UAVs, each of which executes a flight plan covering a distance of 500 meters within the 'generated_city' world.

- **simulations/longterm_simulation.m**: This script implements a long-term simulation in which a predefined number of UAVs are allocated. Each UAV receives a flight plan within the 'generated_city' world in each simulation round. The total number of simulation rounds is determined by the defined total time and route distances.


## Tools

Scripts are located here for miscellaneous use by MATLAB:

- **tools/_error.m**: Computes the error obtained between the planned flight route and the actual traveled route of a UAV.

- **tools/generate_world.m**: Script used for testing a random generation of buildings in a world.

- **tools/ros_custom_message_compiler_MATLAB.m**: This script contains the necessary code to compile ROS custom-defined messages for use in MATLAB.

- **tools/UTRAFMAN_init.m**: Defines global variables `ROS_MASTER_IP` and `UTRAFMAN_DIR` used by further files. They state the IP ROS will use and the directory where U-TRAFMAN is located, respectively.

## Configuration files

Configuration files are stored here, allowing the user to adapt the simulator to its working environment:

- **config/ros.m**: This file allows the configuration of ROS network information for MATLAB.

