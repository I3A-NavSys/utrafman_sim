# 4. UTM Services <!-- {docsify-ignore} -->
Unmanned Traffic Management (UTM) is an emerging technology that will be used to manage the increasing number of unmanned aerial vehicles (UAVs) in the airspace. UTM will enable the safe, efficient, and secure integration of UAVs into the existing air traffic management system. The UTM system will facilitate the safe exchange of information between UAVs, pilots, and other airspace users to ensure that UAVs operate safely and securely. To achieve that, UTM will offer something called "_services_", which are information exchange systems, provided by different services providers, such as governments, private companies, etc. 

Some of these services are:
- Airspace authorization.
- Airspace management.
- Flight coordination.
- Real-time traffic information.
- Weather information.
- Conflict detection and resolution.
- etc.

As UTM technology advances, the services it offers will become more reliable, secure, and efficient, enabling the safe integration of UAVs into the airspace. But nowadays, UTM is still in its infancy, and there are not many services available. Services in development must be tested, and this is where U-TRAFMAN Simulator comes in. U-TRAFMAN Simulator is a flexible simulator that allows the development and test UTM services. These services could be implemented in MATLAB, as it is a widely-used language in the aerospace industry and the main language in this project, but **you can use any other language if you want**.

<!-- In this 1.0 version, U-TRAFMAN does not offers any UTM service. It just offer a way to implement them. However, in the next versions, some UTM services will be implemented.  -->

## 4.1. UTM Services in U-TRAFMAN Simulator
UTM Services inside U-TRAFMAN Simulator are implemented using MATLAB. The communication between services is done using ROS Topics and Services. This allow the services to be implemented in any language that supports ROS.

### 4.1.1. Registry Service
The implementation of this service could be found in [S_Registry.m](https://github.com/I3A-NavSys/utrafman_sim/blob/main/src/matlab/classes/S_Registry.m) .Registry Service maintains the information of entities in the airspace. It is a central service that allows other services to access the information of entities in the airspace. Every time a operator, UAV or Flight Plan is created, it is registered in the Registry Service. The Registry offers each one a unique ID, which is used to identify it.

The topics and services offered by the Registry Service are:

<table>
    <thead>
        <tr>
            <th>Topic</th>
            <th>Type</th>
            <th>ROS Message</th>
            <th>Description</th>
        </tr>
    </thead>
    <tr>
        <td>/service/registry/reg_new_operator</td>
        <td>Service</td>
        <td>utrafman_main/reg_new_operator</td>
        <td>Used to register a new operator</td>
    </tr>
    <tr>
        <td>/service/registry/reg_new_uav</td>
        <td>Service</td>
        <td>/service/registry/reg_new_uav</td>
        <td>Used to register a new UAV</td>
    </tr>
    <tr>
        <td>/service/registry/reg_new_fp</td>
        <td>Service</td>
        <td>utrafman_main/reg_new_flightplan</td>
        <td>Used to register a new FlightPlan</td>
    </tr>
    <tr>
        <td>/service/registry/get_operators</td>
        <td>Service</td>
        <td>utrafman_main/reg_get_operators</td>
        <td>Used to get a list of operators. If a operatorId is specified, the requested only retrieve information about this operator</td>
    </tr>
    <tr>
        <td>/service/registry/get_uavs</td>
        <td>Service</td>
        <td>utrafman_main/reg_get_uavs</td>
        <td>Used to get a list of UAVs. If a uavId is specified, the requested only retrieve information about this UAV</td>
    </tr>
    <tr>
        <td>/service/registry/get_fps</td>
        <td>Service</td>
        <td>utrafman_main/reg_get_fps</td>
        <td>Used to get a list of Flight Plans. If a fpId is specified, the requested only retrieve information about this Flight Plan</td>
    </tr>
    <tr>
        <td>/service/registry/new_uav_advertise</td>
        <td>Topic</td>
        <td>utrafman_main/UAV</td>
        <td>A new message is published when a new UAV is sign u p in the registry</td>
    </tr>
</table>

### 4.1.2. Monitoring Service
The implementation of this service could be found in [S_Monitoring.m](https://github.com/I3A-NavSys/utrafman_sim/blob/main/src/matlab/classes/S_Monitoring.m). Monitoring Service is a service that monitors the airspace receiving telemetry data from UAV and stoing them. It also provides a way to retrieve this data. This service is useful to monitor the behavior of UAVs in the airspace.

The topics and services offered by the Monitoring Service are:
<table>
    <thead>
        <tr>
            <th>Topic</th>
            <th>Type</th>
            <th>ROS Message</th>
            <th>Description</th>
        </tr>
    </thead>
    <tr>
        <td>/service/monitoring/get_telemetry</td>
        <td>Service</td>
        <td>utrafman_main/mon_get_locs</td>
        <td>Used to get the telemetry data of a UAV. A uavId is mandatory to be specified.</td>
    </tr>
    <tr>
        <td>/service/monitoring/get_current_loc</td>
        <td>Service</td>
        <td>/service/monitoring/mon_get_locs</td>
        <td>Used to get the current location of a UAV. An uavid is mandatory to be specified.</td>
    </tr>
</table>

In this image, you can see a class diagram of this services:

![Class diagram](./diagrams/services-diagram.png 'Services class diagram  ').

## 4.2. Interaction between UTM Services
Here is an example of how this service interacts between them and the simulator when a simulation is in progress:

```mermaid
sequenceDiagram
    participant O as Operator
    participant R as Registry Service
    participant G as Gazebo God
    participant D as Gazebo Drone E.S.
    participant M as Monitoring Service

    rect rgba(255,179,186,0.5)
        O->>+R: Register operator
        R->>-O: Operator reg. with id X
    end

    rect rgba(255,223,186,0.5)
        Note over O: Create new UAV
        O->>+R: Register UAV
        R->>+G: Insert UAV with id Y
        G--xD: Launch E.S. for UAV Y
        G->>-R: UAV Y inserted
        R-->>+M: UAV Y added to the simulation
        activate M
        M-->M: Subscribe to UAV Y Telemetry topic
        deactivate M
        R->>-O: UAV reg. with id Y
    end

    rect rgba(255,255,186,0.8)
        Note over O: Create new Flight Plan
        O->>+R: Register Flight Plan for UAV Y
        R->>-O: Flight Plan reg. with id Z for UAV Y
    end

    rect rgba(186,255,201,0.5)
        Note over O: Start Flight Plan
        O->>+D: Start Flight Plan Z
    end

    rect rgba(186,225,255,0.2)
        Note over D: Start Flight Plan
        loop every 1s
            D-->>+M: Send telemetry data
            M-->M: Store telemetry data
            deactivate M
        end
    end
```
where solid line with arrow is a service resquest/response and dashed line with arrow is a topic message.