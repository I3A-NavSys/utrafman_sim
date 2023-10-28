# Installation

## Operative system

Configure a computer with **Ubuntu 20.04.6 LTS (Focal Fossa)**. You can obtain a Desktop Image [here](https://releases.ubuntu.com/focal).

>:warning: U-TRAFMAN has been developed and tested in the specified Ubuntu version. While it might work in other versions, there are no guarantees of compatibility.

You can install it on either a physical machine or a virtual machine. We have successfully tested it with VMware Workstation Player. You can obtain it [here](https://www.vmware.com/es/products/workstation-player/workstation-player-evaluation.html).


## ROS and Gazebo

U-TRAFMAN runs on **ROS** (Robot Operating System) **Noetic**. 
You can follow the official tutorials to install it [here](https://wiki.ros.org/noetic/Installation/Ubuntu).

Don't forget to perform a _full desktop_ installation and install the necessary dependencies for building packages.

As a part of ROS, the Gazebo simulator will be installed. To check its installation, execute the following command in a terminal:

```bash
gazebo
```

Graphycal interface of Gazebo should open and you sholud see something like this:

![Gazebo](./img/gazebo.png 'Gazebo simulator. :size=300px')

>:warning: U-TRAFMAN has been developed and tested in the specified ROS/Gazebo versions. While it might work in other versions, there are no guarantees of compatibility.



## U-TRAFMAN sim

Once your Gazebo and ROS installation is complete, you can install U-TRAFMAN Simulator. **Remember to define the ROS environment variables, as explained in the ROS installation tutorial!**.

