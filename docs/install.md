# Installation

## Operative system

Configure a computer with **Ubuntu 20.04.6 LTS (Focal Fossa)**. You can obtain a Desktop Image [here](https://releases.ubuntu.com/focal).

>:warning: U-TRAFMAN has been developed and tested in the specified Ubuntu version. While it might work in other versions, there are no guarantees of compatibility.

You can install it on either a physical machine or a virtual machine. We have successfully tested it with VMware Workstation Player. You can obtain it [here](https://www.vmware.com/es/products/workstation-player/workstation-player-evaluation.html).


## ROS and Gazebo

U-TRAFMAN runs on **ROS** (Robot Operating System) **Noetic**. 
You can follow the official tutorials to install it [here](https://wiki.ros.org/noetic/Installation/Ubuntu).

Don't forget to perform a _full desktop_ installation and install the necessary dependencies for building packages.

As a part of ROS, the Gazebo simulator will be installed. To check its installation, execute the command `gazebo` in a terminal. The graphical interface of Gazebo should open, and you should see something like this:

![Gazebo](./img/gazebo.png 'Gazebo simulator. :size=300px')


>:warning: U-TRAFMAN has been developed and tested in the specified ROS/Gazebo versions. While it might work in other versions, there are no guarantees of compatibility.



## U-TRAFMAN package

Once your ROS/Gazebo installation is complete, you can install U-TRAFMAN as a ROS package.
You need to clone the repository in your ROS installation:
```bash
cd /opt/ros/noetic/share
sudo git clone https://github.com/I3A-NavSys/utrafman_sim
```

To change the permissions and/or owner of the `utrafman_sim` folder after cloning to your user, execute the following commands (replace `username` with your actual _username_):

```bash
sudo chown -R username utrafman_sim
sudo chgrp -R username utrafman_sim
```


As you can see, `src/gazebo-ros/` is a ROS (Catkin) workspace containing the entire simulation environment. On the other hand, `src/matlab/` includes simulator launch code, tools, and a telemetry viewer.

Once you have cloned the repository, you need to compile the code. To do so, you can use the following commands:
```bash
cd utrafman-sim/src/gazebo-ros
catkin_make
```

>:warning: Please note that `catkin_make` will not work if you haven't sourced the ROS `/opt/ros/noetic/setup.bash` file. Be sure to add the source command to your `.bashrc` as instructed in the ROS installation steps above.


Once the workspace is compiled, you also need to source the workspace into your `.bashrc` file for it to be accessible:

```bash
echo "source /opt/ros/noetic/share/utrafman_sim/devel/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

After doing this, you should be ready to run simulations.


