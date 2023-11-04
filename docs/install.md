# Installation


## Set up your machine

You can install U-TRAFMAN on either a real computer or a virtual machine. In the second case, we have successfully tested it with VMware Workstation Player 17. You can obtain it [here](https://www.vmware.com/es/products/workstation-player/workstation-player-evaluation.html).

Configure a computer with **Ubuntu 20.04.6 LTS (Focal Fossa)**. You can obtain a Desktop Image [here](https://releases.ubuntu.com/focal).

>:warning: U-TRAFMAN has been developed and tested in the specified Ubuntu version. While it might work in other versions, there are no guarantees of compatibility.


## ROS and Gazebo

U-TRAFMAN runs on **ROS** (Robot Operating System) **Noetic**. 
Please follow the official [tutorial](https://wiki.ros.org/noetic/Installation/Ubuntu) to install it.
Make sure that:
- A _full desktop_ installation is performed.
- The necessary dependencies for building packages are installed.
- File `/opt/ros/noetic/setup.bash` was sourced in your `.bashrc`.

As a part of _ROS Noetic_, the _Gazebo 11_ simulator will be installed. To check it, execute the command `gazebo` in a terminal. The graphical interface of Gazebo should open:

![Gazebo](./img/gazebo.png 'Gazebo simulator. :size=600px')

>:warning: U-TRAFMAN has been developed and tested in the specified ROS/Gazebo versions. While it might work in other versions, there are no guarantees of compatibility.


## U-TRAFMAN package

Once your ROS/Gazebo installation is complete, you can install U-TRAFMAN as a ROS package.
You need to clone the repository in your ROS installation:
```bash
cd /opt/ros/noetic/share
sudo git clone https://github.com/I3A-NavSys/utrafman_sim
```

After cloning, change the permissions of the `utrafman_sim` folder to your user executing the following commands (replace `username` with your actual _username_):
```bash
sudo chown -R username utrafman_sim
sudo chgrp -R username utrafman_sim
```

A ROS (Catkin) workspace containing the simulation environment is located in `src/gazebo-ros/`. 
You need to compile it with the following commands:
```bash
cd utrafman-sim/src/gazebo-ros
catkin_make
```
>:warning: Make sure that the `/opt/ros/noetic/setup.bash` file was sourced in your `.bashrc` file as instructed in the ROS installation step above. Failure to do so may cause the `catkin_make` command to not work properly.

You also need to source the compiled workspace into your `.bashrc` file for it to be accessible:
```bash
echo "source /opt/ros/noetic/share/utrafman_sim/src/gazebo-ros/devel/setup.bash" >> ~/.bashrc
source ~/.bashrc
```


## MATLAB

U-TRAFMAN provides a variety of tools for launching and analyzing simulations. These tools are programmed in MATLAB®.
To install MATLAB, you can follow these steps:


1. Download MATLAB R2023a from the official MathWorks website: [MathWorks Downloads](https://es.mathworks.com/downloads).

   >:warning:  U-TRAFMAN simulator has been tested in **MATLAB R2023a**. While it might work in other versions, there are no guarantees of compatibility.

2. Unzip the installer package by running the following commands in your terminal:
```bash
unzip matlab_R2023a_glnxa64.zip -d matlabinstaller
cd matlabinstaller
```
3. Execute the installation script with root privileges:
```bash
sudo ./install
```
4. Follow the installation process:

   a) Select your license and user information.
   
   b) Choose the destination folder (the default location `/usr/local/MATLAB/R2023a` is fine).

   c) Select the MATLAB products you want to install. Ensure that you select at least the following components:
      - MATLAB R2023a
      - Parallel Computing Toolbox
      - ROS Toolbox
      
   d) Configure installation options:
      - Set _Create symbolic links to MATLAB scripts in:_ `/usr/local/bin`
      - Choose _Improve MATLAB startup performance_ based on your preferences.
      
   e) Confirm your selections.

Once MATLAB is installed, you can delete the temporary `matlabinstaller` folder.

>:warning:  The recommended setup is to install ROS/Gazebo and MATLAB on the same computer. However, in the case of very large simulations where the resources of typical machines may be insufficient, it may be beneficial to run MATLAB on a different computer, including Windows platforms. In such scenarios, make sure that the computers are connected to the same network and can communicate with each other.

In a new terminal, open MATLAB in the working folder employing the following commands:
```bash
cd /opt/ros/noetic/share/utrafman_sim/src/matlab
matlab
```
Matlab indicates that there is a new release available and prompts you to update. Click on "Don't Show Again."


### Compiling ROS messages with MATLAB

Custom ROS messages are used by U-TRAFMAN. Although the simulator is not coded in Python, MATLAB will require it to compile ROS messages. You can install Python 3.8 executing the following command in a terminal:
```bash
sudo apt install python3.8-venv
```
>⚠️ U-TRAFMAN simulator has been tested with **Python 3.8**. While it may work with other versions, there are no guarantees of compatibility.

To compile ROS messages, 

1. Create folder "/tmp/venv"
2. Run matlab as root to compile ROS messages (otherwise, a warning appears))
3. Edit the script `tools/ros-custom-message-compiler.m` to define where the working directory (repo) and python is installed on your computer.
4. Run the script. If everything is correct, you should see a message in the MATLAB console saying `Build succeeded`.

You could find more information about how to compile custom ROS messages [here](https://es.mathworks.com/help/ros/custom-message-support.html?s_tid=CRUX_lftnav). 


### Setting up ROS_MASTER IP address in MATLAB
MATLAB must be configured with ROS_MASTER IP address, as explained in this [tutorial](https://es.mathworks.com/help/ros/ug/get-started-with-ros.html).
- If you use a one-computer setup, you can simply use `localhost` as ROS_MASTER IP address.
- If you use a two-computer setup, you must configure MATLAB with the IP address of the computer where ROS is running, in the file `/src/matlab/config/ros.m`. 


## Running your first simulation

U-TRAFMAN includes a basic simulation to help you verify if everything is functioning correctly. To get started, open a new terminal and execute the following command:

```bash
roslaunch utrafman_main generated_city.launch
```
You should observe ROS being launched in the terminal, and Gazebo opening in a new window. In the Gazebo window, you should see a world with simple buildings.

![Empty World](./img/tutorials/simple-simulation-1.png 'Gazebo Viewer with a generated city world.  :size=800px')


Next, to add UAVs to the world and send flight plans, follow these steps:

1. In MATLAB, set `opt/ros/noetic/share/utrafman_sim/src/matlab/` as the current directory.
2. From that directory, run `simulations/simple_simulation.m`. This script will perform the following tasks:
   - Connect with the ROS master from MATLAB.
   - Load the world definition file in MATLAB.
   - Instance an _operator_ and 10 _UAVs_.
   - Register both the operator and UAVs using the "Registry" service.
   - Generate a flight plan for each UAV with a random route of 500 meters.
   - Send the generated flight plans to the UAVs.

This will set up your simulation environment with UAVs and their flight plans.


![UAVs](./img/tutorials/simple-simulation-2.png 'Gazebo Viewer. Ten UAVs flying in the world. :size=800px')

![UAVs](./img/tutorials/simple-simulation-3.png 'Gazebo Viewer. Ten UAVs flying in the world. :size=800px')

Each time a random route is generated for a UAV, the script will display it in a 3D figure, showing both the flight path and the buildings in the world. This visualization can help you understand the path that each UAV will follow in the simulation.

![Routes generated](./img/tutorials/simple-simulation-random-routes.png 'Random routes generated :size=800px')

Once the simulation has completed, you will have access to a `SimulationProcesser` object in the MATLAB workspace, referenced by the variable name `SP`. This object allows you to retrieve and analyze simulation data. For instance, if you want to visualize the telemetry data of a specific flight plan, you can execute the following command in MATLAB:

```matlab
SP.telemetryViewer(fp_id);
```
Where `fp_id` is the identifier of the flight plan you wish to analyze. This command will enable you to view and analyze telemetry data for that specific flight plan.


![Telemetry Viewer](./img/tutorials/simple-simulation-telemetry-viewer.png 'MATLAB Telemetry Viewer :size=800px')



