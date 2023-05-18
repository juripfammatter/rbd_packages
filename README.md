# RoboDog (rbd) Packages
This repository serves as the codebase for the Bachelor's Thesis of *Juri Pfammatter* and *Daniel Schweizer*. It's split up into three subfolders, each to be placed in another environment: 
* `rbd_local`: these files require Python 3.10 and have therefore be executed in a conda environment with the required dependencies. They communicate via the rosbridge with the ROS packages.  <br>
**Depedencies:**
    * [`anaconda`](https://repo.anaconda.com/archive/Anaconda3-2023.03-Linux-x86_64.sh)
    * `(robodog) user$ pip install -r  `[`requirements.txt`](https://github.com/juripfammatter/rbd_packages/blob/main/rbd_local/zhaw_ba_robodog/requirements.txt)
    * [`rosbridge`](http://wiki.ros.org/rosbridge_suite)
<br><br>
* `rbd_ros`: these are the ROS packages running on the robot. They can simply be copied into the `catkin_ws` folder of the Nvidia Xavier board. <br>
**Depedencies:**
    * [`ouster-ros`](https://github.com/ouster-lidar/ouster-ros) (including dependencies)
    * [`robot_localization`](https://github.com/cra-ros-pkg/robot_localization/tree/melodic-devel)
    * [`qre_a1 version 3.2.0`](https://github.com/MYBOTSHOP/qre_a1)  (private repo)
    * [`zed-ros-wrapper`](https://github.com/stereolabs/zed-ros-wrapper) (including dependencies)
    * [`ZED SDK 3.8.2`](https://download.stereolabs.com/zedsdk/3.8/l4t32.5/jetsons) for L4T 32.5 (Jetpack 4.5)
<br><br>
* `rbd_ros_local`: these are the ROS packages running on the local Ubuntu VM on a Laptop connected via the WiFi Hotspot.  They feature the gesture recognition algorithm and the visualization.<br>
**Depedencies:**
    * [`Gazebo`](http://wiki.ros.org/gazebo_ros_pkgs)
    * [`rqt_multiplot`](https://github.com/ANYbotics/rqt_multiplot_plugin) (including dependencies)
    * [`Rviz`](http://wiki.ros.org/rviz)
<br><br>

# Mounting Hardware
1. Mount the camera mount to the ZED2 Stereo camera.
2. Mount the camera with the camera mount to the robot.
3. Insert two M3 screws diagonally into the LiDAR mount.
4. Connect the power cable to the 19 V output and the LiDAR's USB-cable to the robot. 
5. Mount the LiDAR mount onto the robot.
6. Connect the ZED2 camera via its USB-cable.

# Starting the Robot
1. Check the battery status by pressing the power button located on the battery on the left-hand side once.
2. Start the robot by pressing the power button once and then holding it until the LEDs begin to move.
3. After a few seconds connect your local machine to the Unitree's WiFi hotspot.
4. Unplug the USB-cable of the LiDAR for one second, then plug it back in.
5. Check the connection to the robot with `ping unitree@192.168.123.12`.
6. Place all files from `rbd_local/utilities` in your home directory.
7. Start the whole system by running `./rbd_startup.sh` in your home directory.
8. Now the GUI, camera image and Rviz should open a window on your local machine.

More information about the project and a quick-start guide can be found [here](https://robodog.gitbook.io/robodog-documentation/).
