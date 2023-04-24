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

More information about the project and a quick-start guide can be found [here](https://robodog.gitbook.io/robodog-documentation/).
