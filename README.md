# RoboDog (rbd) Packages
This repository serves as the codebase for the Bachelor's Thesis of *Juri Pfammatter* and *Daniel Schweizer*. It's split up into three subfolders, each to be placed in another environment: 
* `rbd_local`: these files require Python 3.10 and have therefore be executed in a conda environment with the required dependencies. They communicate via the rosbridge with the ROS packages.
* `rbd_ros`: these are the ROS packages running on the robot. They can simply be copied into the `catkin_ws` folder of the Nvidia Xavier board. 
* `rbd_ros_local`: these are the ROS packages running on the local Ubuntu VM on a Laptop connected via the WiFi Hotspot.  They feature the gesture recognition algorithm and the visualization.

More information about the project and a quick-start guide can be found [here](https://robodog.gitbook.io/robodog-documentation/).
