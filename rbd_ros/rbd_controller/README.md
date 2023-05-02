# rbd_controller
Main controller node.

## Launch Files
`rbd_controller.launch` <br>
* Starts main controller node.
* Launches tf transformations.
* Launches navigation node.
* Launches position control node.
* Launches localization node (incl. EKF & ZED2).
* Launches highlevel controller.
* Launches lidar node (incl. Ouster).
