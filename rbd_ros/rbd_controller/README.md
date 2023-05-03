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
* Launches lidar node (incl. Ouster). <br><br>

`rbd_record.launch bag_name:=bagname.bag` <br>
* Needs to run on the robot because of latency issues when recoridng over SSH.
* Starts recording of specified topics.
* `bagname.bag` name of .bag file to be recorded to.
* To be replayed with `rbd_replay.launch` in `rbd_ros_local/rbd_gestrec`<br><br>

`test_control.launch` <br>
* Launchfile to test position controller.
* Launches position controller.
* Launches highlevel controller.
* Launches localization (incl. EKF & ZED2). <br><br>

`test_state_machine.launch` <br>
* Launchfile to test state machine.
* Launches main controller node.
* Launches navigation node.
* Launches position controller.
* Additionally `rbd_gestrec_py` and `gestrec.py` need to run locally.
