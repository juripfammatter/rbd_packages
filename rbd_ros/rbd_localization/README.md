# rbd_localzation
Node currently not in use!.
Implementation of discrete integration and filtering of IMU sensor data.

## Launch Files
`rbd_localization.launch` <br>
* Starts localizatoin node id `enable_loc_node = true` (defaults to false).
* Starts ZED2 node if `enable_zed = true` (defaults to false).
* Starts EKF localization with `localization.yaml` configuration file.