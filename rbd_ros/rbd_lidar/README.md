# rbd_lidar

## Launch Files
`ouster_record.launch metadata_file:=metadata_name.json bag_name.bag`<br>
* Records via Ouster node (to be replayed with `ouster_replay.launch`).<br>
* `metadata_file.json`: metadata file to be written too (needed for replay).<br>
* `bag_name.bag`: desired name of .bag file to be written to.<br><br>
 
`ouster_replay.launch metadata_file:=metadata_name.json bag_name`<br>
* Replay .bag files recorded with `ouster_record.launch`.
`metadata_file.json`: metadata file to be initialized with(same as in recording!).<br>
* `bag_name.bag`: name of .bag file to be replayed.<br><br>

`rbd_lidar_local.launch`<br>
* Starts lidar node only.<br><br>

`rbd_lidar.launch`<br>
* Starts lidar and Ouster node (in 512x10 mode).