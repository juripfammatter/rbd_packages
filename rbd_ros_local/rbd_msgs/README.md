# Imlpemented services

## SetPosition
Request:<br> `geometry_msgs/Pose goal` <br>
Response:<br> `string message` <br>
`bool success` <br>
`float64 distance`<br><br>
Set goal position of position controller.

<br><br>
## GeneratePath
Request:<br> `string command` <br>
Response:<br> `geometry_msgs/Pose[] poses` <br>
`int32 nr_of_poses` <br><br>
Generates Path based on command. Current commands are : 
* "wiggle"