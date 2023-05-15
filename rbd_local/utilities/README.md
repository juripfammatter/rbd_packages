# Utilities
Place these files into your home directory for ease of use.

## Starting up RoboDog
1. First mount the ZED stereo camera then the Lidar with two screws diagonally.
2. Start the RoboDog by pressing the button on the battery once and then hold until the LED begin to move.
3. Connect to the Unitree hotspot.
4. Check connection with `ping unitree@192.168.123.12`.
5. Unplug the USB connection of the Lidar for one second.
6. Start up the whole system with `./rbd_startup.sh`.

## Mounting the `catkin_ws` folder on a local machine
1. Connect to the WiFi.
2. `source mount_sshfs.sh`
3. Enter the password of your local machine.
4. Enter the password of the Jetson board: `123`