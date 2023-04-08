# PA Robodog
Bachelor thesis from Juri Pfammatter and Daniel Schweizer based on the project thesis from Martin Oswald and Tenzin Langdun at Zürich University of Applied Science, Winterthur.

## Setup the environment
Clone this repository and create a [conda](https://docs.conda.io/projects/conda/en/latest/user-guide/install/index.html) environment:

````bash
conda create -n robodog python=3.9
conda activate robodog

````
on Windows:
````bash
python -m pip install -r C:\path\to\requirements.txt
````

on MacOS:
````bash
pip install -r path\to\requirements.txt
````

## Connect to the robodog

1. Start the robot
2. Connect to the A1's WIFI hotspot: The SSID of the Wifi network of A1's hot-spot begins with UnitreeRoboticsA1 and the default password is 00000000.
3. Open a terminal and connect to the robot with ssh -X unitree@192.168.123.12. Enter the password 123
4. This is important as it adds the robot to the known hosts.
5. Start the UI (app.py) and connect to the robot
6. Press on sit and activate gesture recognition

## Additional information:

The SDK files we created are in the example_files folder. Any additional SDK files must be scp'd to the robot's catkin_ws/utils/unitree_legged_sdk-3.2.0/examples folder.

The any new bash files like those in the bash_files folder belong in the home directory of the robot.
