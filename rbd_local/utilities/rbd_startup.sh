#connect to RoboDog and start ROS master
#"exec bash -i" will not close the tab
gnome-terminal --tab -- bash -c "./rbd_ros_master.exp; exec bash -i"
sleep 10s  #two ROS nodes can interfere with eachother if they are startet at the same time

#launch local ROS
gnome-terminal --tab -- bash -c "roslaunch rbd_gestrec_py rbd_local.launch"
sleep 1s

#launch gesture recognition
gnome-terminal --tab -- bash -c "source ~/anaconda3/etc/profile.d/conda.sh;
				 conda activate robodog;
				/home/juri/anaconda3/envs/robodog/bin/python /home/juri/git/rbd_packages/rbd_local/zhaw_ba_robodog/gestrec.py;
				conda activate base"
#wait for lidar
sleep 15s
#launch gesture recognition
gnome-terminal --tab -- bash -c "source ~/anaconda3/etc/profile.d/conda.sh;
				 conda activate robodog;
				/home/juri/anaconda3/envs/robodog/bin/python /home/juri/git/rbd_packages/rbd_local/GUI/gui.py;
				conda activate base"
