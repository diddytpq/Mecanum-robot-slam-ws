# Mecanum Slam Workspace

## 1. Dependency

### 1.1. pcl ver  
	https://ropiens.tistory.com/65
### 1.2 GTSAM
	https://github.com/diddytpq/LIO-SAM/tree/noetic

## 2. check usb port and port permission 
	ls /dev/ttyUSB* 
	sudo chmod 666 /dev/tty*
	
## 3. Record topic
	rosbag record /velodyne_points /tf /imu/data
	roscore
	rosparam set /use_sim_time true
	rosbag play --clock --pause rosbag/*.bag

## 4. Run lio sam (test)
	roslaunch velodyne_pointcloud VLP16_points.launch
	roslaunch ros-ngimu run.launch

	roslaunch lio_sam run.launch 
	rosbag play --clock --pause rosbag/*.bag --topic /velodyne_points /tf /imu/data
	
## 5. Run mecanum_robot_control_joy && mecanum of lio sam 

	roslaunch openrobot_control openrobot_control_6omni.launch
	roslaunch lio_sam run_mecanum.launch


## 6. trouble shooting
* cv_bridge Error
	```
	https://jstar0525.tistory.com/118
	```
* fatal error: pcap.h: No such file or directory
	```
	sudo apt-get update && sudo apt-get install libpcap-dev
	```


