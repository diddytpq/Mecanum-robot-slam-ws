# Mecanum Slam Workspace

## 1. Dependency

### 1.1. pcl ver  
	https://ropiens.tistory.com/65
### 1.2 GTSAM
	https://github.com/diddytpq/LIO-SAM/tree/noetic

## 2. check usb port and port permission 
	ls /dev/ttyUSB* 
	sudo chmod 666 /dev/tty* # vesc, imu, joy
	
## 3. Record topic && play rosbag
	rosbag record /velodyne_points /tf /imu/data
	roscore
	rosparam set /use_sim_time true
	
	rosbag play --clock --pause rosbag/*.bag
	rosbag play --clock --pause rosbag/*.bag --topic /velodyne_points /tf /imu/data --start 0

## 4. Run lio sam
	roslaunch velodyne_pointcloud VLP16_points.launch
	roslaunch ros-ngimu run.launch
	roslaunch lio_sam run.launch 

	
## 5. Run mecanum_robot_control_joy && mecanum of lio sam 

	roslaunch lio_sam run_mecanum.launch

## 6. view PCL MAP
 *https://pcl.gitbook.io/tutorial/appendix/visualization-tools

	pcl_viewer -fc 255,255,255 -ps 1 -multiview 1 [파일명]

## 7. Run localization based lio sam
 *https://github.com/Gaochao-hit/LIO-SAM_based_relocalization

	roslaunch lio_sam_localization run_relocalize.launch

## 8. trouble shooting
* cv_bridge Error
	```
	https://jstar0525.tistory.com/118
	```
* fatal error: pcap.h: No such file or directory
	```
	sudo apt-get update && sudo apt-get install libpcap-dev
	```


