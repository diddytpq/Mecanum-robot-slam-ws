# Mecanum Slam Workspace

## 1. Dependency

### 1.1. pcl ver  
	https://ropiens.tistory.com/65
### 1.2 GTSAM
	https://github.com/diddytpq/LIO-SAM/tree/noetic

## 2. check usb port and port permission 
	ls /dev/ttyUSB* 
	sudo chmod 666 /dev/tty* # 1. vesc, 2. joy 3. imu 
	
## 3. Record topic && play rosbag
	rosbag record /velodyne_points /tf /imu/data
	roscore
	rosparam set /use_sim_time true
	
	rosbag play *.bag --clock --pause 
	rosbag play *.bag --clock --pause --topic /velodyne_points /tf /imu/data --start 0

## 4. Run lio sam
	roslaunch velodyne_pointcloud VLP16_points.launch
	roslaunch ros-ngimu run.launch
	roslaunch lio_sam run.launch 

## 5. Run mecanum_robot_control_joy && mecanum of lio sam 

	roslaunch lio_sam run_real_robot.launch

## 6. view PCL MAP
 *https://pcl.gitbook.io/tutorial/appendix/visualization-tools

	pcl_viewer -fc 255,255,255 -ps 1 -multiview 1 [파일명]

## 7. Run localization based lio sam
 *https://github.com/Gaochao-hit/LIO-SAM_based_relocalization

	roslaunch lio_sam_localization run.launch
	roslaunch lio_sam_localization run_real_robot.launch

## 8. Run Far planner(path planning)

	roslaunch far_planner run_real_robot.launch


## 8. gazebo simulation

	roslaunch mecanum_robot_gazebo mecanum_velodyne.launch

	roslaunch lio_sam run.launch
	roslaunch lio_sam_localization run_relocalize.launch

	roslaunch vehicle_simulator system_real_robot.launch 


## trouble shooting
 *cv_bridge Error
	https://jstar0525.tistory.com/118
	
 *fatal error: pcap.h: No such file or directory
	sudo apt-get update && sudo apt-get install libpcap-dev
	
 *fatal error: pcap.h: No such file or directory
 	Failed to find libusb

## convert pcd to ply
	https://imagetostl.com/kr/convert/file/pcd/to/ply