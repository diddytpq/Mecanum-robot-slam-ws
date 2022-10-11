# Mecanum Slam Workspace

## 1. Dependency

### 1.1. pcl ver  
	https://ropiens.tistory.com/65
### 1.2 GTSAM
	https://github.com/diddytpq/LIO-SAM/tree/noetic

### 1.3 Ros dep
	rosdep install --from-paths ~/your_workspace/src --ignore-src -r -y

## 2. check usb port and port permission 
	ls /dev/ttyUSB* 
	sudo chmod 666 /dev/ttyACM* # 0. vesc, 1. joy 2. imu 
	
## 3. Record topic && play rosbag
	rosbag record /velodyne_points /tf /imu/data
	roscore
	rosparam set /use_sim_time true
	
	rosbag play *.bag --clock --pause 
	rosbag play *.bag --clock --pause --topic /velodyne_points /tf /imu/data --start 0

## 4. Run lio sam
	roslaunch lio_sam run.launch 

## 5. Run localization based lio sam
 *https://github.com/Gaochao-hit/LIO-SAM_based_relocalization

	roslaunch lio_sam_localization run.launch map_name:="mj_part_A"
	
## 6. real robot version

	roslaunch lio_sam run_real_robot.launch
	roslaunch lio_sam_localization run_real_robot.launch map_name:="mj_part_A"
	roslaunch far_planner run_real_robot.launch map_name:="mj_part_A"

## 7. gazebo simulation

	roslaunch mecanum_robot_gazebo mecanum_velodyne.launch

	roslaunch lio_sam run.launch
	roslaunch lio_sam_localization run_relocalize.launch
	roslaunch vehicle_simulator system_real_robot.launch 
	roslaunch far_planner far_planner.launch

## 8. view PCL MAP
 *https://pcl.gitbook.io/tutorial/appendix/visualization-tools

	pcl_viewer -fc 255,255,255 -ps 1 -multiview 1 [파일명]

## trouble shooting
 *cv_bridge Error
	https://jstar0525.tistory.com/118
	
 *fatal error: pcap.h: No such file or directory
	sudo apt-get update && sudo apt-get install libpcap-dev
	
 *fatal error: pcap.h: No such file or directory
 	Failed to find libusb

## convert pcd to ply
	https://fabconvert.com/convert/pcd/to/ply
