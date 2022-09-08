#include <math.h>
#include <time.h>
#include <stdio.h>
#include <stdlib.h>
#include <ros/ros.h>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <std_msgs/Float32.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PolygonStamped.h>
#include <sensor_msgs/PointCloud2.h>

#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>

using namespace std;

const double PI = 3.1415926;

string stateEstimationTopic = "/integrated_to_init";
string registeredScanTopic = "/velodyne_cloud_registered";
bool flipStateEstimation = true;
bool flipRegisteredScan = true;
bool sendTF = true;
bool reverseTF = false;

float vehicleX = 0;
float vehicleY = 0;
float vehicleZ = 0;
float vehicleRoll = 0;
float vehiclePitch = 0;
float vehicleYaw = 0;

float odomX = 0;
float odomY = 0;
float odomZ = 0;
float odomRoll = 0;
float odomPitch = 0;
float odomYaw = 0;

float terrainZ = 0;
float terrainRoll = 0;
float terrainPitch = 0;

pcl::PointCloud<pcl::PointXYZI>::Ptr laserCloud(new pcl::PointCloud<pcl::PointXYZI>());

nav_msgs::Odometry odomData;
geometry_msgs::PoseStamped maptoodomData;

tf::StampedTransform odomTrans;
ros::Publisher *pubOdometryPointer = NULL;
tf::TransformBroadcaster *tfBroadcasterPointer = NULL;
ros::Publisher *pubLaserCloudPointer = NULL;

void maptoodometryHandler(const geometry_msgs::PoseStamped::ConstPtr& maptoodom)

{
  double roll, pitch, yaw;
  geometry_msgs::Quaternion geoQuat = maptoodom->pose.orientation;
  maptoodomData = *maptoodom;

  tf::Matrix3x3(tf::Quaternion(geoQuat.x, geoQuat.y, geoQuat.z, geoQuat.w)).getRPY(roll, pitch, yaw);

  odomX = maptoodom->pose.position.x;
  odomY = maptoodom->pose.position.y;
  odomZ = maptoodom->pose.position.z;
  odomRoll = roll;
  odomPitch = pitch;
  odomYaw = yaw;

}


void odometryHandler(const nav_msgs::Odometry::ConstPtr& odom)
{
  double roll, pitch, yaw;
  geometry_msgs::Quaternion geoQuat = odom->pose.pose.orientation;
  odomData = *odom;

  tf::Matrix3x3(tf::Quaternion(geoQuat.x, geoQuat.y, geoQuat.z, geoQuat.w)).getRPY(roll, pitch, yaw);

  vehicleX = odomX + odom->pose.pose.position.x;
  vehicleY = odomY + odom->pose.pose.position.y;
  vehicleZ = odomZ + odom->pose.pose.position.z;
  vehicleRoll = odomRoll + roll;
  vehiclePitch = odomPitch + pitch;
  vehicleYaw = odomYaw + yaw;

  // vehicleX = odom->pose.pose.position.x;
  // vehicleY = odom->pose.pose.position.y;
  // vehicleZ = odom->pose.pose.position.z;
  // vehicleRoll =  roll;
  // vehiclePitch =  pitch;
  // vehicleYaw =  yaw;

  // publish odometry messages
  odomData.header.frame_id = "odom";
  odomData.child_frame_id = "sensor";
  pubOdometryPointer->publish(odomData);

  // publish tf messages
  odomTrans.stamp_ = odom->header.stamp;
  odomTrans.frame_id_ = "odom";
  odomTrans.child_frame_id_ = "sensor";
  odomTrans.setRotation(tf::Quaternion(geoQuat.x, geoQuat.y, geoQuat.z, geoQuat.w));
  odomTrans.setOrigin(tf::Vector3(odomData.pose.pose.position.x, odomData.pose.pose.position.y, odomData.pose.pose.position.z));

  if (sendTF) {
    if (!reverseTF) {
      tfBroadcasterPointer->sendTransform(odomTrans);
    } else {
      tfBroadcasterPointer->sendTransform(tf::StampedTransform(odomTrans.inverse(), odom->header.stamp, "sensor", "map"));
    }
  }
}

void laserCloudHandler(const sensor_msgs::PointCloud2ConstPtr& laserCloudIn)
{
  laserCloud->clear();
  pcl::fromROSMsg(*laserCloudIn, *laserCloud);

  if (flipRegisteredScan) {
    int laserCloudSize = laserCloud->points.size();
    for (int i = 0; i < laserCloudSize; i++) {
      float temp = laserCloud->points[i].x;
      laserCloud->points[i].x = laserCloud->points[i].z;
      laserCloud->points[i].z = laserCloud->points[i].y;
      laserCloud->points[i].y = temp;
    }
  }

  float vehicleRecX = odomX + vehicleX;
  float vehicleRecY = odomY + vehicleY;
  float vehicleRecZ = odomZ + vehicleZ;
  float vehicleRecRoll = odomRoll + vehicleRoll;
  float vehicleRecPitch = odomPitch + vehiclePitch;
  float vehicleRecYaw = odomYaw + vehicleYaw;
  float terrainRecRoll = terrainRoll;
  float terrainRecPitch = terrainPitch;

  float sinTerrainRecRoll = sin(terrainRecRoll);
  float cosTerrainRecRoll = cos(terrainRecRoll);
  float sinTerrainRecPitch = sin(terrainRecPitch);
  float cosTerrainRecPitch = cos(terrainRecPitch);

  int scanDataSize = laserCloud->points.size();
  for (int i = 0; i < scanDataSize; i++)
  {
    // float pointX1 = laserCloud->points[i].x;
    // float pointY1 = laserCloud->points[i].y * cosTerrainRecRoll - laserCloud->points[i].z * sinTerrainRecRoll;
    // float pointZ1 = laserCloud->points[i].y * sinTerrainRecRoll + laserCloud->points[i].z * cosTerrainRecRoll;

    // float pointX2 = pointX1 * cosTerrainRecPitch + pointZ1 * sinTerrainRecPitch;
    // float pointY2 = pointY1;
    // float pointZ2 = -pointX1 * sinTerrainRecPitch + pointZ1 * cosTerrainRecPitch;

    float pointX2 = laserCloud->points[i].x * cos(vehicleRecYaw) - laserCloud->points[i].y * sin(vehicleRecYaw);
    float pointY2 = laserCloud->points[i].x * sin(vehicleRecYaw) + laserCloud->points[i].y * cos(vehicleRecYaw);;
    float pointZ2 = laserCloud->points[i].z;


    float pointX3 = pointX2 + vehicleRecX;
    float pointY3 = pointY2 + vehicleRecY;
    float pointZ3 = pointZ2 + vehicleRecZ;


    laserCloud->points[i].x = pointX3;
    laserCloud->points[i].y = pointY3;
    laserCloud->points[i].z = pointZ3;
  }

  // publish registered scan messages
  sensor_msgs::PointCloud2 laserCloud2;
  pcl::toROSMsg(*laserCloud, laserCloud2);
  laserCloud2.header.stamp = laserCloudIn->header.stamp;
  laserCloud2.header.frame_id = "odom";
  pubLaserCloudPointer->publish(laserCloud2);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "loamInterface");
  ros::NodeHandle nh;
  ros::NodeHandle nhPrivate = ros::NodeHandle("~");

  nhPrivate.getParam("stateEstimationTopic", stateEstimationTopic);
  nhPrivate.getParam("registeredScanTopic", registeredScanTopic);
  nhPrivate.getParam("flipStateEstimation", flipStateEstimation);
  nhPrivate.getParam("flipRegisteredScan", flipRegisteredScan);
  nhPrivate.getParam("sendTF", sendTF);
  nhPrivate.getParam("reverseTF", reverseTF);

  // ros::Subscriber subMaptoOdometry = nh.subscribe<geometry_msgs::PoseStamped> ("/lio_sam/mapping/pose_odomTo_map", 5, maptoodometryHandler);
  ros::Subscriber subOdometry = nh.subscribe<nav_msgs::Odometry> (stateEstimationTopic, 5, odometryHandler);

  ros::Subscriber subLaserCloud = nh.subscribe<sensor_msgs::PointCloud2> (registeredScanTopic, 5, laserCloudHandler);

  ros::Publisher pubOdometry = nh.advertise<nav_msgs::Odometry> ("/state_estimation", 5);
  pubOdometryPointer = &pubOdometry;

  tf::TransformBroadcaster tfBroadcaster;
  tfBroadcasterPointer = &tfBroadcaster;

  ros::Publisher pubLaserCloud = nh.advertise<sensor_msgs::PointCloud2> ("/registered_scan", 100);
  pubLaserCloudPointer = &pubLaserCloud;

  ros::spin();

  return 0;
}