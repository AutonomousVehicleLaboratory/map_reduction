#include "ros/ros.h"
#include "std_msgs/String.h"
#include "sensor_msgs/PointCloud2.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Quaternion.h"
#include "tf/transform_datatypes.h"
#include "tf/tf.h"
#include <pcl/filters/crop_box.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>

#define CLIP_X_MAX 60 // max distance in x from pose
#define CLIP_X_MIN 0  // min distance in x from pose
#define CLIP_Y 10     // clip on both size of y, (-10, 10)
#define CLIP_Z 10     // clip on both size of z, (-10, 10)
#define CYCLE 5       // publish cycle, to prevent lag

//#include <pcl/point_cloud.h>

//#include <tf/transform_datatypes.h>
#include <sstream>


ros::Publisher reduced_map_pub;
ros::Subscriber pose_sub; 
ros::Subscriber map_sub; 
ros::Subscriber lidar_sub; 

unsigned int map_width = 0;
unsigned int counter = 0;
pcl::PCLPointCloud2* pcd_map = new pcl::PCLPointCloud2;
pcl::PCLPointCloud2ConstPtr pcd_map_ptr(pcd_map);
pcl::PCLPointCloud2 cloud_filtered;
//geometry_msgs::PoseStamped::ConstPtr vehicle_pose;
pcl::PointXYZ veh_pose(0, 0, 0);
pcl::PointXYZ veh_orient(0, 0, 0);

void pose_callback(const geometry_msgs::PoseStamped::ConstPtr& msg){
  //vehicle_pose = msg;
  veh_pose.x = msg->pose.position.x; 
  veh_pose.y = msg->pose.position.y; 
  veh_pose.z = msg->pose.position.z; 
  tf::Quaternion veh_quat(msg->pose.orientation.x,
                          msg->pose.orientation.y,
                          msg->pose.orientation.z,
                          msg->pose.orientation.w);
  //tf::quaternionMsgToTf(msg->pose.orientation, veh_quat);
  double yaw,pitch,roll;
  tf::Matrix3x3(veh_quat).getRPY(yaw,
                                 pitch,
                                 roll); 
  veh_orient.x = yaw; 
  veh_orient.y = pitch; 
  veh_orient.z = roll; 
}
void map_callback(const sensor_msgs::PointCloud2::ConstPtr& msg){
  if(map_width != msg->width){
    ROS_INFO("Received map.");
    map_width = msg->width;
    pcl_conversions::toPCL(*msg, *pcd_map);

  } 

}
void lidar_callback(const sensor_msgs::PointCloud2::ConstPtr& msg){
  //ROS_INFO("Received lidar data.");
  if(map_width != 0){
    counter += 1;

    // publish at a reduced rate compared to lidar scan
    if (counter % CYCLE != 0){
      return;
    }

    pcl::CropBox<pcl::PCLPointCloud2> crop_filter;
    crop_filter.setInputCloud(pcd_map_ptr);
    crop_filter.setMin(Eigen::Vector4f( CLIP_X_MIN, -CLIP_Y, -CLIP_Z, 0));
    crop_filter.setMax(Eigen::Vector4f( CLIP_X_MAX,  CLIP_Y,  CLIP_Z, 0));
    crop_filter.setTranslation(Eigen::Vector3f(veh_pose.x, 
                                             veh_pose.y, 
                                             veh_pose.z));
     
    crop_filter.setRotation(Eigen::Vector3f(veh_orient.x,
                                            veh_orient.y,
                                            veh_orient.z)); 
    crop_filter.filter(cloud_filtered);
    sensor_msgs::PointCloud2 output_cloud;
    pcl_conversions::fromPCL(cloud_filtered, output_cloud);
    reduced_map_pub.publish(output_cloud);
  }
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "map_reduction");
  ros::NodeHandle n;
  reduced_map_pub = n.advertise<sensor_msgs::PointCloud2>("reduced_map", 1000);
  pose_sub = n.subscribe("/current_pose", 1000, pose_callback); 
  map_sub = n.subscribe("/points_map", 1000, map_callback); 
  lidar_sub = n.subscribe("/points_raw", 1, lidar_callback); 
  ros::spin(); 


  return 0;
}
