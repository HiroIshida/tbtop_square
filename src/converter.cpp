#include <ros/ros.h>
#include <ros/package.h>
#include <pcl/PCLPointCloud2.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/PointCloud.h>
#include <geometry_msgs/Point32.h>
#include <pcl_conversions/pcl_conversions.h> // japanese version

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include "geometry_msgs/Point.h" 
#include <iostream>
#include <fstream>
#include <vector>
#include <string>

#define COUNTOF(array) (sizeof(array) / sizeof(array[0]))
#define PRINT(somthing) std::cout << somthing << std::endl;
using namespace std;

ros::Publisher pub;
void cloud_cb (const sensor_msgs::PointCloud2ConstPtr& msg_input)
{
  // http://docs.pointclouds.org/1.7.2/a01420.html#a89aca82e188e18a7c9a71324e9610ec9
  // tutorial in Japanese is wrong (using depricated header)  
  pcl::PointCloud<pcl::PointXYZ> cloud;
  pcl::fromROSMsg(*msg_input, cloud); 
  sensor_msgs::PointCloud msg_pc;
  msg_pc.header = msg_input->header;
  for(int i = 0; i < cloud.points.size(); i++){
    geometry_msgs::Point32 pt;
    pt.x = cloud.points[i].x;
    pt.y = cloud.points[i].y;
    pt.z = cloud.points[i].z;
    msg_pc.points.push_back(pt);
  }
  pub.publish(msg_pc);
}

int main (int argc, char** argv)
{
  ros::init (argc, argv, "pcl_center");
  ros::NodeHandle nh;
  ros::Subscriber sub = nh.subscribe("/kinect_head/depth_registered/points", 1, cloud_cb);
  pub = nh.advertise<sensor_msgs::PointCloud>("/kinect_converted", 1);
  ros::spin ();
}


