#include <ros/ros.h>
#include <pcl/PCLPointCloud2.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h> // japanese version

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include "geometry_msgs/Point.h" 
#include <iostream>
#include <vector>

#define COUNTOF(array) (sizeof(array) / sizeof(array[0]))
#define PRINT(somthing) std::cout << somthing << std::endl;

ros::Publisher pub;
void cloud_cb (const sensor_msgs::PointCloud2ConstPtr& msg_input)
{
  // http://docs.pointclouds.org/1.7.2/a01420.html#a89aca82e188e18a7c9a71324e9610ec9
  // tutorial in Japanese is wrong (using depricated header)  
  pcl::PointCloud<pcl::PointXYZ> cloud;
  pcl::fromROSMsg(*msg_input, cloud); 
  float x_sum, y_sum, z_sum;
  x_sum = y_sum = z_sum = 0;
  int i = 0;
  
  // https://vml.sakura.ne.jp/koeda/PCL/tutorials/html/basic_structures.html#basic-structures
  int N_valid = 0;
  for(int i=0; i<cloud.points.size(); i++){
    float x = cloud.points[i].x;
    float y = cloud.points[i].y;
    float z = cloud.points[i].z;
    if(!(std::isnan(x) || std::isnan(y) || std::isnan(z))){
        x_sum += x;
        y_sum += y;
        z_sum += z;
        N_valid++;
    }
  }
  auto point_msgs = geometry_msgs::Point();
  point_msgs.x = x_sum/N_valid;
  point_msgs.y = y_sum/N_valid;
  point_msgs.z = z_sum/N_valid;
  PRINT(z_sum/N_valid);
  pub.publish(point_msgs);
}

int main (int argc, char** argv)
{
  ros::init (argc, argv, "pcl_center");
  ros::NodeHandle nh;
  ros::Subscriber sub = nh.subscribe ("input", 1, cloud_cb);
  pub = nh.advertise<geometry_msgs::Point> ("output", 1);
  ros::spin ();
}

