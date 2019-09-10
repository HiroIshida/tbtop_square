#include <pcl/PCLPointCloud2.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/PointCloud.h>
#include <geometry_msgs/Point32.h>
#include <pcl_conversions/pcl_conversions.h> // japanese version
#include <vase_icp/Projected.h>

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

void callback (const sensor_msgs::PointCloud2ConstPtr& msg_input)
{
  // http://docs.pointclouds.org/1.7.2/a01420.html#a89aca82e188e18a7c9a71324e9610ec9
  // tutorial in Japanese is wrong (using depricated header)  
  pcl::PointCloud<pcl::PointXYZ> cloud;
  pcl::fromROSMsg(*msg_input, cloud); 
  int N = cloud.points.size();

  std_msgs::Float32MultiArray x_array, y_array;
  x_array.data.resize(N);
  y_array.data.resize(N);
  for(int i=0; i< N; i++){
    x_array.data[i] = cloud.points[i].x;
    y_array.data[i] = cloud.points[i].y;
  }
  vase_icp::Projected msg;
  msg.x_array = x_array;
  msg.y_array = y_array;
  pub.publish(msg);
}

int main (int argc, char** argv)
{
  ros::init(argc, argv, "projector");
  ros::NodeHandle nh;
  pub = nh.advertise<vase_icp::Projected>("output", 1);
  ros::Subscriber sub = nh.subscribe("input", 1000, callback);
  ros::spin();

}

