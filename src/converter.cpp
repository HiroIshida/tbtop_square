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

/*
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
*/

sensor_msgs::PointCloud2 read_file()
{
  string path = ros::package::getPath("vase_icp");
  string str;
  ifstream ifs(path + "/model/vase.csv");
  vector<double> x_vec, y_vec, z_vec;
  while(getline(ifs, str)){
    double x, y, z;
    sscanf(str.data(), "%lf, %lf, %lf", &x, &y, &z);
    x_vec.push_back(x);
    y_vec.push_back(y);
    z_vec.push_back(z);
  }

  pcl::PointCloud<pcl::PointXYZ> cloud_pcl;
  cloud_pcl.width = x_vec.size();
  cloud_pcl.height = 1;
  cloud_pcl.is_dense = false;
  cloud_pcl.resize(x_vec.size());
  for(int i=0; i<x_vec.size(); i++){
    pcl::PointXYZ pt(x_vec[i], y_vec[i], z_vec[i]);
    cloud_pcl.points[i] = pt;
  }

  sensor_msgs::PointCloud2 msg_pc2;
  pcl::toROSMsg(cloud_pcl, msg_pc2);
  return msg_pc2;
}



int main (int argc, char** argv)
{
  ros::init (argc, argv, "pcl_center");
  ros::NodeHandle nh;

  ros::Publisher pub;
  pub = nh.advertise<sensor_msgs::PointCloud2>("output", 1);
  auto msg_pc2 = read_file();
  ros::Rate loop_rate(10);
  while(ros::ok()){
    pub.publish(msg_pc2);
    loop_rate.sleep();
  }
  return 0;
}


