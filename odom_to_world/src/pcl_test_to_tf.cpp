#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <boost/foreach.hpp>

#include <tf/transform_listener.h>
#include "pcl_ros/transforms.h"
#include <boost/foreach.hpp>

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

ros::Publisher tf_pub;

tf::TransformListener *tf_listener; 


void callback(const PointCloud::ConstPtr& pcl_in)
{

  PointCloud pcl_out;
   

  tf_listener->waitForTransform("/world", "/pcl", (*pcl_in).header.stamp, ros::Duration(5.0));
  pcl_ros::transformPointCloud("/world", *pcl_in, pcl_out, *tf_listener);
  //tf_pub.publish(pcl_out);



  printf ("Cloud: width = %d, height = %d\n", pcl_in->width, pcl_in->height);
  BOOST_FOREACH (const pcl::PointXYZ& pt, pcl_in->points)
    printf ("\t(%f, %f, %f)\n", pt.x, pt.y, pt.z);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "sub_pcl");
  ros::NodeHandle nh;
  ros::Subscriber sub = nh.subscribe<PointCloud>("points2", 1, callback);
  tf_pub = nh.advertise<PointCloud> ("tf_points2", 1);
  
  tf_listener    = new tf::TransformListener();
  
  ros::spin();
  //delete tf_listener; 
  //return 0; 
}

//https://robotics.wtb.tue.nl/svn/ros/code/tue-ros-pkg/dev/tue_executioner_dev/tue_open_cabinet_dev/src/handle_publisher.cpp
