#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <boost/foreach.hpp>

#include <tf/transform_broadcaster.h>


#include "pcl_ros/transforms.h"
#include <boost/foreach.hpp>

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;


tf::TransformBroadcaster *broadcaster;


void callback(const PointCloud::ConstPtr& pcl_in)
{

  geometry_msgs::TransformStamped pcl_trans;
  pcl_trans.header.frame_id = "world";
  pcl_trans.child_frame_id = "pcl";
    
  // update transform
  // (moving in a circle with radius=2)
  pcl_trans.header.stamp = ros::Time::now();
    

  //send the transform
  broadcaster->sendTransform(pcl_trans);
  /*printf ("Cloud: width = %d, height = %d\n", pcl_in->width, pcl_in->height);
  BOOST_FOREACH (const pcl::PointXYZ& pt, pcl_in->points)
    printf ("\t(%f, %f, %f)\n", pt.x, pt.y, pt.z);*/
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "sub_pcl");
  ros::NodeHandle nh;
  ros::Subscriber sub = nh.subscribe<PointCloud>("points2", 1, callback);
  broadcaster = new tf::TransformBroadcaster(); 
  
  
  ros::spin();
  delete broadcaster;
  return 0;
}

//https://robotics.wtb.tue.nl/svn/ros/code/tue-ros-pkg/dev/tue_executioner_dev/tue_open_cabinet_dev/src/handle_publisher.cpp
