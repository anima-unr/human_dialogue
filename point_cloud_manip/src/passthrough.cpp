#include "ros/ros.h"
#include "std_msgs/String.h"
#include <tf/transform_listener.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>

// global variable for point cloud
sensor_msgs::PointCloud2 pCloud;
sensor_msgs::PointCloud2 pCloud_out;
sensor_msgs::PointCloud2 pCloud_filtered;
tf::TransformListener* listener;
ros::Publisher pub;

// https://answers.ros.org/question/205236/transforming-point-cloud/
void callback(const sensor_msgs::PointCloud2ConstPtr& pc){
  // set global point cloud to pc from kinect subscriber
  //ROS_WARN("AHHHHHx3!!!");
  // transform the point cloud
  tf::StampedTransform transform;
  try{
    // listener->lookupTransform( "/test", "/camera_depth_optical_frame",
    listener->lookupTransform( "/test", "/camera_rgb_optical_frame",
             ros::Time(0), transform);
    }
  catch (tf::TransformException ex){
      ROS_ERROR("%s",ex.what());
      ros::Duration(1.0).sleep();
  }

  // transform cloud
  pcl_ros::transformPointCloud("/test", *pc, pCloud_out, *listener);

  // add passthrough filter
  pcl::PCLPointCloud2* cloud = new pcl::PCLPointCloud2;
  pcl::PCLPointCloud2ConstPtr cloudPtr(cloud);
  
  pcl::PCLPointCloud2* tmp = new pcl::PCLPointCloud2;
  pcl::PCLPointCloud2ConstPtr tmpPtr(tmp);

  pcl_conversions::toPCL(pCloud_out, *cloud);
  pcl::PassThrough<pcl::PCLPointCloud2> passx;
  passx.setInputCloud (cloudPtr);
  passx.setFilterFieldName ("x");
  passx.setFilterLimits (0.0, 1.0);
  passx.filter(*tmp);
  
  pcl::PassThrough<pcl::PCLPointCloud2> passz;
  passz.setInputCloud(tmpPtr);
  passz.setFilterFieldName ("z");
  passz.setFilterLimits (-2.0, -1.0);
  passz.filter(*tmp);
  
  pcl::PassThrough<pcl::PCLPointCloud2> passy;
  passy.setInputCloud(tmpPtr);
  passy.setFilterFieldName ("y");
  passy.setFilterLimits (0.0, 1.0);
  passy.filter(*tmp);

  // publish the point cloud
  pub.publish(*tmp);
  printf(".");
  fflush(stdout);
}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "passthrough_filter");
  ros::NodeHandle n;
  ros::AsyncSpinner a(4);

  listener = new tf::TransformListener();
  pub = n.advertise<sensor_msgs::PointCloud2>("/local/depth_registered/trans_points", 1000);

  // subscribe to point cloud
  ros::Subscriber sub = n.subscribe("points", 1, callback);

  a.start();
  ros::spin();
  return 0;
}
