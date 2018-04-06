#include "ros/ros.h"
#include "vision_manip_pipeline/Conv2DTo3D.h"
#include "sensor_msgs/PointCloud2.h"

// global variable for point cloud
sensor_msgs::PointCloud2 pCloud;

void callback(const sensor_msgs::PointCloud2 pc){
  // set global point cloud to pc from kinect subscriber
  pCloud = pc;
}

bool convert2Dto3D(const sensor_msgs::PointCloud2 pCloud,
  vision_manip_pipeline::Conv2DTo3D::Request &req,
  vision_manip_pipeline::Conv2DTo3D::Response &res){

  // get width and height of 2D point cloud data
  int width = pCloud.width;
  int height = pCloud.height;

  // Convert from u (column / width), v (row/height) to position in array
  // where X,Y,Z data starts
  int arrayPosition = req.x*pCloud.row_step + req.y*pCloud.point_step;

  ROS_INFO("request: x=%ld, y=%ld", (long int)req.x, (long int)req.y);
  // compute position in array where x,y,z data start
  int arrayPosX = arrayPosition + pCloud.fields[0].offset; // X has an offset of 0
  int arrayPosY = arrayPosition + pCloud.fields[1].offset; // Y has an offset of 4
  int arrayPosZ = arrayPosition + pCloud.fields[2].offset; // Z has an offset of 8

  float X = 0.0;
  float Y = 0.0;
  float Z = 0.0;

  memcpy(&X, &pCloud.data[arrayPosX], sizeof(float));
  memcpy(&Y, &pCloud.data[arrayPosY], sizeof(float));
  memcpy(&Z, &pCloud.data[arrayPosZ], sizeof(float));

  res.newX = X;
  res.newY = Y;
  res.newZ = Z;

  ROS_INFO("solution: x=%f, y=%f, z=%f", (float)res.newX, (float)res.newY, (float)res.newZ);

  return true;
}

bool handle_conv_coord(vision_manip_pipeline::Conv2DTo3D::Request &req,
  vision_manip_pipeline::Conv2DTo3D::Response &res){

    convert2Dto3D(pCloud, req, res);
    return true;
}

int main(int argc, char** argv){
  ros::init(argc, argv, "conv_coord");
  ros::NodeHandle n;

  // subscribe to kinect point cloud
  ros::Subscriber sub = n.subscribe("/camera/depth_registered/points", 1000, callback);
  ros::ServiceServer service = n.advertiseService("conv_coord", handle_conv_coord);

  ROS_INFO("Ready to convert points.");
  ros::spin();

  return 0;
}
