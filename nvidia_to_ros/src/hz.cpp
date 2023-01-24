#include <ros/ros.h>
#include <sensor_msgs/fill_image.h>
#include <opencv2/opencv.hpp>

#include <sensor_msgs/Image.h>
#include <sensor_msgs/CompressedImage.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/SetCameraInfo.h>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>

#include <stdio.h>
#include <stdlib.h>
#include <sstream>
#include <fstream>
#include <iostream>
#include <fcntl.h>

#include <nvidia_to_ros/nv2ros.h>

cv_bridge::CvImage img_bridge;
bool visualize = false;
ros::Time start_time;
int count;

void multi_callback(std::vector<nv2ros::NvImageMessage> msgs){
  count++;
  
  ros::Time now = ros::Time::now();
  if((now - start_time).toSec() >= 1.0){
    ROS_INFO_STREAM("image count: " << count);
    start_time = now;
    count = 0;
  }
}

int main(int argc, char * argv[])
{
  ros::init(argc, argv, "nv_consumer");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");
  
  std::vector<std::string> input_topics;
  input_topics = pnh.param("input_topics", input_topics);

  count = 0;
  start_time = ros::Time::now();
  
  nv2ros::Subscriber sub(nh, input_topics, 1, multi_callback);

  ros::spin();
  
  return 0;
}
