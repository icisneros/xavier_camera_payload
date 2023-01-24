#include <ros/ros.h>
#include <sensor_msgs/fill_image.h>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>

// add the camerainfo
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CompressedImage.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/SetCameraInfo.h>
#include <sensor_msgs/image_encodings.h>

#include <stdio.h>
#include <stdlib.h>
#include <sstream>
#include <fstream>
#include <iostream>
#include <fcntl.h>
#include <algorithm>
#include <string>

#include <nvidia_to_ros/nv2ros.h>

nv2ros::NvImageMessage* image = NULL;
cv::Mat img, img2;
int dmabuf_fd, dmabuf_fd2;
void* pdata;
void* pdata2;
nv2ros::Params p, p2;
nv2ros::Publisher* pub;


void pub_image(cv::Mat& img){
  static ros::NodeHandle nh;
  static ros::Publisher pub = nh.advertise<sensor_msgs::Image>("hist_image", 1);
  std_msgs::Header header;
  header.stamp = ros::Time::now();
  cv_bridge::CvImage img_bridge = cv_bridge::CvImage(header, sensor_msgs::image_encodings::RGBA8, img);
  sensor_msgs::Image image;
  img_bridge.toImageMsg(image);
  pub.publish(image);

}


void img_callback(const sensor_msgs::ImageConstPtr &img_msg){
  cv_bridge::CvImageConstPtr ptr;
  ptr = cv_bridge::toCvCopy(img_msg, sensor_msgs::image_encodings::RGBA8);
  cv::Mat img = ptr->image;
  //pub_image(img);
  //ROS_INFO_STREAM("rows: " << img.rows << " cols: " << img.cols << " step: " << img.step << " size: " << img.size);

  if(image == NULL)
    image = new nv2ros::NvImageMessage(img.cols, img.rows, img_msg->header.stamp);
  image->set_stamp(img_msg->header.stamp);

  //ROS_INFO_STREAM("image pitch: " << image->get_pitch());

  void* data = NULL;
  image->mem_map(&data);
  image->mem_sync_for_cpu(&data);

  //std::memcpy(data, img.data, img.cols*img.rows*4);
  cv::Mat i = cv::Mat(image->get_height(), image->get_width(), CV_8UC4, data, image->get_pitch());
  img.copyTo(i);
  
  image->mem_sync_for_device(&data);
  image->mem_unmap(&data);

  pub->publish(*image);
  
}

int main(int argc, char * argv[]){
  ros::init(argc, argv, "ros2nv");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

  std::string output_topic = pnh.param("output_topic", std::string("output"));

  ros::Subscriber sub = nh.subscribe("image", 10, img_callback);
  
  std::vector<std::string> topic_names;
  topic_names.push_back(output_topic);
  pub = new nv2ros::Publisher(nh, topic_names);

  ros::spin();
  
  return 0;
}
