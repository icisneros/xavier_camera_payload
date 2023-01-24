#include <ros/ros.h>
#include <sensor_msgs/fill_image.h>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>

#include <nvbuf_utils.h>
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

nv2ros::NvImageMessage* image1;
nv2ros::NvImageMessage* image2;
cv::Mat img, img2;
void* pdata;
void* pdata2;
nv2ros::Params p, p2;
nv2ros::Publisher* pub;

int width, height;
int circle_x, circle_y, circle_radius, circle_vel_x, circle_vel_y;
cv::Vec4b circle_color, background_color1, background_color2;

void timer_callback(const ros::TimerEvent& e){
  circle(img, cv::Point(circle_x, circle_y), circle_radius, background_color1, cv::FILLED);
  circle(img2, cv::Point(circle_x, circle_y), circle_radius, background_color2, cv::FILLED);

  circle_x += circle_vel_x;
  if(circle_x-circle_radius < 0 || circle_x+circle_radius >= width)
    circle_vel_x *= -1;
  circle_y += circle_vel_y;
  if(circle_y-circle_radius < 0 || circle_y+circle_radius >= height)
    circle_vel_y *= -1;
  
  cv::circle(img, cv::Point(circle_x, circle_y), circle_radius, circle_color, cv::FILLED);
  cv::circle(img2, cv::Point(circle_x, circle_y), circle_radius, circle_color, cv::FILLED);
  
  image1->mem_sync_for_device(&pdata);
  image2->mem_sync_for_device(&pdata2);
  
  ros::Time stamp = ros::Time::now();
  image1->set_stamp(stamp);
  image2->set_stamp(stamp);
  pub->publish(*image1, 0);
  pub->publish(*image2, 1);
}

int main(int argc, char * argv[])
{
  ros::init(argc, argv, "nv_producer");
  ros::NodeHandle nh;
  
  width = 1280;
  height = 960;

  circle_x = width/4;
  circle_y = height/2;
  circle_radius = 20;
  circle_vel_x = 2;
  circle_vel_y = 2;

  circle_color[0] = 0;
  circle_color[1] = 0;
  circle_color[2] = 255;
  circle_color[3] = 255;
  
  background_color1[0] = 255;
  background_color1[1] = 0;
  background_color1[2] = 0;
  background_color1[3] = 255;
  
  background_color2[0] = 0;
  background_color2[1] = 255;
  background_color2[2] = 0;
  background_color2[3] = 255;

  image1 = new nv2ros::NvImageMessage(width, height, ros::Time::now());
  image2 = new nv2ros::NvImageMessage(width, height, ros::Time::now());
  pdata = NULL;
  pdata2 = NULL;
  
  image1->mem_map(&pdata);
  image1->mem_sync_for_cpu(&pdata);
  
  image2->mem_map(&pdata2);
  image2->mem_sync_for_cpu(&pdata2);

  img = cv::Mat(height, width, CV_8UC4, pdata, p.params.pitch[0]);
  img2 = cv::Mat(height, width, CV_8UC4, pdata2, p.params.pitch[0]);
  for(int x = 0; x < img.cols; x++){
    for(int y = 0; y < img.rows; y++){
      cv::Vec4b& color = img.at<cv::Vec4b>(y,x);
      color = background_color1;

      cv::Vec4b& color2 = img2.at<cv::Vec4b>(y,x);
      color2 = background_color2;
    }
  }

  image1->mem_sync_for_device(&pdata);
  image2->mem_sync_for_device(&pdata2);

  std::vector<std::string> topic_names;
  topic_names.push_back("nv_image1");
  topic_names.push_back("nv_image2");
  pub = new nv2ros::Publisher(nh, topic_names);

  sleep(1);
  
  ros::Timer timer = nh.createTimer(ros::Duration(1.0/30.0), timer_callback);

  ros::spin();
  
  return 0;
}
