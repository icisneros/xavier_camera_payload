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

std::ofstream fout;
bool first = true;

void multi_callback(std::vector<nv2ros::NvImageMessage> msgs){
  void* pdata = NULL;
  msgs[0].mem_map(&pdata);
  msgs[0].mem_sync_for_cpu(&pdata);
  
  if(first){
    first = false;
    int width = msgs[0].get_width();
    int height = msgs[0].get_height();
    int pitch = msgs[0].get_pitch();
    fout.write((char*)&width, sizeof(width));
    fout.write((char*)&height, sizeof(height));
    fout.write((char*)&pitch, sizeof(pitch));

    //std::cout << "first: " << width << ", " << height << ", " << pitch << std::endl;
  }

  ros::Time stamp = msgs[0].get_stamp();
  fout.write((char*)&stamp.sec, sizeof(stamp.sec));
  fout.write((char*)&stamp.nsec, sizeof(stamp.nsec));
  //std::cout << "image: " << stamp << std::endl;
  
  fout.write((char*)pdata, msgs[0].get_height()*msgs[0].get_pitch());
  
  msgs[0].mem_unmap(&pdata);
}

int main(int argc, char * argv[])
{
  ros::init(argc, argv, "nv_log");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");
  
  std::vector<std::string> input_topics;
  std::string input_topic = pnh.param("input_topic", std::string(""));
  input_topics.push_back(input_topic);
  std::string filename = pnh.param("filename", std::string(""));

  fout.open(filename, std::ios::binary | std::ios::out);
  
  nv2ros::Subscriber sub(nh, input_topics, 1, multi_callback);

  while(ros::ok())
    ros::spinOnce();

  fout.close();
  
  return 0;
}
