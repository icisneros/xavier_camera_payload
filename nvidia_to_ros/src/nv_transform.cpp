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

std::vector<ros::Publisher> image_pubs;
cv_bridge::CvImage img_bridge;
bool visualize = false;

int new_width;
int new_height;
int crop_src_rect_top;
int crop_src_rect_left;
int crop_src_rect_width;
int crop_src_rect_height;
int crop_dst_rect_top;
int crop_dst_rect_left;
int crop_dst_rect_width;
int crop_dst_rect_height;
int flip;
int filter;
bool convert_to_gray;

NvBufferTransform_Filter filter_type;
NvBufferTransform_Flip flip_type;
NvBufferTransform_Flag flag;

void multi_callback(std::vector<nv2ros::NvImageMessage> msgs){
  for(int i = 0; i < msgs.size(); i++){
    // copy and resize the image
    int new_image_width = new_width >= 0 ? new_width : msgs[i].get_width();
    int new_image_height = new_height >= 0 ? new_height : msgs[i].get_height();
    static NvBufferColorFormat color_format = convert_to_gray ? NvBufferColorFormat_GRAY8 : NvBufferColorFormat_ABGR32;
    static nv2ros::NvImageMessage test_image(new_image_width, new_image_height, msgs[i].get_stamp(), color_format);
    
    NvBufferRect src_rect;
    src_rect.top = crop_src_rect_top >= 0 ? crop_src_rect_top : 0;
    src_rect.left = crop_src_rect_left >= 0 ? crop_src_rect_left : 0;
    src_rect.width = crop_src_rect_width >= 0 ? crop_src_rect_width : msgs[i].get_width();
    src_rect.height = crop_src_rect_height >= 0 ? crop_src_rect_height : msgs[i].get_height();
    
    NvBufferRect dst_rect;
    dst_rect.top = crop_dst_rect_top >= 0 ? crop_dst_rect_top : 0;
    dst_rect.left = crop_dst_rect_left >= 0 ? crop_dst_rect_left : 0;
    dst_rect.width = crop_dst_rect_width >= 0 ? crop_dst_rect_width : test_image.get_width();
    dst_rect.height = crop_dst_rect_height >= 0 ? crop_dst_rect_height : test_image.get_height();

    ROS_INFO_STREAM("convert to gray: " << convert_to_gray);
    ROS_INFO_STREAM("new width: " << new_image_width << " new height: " << new_image_height);
    ROS_INFO_STREAM("src rect: " << src_rect.top << ", " << src_rect.left << ", " << src_rect.width << ", " << src_rect.height);
    ROS_INFO_STREAM("dst rect: " << dst_rect.top << ", " << dst_rect.left << ", " << dst_rect.width << ", " << dst_rect.height);
    ROS_INFO_STREAM("flip type: " << flip_type);
    ROS_INFO_STREAM("filter type: " << filter_type);
    ROS_INFO_STREAM("flag: " << flag);
    
    NvBufferTransformParams transform_params{0};
    transform_params.transform_flag = flag;
    transform_params.transform_flip = flip_type;
    transform_params.transform_filter = filter_type;
    transform_params.src_rect = src_rect;
    transform_params.dst_rect = dst_rect;
    msgs[i].transform(test_image, &transform_params);
    void* test_pdata = NULL;
    test_image.mem_map(&test_pdata);
    test_image.mem_sync_for_cpu(&test_pdata);
    
    cv::Mat img;
    if(convert_to_gray)
      img = cv::Mat(test_image.get_height(), test_image.get_width(), CV_8UC1, test_pdata, test_image.get_pitch());
    else
      img = cv::Mat(test_image.get_height(), test_image.get_width(), CV_8UC4, test_pdata, test_image.get_pitch());
    
    sensor_msgs::Image img_msg;
    std_msgs::Header header;
    header.stamp = msgs[i].get_stamp();
    if(convert_to_gray)
      img_bridge = cv_bridge::CvImage(header, sensor_msgs::image_encodings::MONO8, img);
    else
      img_bridge = cv_bridge::CvImage(header, sensor_msgs::image_encodings::RGBA8, img);
    
    img_bridge.toImageMsg(img_msg);
    image_pubs[i].publish(img_msg);
    
    test_image.mem_unmap(&test_pdata);
  }
}

int main(int argc, char * argv[])
{
  ros::init(argc, argv, "nv_consumer");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

  visualize = pnh.param("visualize", false);
  std::vector<std::string> input_topics, output_topics;
  input_topics = pnh.param("input_topics", input_topics);
  output_topics = pnh.param("output_topics", output_topics);
  
  new_width = pnh.param("new_width", -1);
  new_height = pnh.param("new_height", -1);
  crop_src_rect_top = pnh.param("crop_src_rect_top", -1);
  crop_src_rect_left = pnh.param("crop_src_rect_left", -1);
  crop_src_rect_width = pnh.param("crop_src_rect_width", -1);
  crop_src_rect_height = pnh.param("crop_src_rect_height", -1);
  crop_dst_rect_top = pnh.param("crop_dst_rect_top", -1);
  crop_dst_rect_left = pnh.param("crop_dst_rect_left", -1);
  crop_dst_rect_width = pnh.param("crop_dst_rect_width", -1);
  crop_dst_rect_height = pnh.param("crop_dst_rect_height", -1);
  flip = pnh.param("flip", -1);
  filter = pnh.param("filter", -1);
  convert_to_gray = pnh.param("convert_to_gray", false);

  int temp_flag = 0;
  
  if(flip >= 0){
    temp_flag |= NVBUFFER_TRANSFORM_FLIP;
    flip_type = (NvBufferTransform_Flip)flip;
  }
  else
    flip_type = NvBufferTransform_None;
  
  if(filter >= 0){
    temp_flag |= NVBUFFER_TRANSFORM_FILTER;
    filter_type = (NvBufferTransform_Filter)filter;
  }
  else
    filter_type = NvBufferTransform_Filter_Nearest;
  
  if(crop_src_rect_top >= 0 || crop_src_rect_left >= 0 ||
     crop_src_rect_width >= 0 || crop_src_rect_height >= 0)
    temp_flag |= NVBUFFER_TRANSFORM_CROP_SRC;
  if(crop_dst_rect_top >= 0 || crop_dst_rect_left >= 0 ||
     crop_dst_rect_width >= 0 || crop_dst_rect_height >= 0)
    temp_flag |= NVBUFFER_TRANSFORM_CROP_DST;
  
  flag = (NvBufferTransform_Flag)temp_flag;
  
  if(input_topics.size() != output_topics.size()){
    ROS_ERROR_STREAM("input_topics is not the same length as output_topics. Exiting.");
    return 0;
  }
  
  for(int i = 0; i < output_topics.size(); i++)
    image_pubs.push_back(nh.advertise<sensor_msgs::Image>(output_topics[i], 1));
  
  nv2ros::Subscriber sub(nh, input_topics, 1, multi_callback);

  ros::spin();
  
  return 0;
}
