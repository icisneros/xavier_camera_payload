#include <ros/ros.h>
#include <sensor_msgs/fill_image.h>
#include <opencv2/opencv.hpp>
#include "Error.h"
#include "Thread.h"

#include <Argus/Argus.h>
//#include <Argus/Ext/SensorTimestampTsc.h>
#include <EGLStream/EGLStream.h>
#include <EGLStream/NV/ImageNativeBuffer.h>
#include <cv_bridge/cv_bridge.h>

#include <nvbuf_utils.h>
#include <NvEglRenderer.h>
//#include <NvJpegEncoder.h>

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

#include <nvidia_to_ros/nv_server_client.h>
#include <nvidia_to_ros/nv2ros.h>

#include <NvVideoEncoder.h>

using namespace Argus;
using namespace EGLStream;

ros::Publisher image_pub1, image_pub2;
cv_bridge::CvImage img_bridge;
bool visualize = false;

void multi_callback(std::vector<nv2ros::NvImageMessage> msgs){
  // copy and resize the image
  nv2ros::NvImageMessage test_image(msgs[0].get_width()/2, msgs[0].get_height()/2, msgs[0].get_stamp());
  NvBufferRect rect;
  rect.top = 0;
  rect.left = 0;
  rect.width = test_image.get_width();
  rect.height = test_image.get_height();
  NvBufferTransformParams transform_params{0};
  transform_params.transform_flag = NVBUFFER_TRANSFORM_FLIP;
  transform_params.transform_flip = NvBufferTransform_None;
  transform_params.transform_filter = NvBufferTransform_Filter_Nearest;
  transform_params.src_rect = rect;
  transform_params.dst_rect = rect;
  msgs[0].transform(test_image, &transform_params);
  void* test_pdata = NULL;
  test_image.mem_map(&test_pdata);
  test_image.mem_sync_for_cpu(&test_pdata);

  if(visualize){
    void* pdata = NULL;
    msgs[0].mem_map(&pdata);
    msgs[0].mem_sync_for_cpu(&pdata);
    
    cv::Mat img = cv::Mat(test_image.get_height(), test_image.get_width(), CV_8UC4, test_pdata, test_image.get_pitch());
    //cv::Mat img = cv::Mat(msgs[0].get_height(), msgs[0].get_width(), CV_8UC4, pdata, msgs[0].get_pitch());
    sensor_msgs::Image img_msg;
    std_msgs::Header header;
    header.stamp = ros::Time::now();
    img_bridge = cv_bridge::CvImage(header, sensor_msgs::image_encodings::RGBA8, img);
    img_bridge.toImageMsg(img_msg);
    image_pub1.publish(img_msg);

    msgs[0].mem_unmap(&pdata);
  }
  /*
  if(visualize){
    void* pdata2 = NULL;
    msgs[1].mem_map(&pdata2);
    msgs[1].mem_sync_for_cpu(&pdata2);
    
    cv::Mat img2 = cv::Mat(msgs[1].params.height[0], msgs[1].params.width[0], CV_8UC4, pdata2, msgs[1].params.pitch[0]);
    sensor_msgs::Image img_msg2;
    std_msgs::Header header2;
    header2.stamp = ros::Time::now();
    img_bridge = cv_bridge::CvImage(header2, sensor_msgs::image_encodings::RGBA8, img2);
    img_bridge.toImageMsg(img_msg2);
    image_pub2.publish(img_msg2);
    
    msgs[1].mem_unmap(&pdata2);
  }
  //*/
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
  for(int i = 0; i < input_topics.size(); i++)
    ROS_INFO_STREAM("input topics: " << input_topics[i]);
  
  for(int i = 0; i < output_topics.size(); i++)
    ROS_INFO_STREAM("output topics: " << output_topics[i]);
  
  image_pub1 = nh.advertise<sensor_msgs::Image>(output_topics[0], 1);
  //image_pub2 = nh.advertise<sensor_msgs::Image>(output_topics[1], 1);
  nv2ros::Subscriber sub(nh, input_topics, 1, multi_callback);

  ros::spin();
  
  return 0;
}
