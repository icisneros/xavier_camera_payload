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





void multi_callback(std::vector<nv2ros::NvImageMessage> msgs){

  for(int i = 0; i < msgs.size(); i++){
    void* pdata = NULL;
    msgs[i].mem_map(&pdata);
    msgs[i].mem_sync_for_cpu(&pdata);
    
    cv::Mat img = cv::Mat(msgs[i].get_height(), msgs[i].get_width(), CV_8UC4, pdata, msgs[i].get_pitch());
    
    cv::Mat img_rgb8; // ivan
    cv::cvtColor(img, img_rgb8, CV_BGRA2BGR); //ivan

    sensor_msgs::Image img_msg;
    std_msgs::Header header;
    header.stamp = msgs[i].get_stamp();
    header.frame_id = "camera" + std::to_string(i);  //ivan
    img_bridge = cv_bridge::CvImage(header, sensor_msgs::image_encodings::RGB8, img_rgb8);  //ivan
    
    img_bridge.toImageMsg(img_msg);
    image_pubs[i].publish(img_msg);
  
    msgs[i].mem_unmap(&pdata);
  }

}



/*
Ivan
Rate limiter (limits to approx. half the normal framerate)
Modifications over the original "nv_consumer.cpp":
  0) Publishes every other frame, cutting the fps to half of the original framerate (typically 24fps -> 12fps).
      This helps us produce smaller rosbags. Also, we don't need super fast framerate for localization.
  1) Publishes an rgb8 rather than rgba8 image. We don't need the alpha channel anyway.
*/

// bool publish_this_frame = true;  //ivan

// void multi_callback(std::vector<nv2ros::NvImageMessage> msgs){
  
//   if (publish_this_frame){    //ivan
//     for(int i = 0; i < msgs.size(); i++){
//       void* pdata = NULL;
//       msgs[i].mem_map(&pdata);
//       msgs[i].mem_sync_for_cpu(&pdata);
      
//       cv::Mat img = cv::Mat(msgs[i].get_height(), msgs[i].get_width(), CV_8UC4, pdata, msgs[i].get_pitch());
      
//       cv::Mat img_rgb8; // ivan
//       cv::cvtColor(img, img_rgb8, CV_BGRA2BGR); //ivan

//       sensor_msgs::Image img_msg;
//       std_msgs::Header header;
//       header.stamp = msgs[i].get_stamp();
//       header.frame_id = "camera" + std::to_string(i);  //ivan
//       img_bridge = cv_bridge::CvImage(header, sensor_msgs::image_encodings::RGB8, img_rgb8);  //ivan
      
//       img_bridge.toImageMsg(img_msg);
//       image_pubs[i].publish(img_msg);
    
//       msgs[i].mem_unmap(&pdata);
//     }
//     publish_this_frame = false;
//   }
//   else{
//     publish_this_frame = true;
//   }
// }




// void multi_callback(std::vector<nv2ros::NvImageMessage> msgs){

//   for(int i = 0; i < msgs.size(); i++){
//     void* pdata = NULL;
//     msgs[i].mem_map(&pdata);
//     msgs[i].mem_sync_for_cpu(&pdata);
    
//     cv::Mat img = cv::Mat(msgs[i].get_height(), msgs[i].get_width(), CV_8UC4, pdata, msgs[i].get_pitch());
    
//     cv::Mat img_rgb8; // ivan
//     cv::cvtColor(img, img_rgb8, CV_BGRA2BGR); //ivan

//     sensor_msgs::Image img_msg;
//     std_msgs::Header header;
//     header.stamp = msgs[i].get_stamp();
//     img_bridge = cv_bridge::CvImage(header, sensor_msgs::image_encodings::RGB8, img_rgb8);  //ivan
    
//     img_bridge.toImageMsg(img_msg);
//     image_pubs[i].publish(img_msg);
  
//     msgs[i].mem_unmap(&pdata);
//   }
// }




int main(int argc, char * argv[])
{
  ros::init(argc, argv, "nv_consumer");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

  visualize = pnh.param("visualize", false);
  std::vector<std::string> input_topics, output_topics;
  input_topics = pnh.param("input_topics", input_topics);
  output_topics = pnh.param("output_topics", output_topics);

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
