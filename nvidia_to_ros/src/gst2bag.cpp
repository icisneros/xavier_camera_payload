#include <nvidia_to_ros/nv2ros.h>
#include <cstdlib>
#include <gst/gst.h>
#include <gst/gstinfo.h>
#include <gst/app/gstappsink.h>
#include <glib-unix.h>
#include <dlfcn.h>
#include <opencv2/videoio.hpp>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>  //ivan
#include <cv_bridge/cv_bridge.h>

#include <iostream>
#include <sstream>
#include <thread>
#include <stdlib.h>
#include <libgen.h>

#include <rosbag/bag.h>

#include "nvbuf_utils.h"


using namespace std;

#define USE(x) ((void)(x))

static GstPipeline *gst_pipeline = nullptr;
static string launch_string;
static int frame_count = 0;
static int sleep_count = 0;
static int eos = 0;
//static NvEglRenderer *renderer;
nv2ros::Publisher* pub;
cv_bridge::CvImage img_bridge;
ros::Time start_time;
std::string output_topic;
rosbag::Bag bag;

static void appsink_eos(GstAppSink * appsink, gpointer user_data)
{
  printf("app sink receive eos\n");
  eos = 1;
  //    g_main_loop_quit (hpipe->loop);
}

static GstFlowReturn new_buffer(GstAppSink *appsink, gpointer user_data)
{
  GstSample *sample = NULL;

  g_signal_emit_by_name (appsink, "pull-sample", &sample,NULL);

  if (sample)
    {
      GstBuffer *buffer = NULL;
      GstCaps   *caps   = NULL;
      GstMapInfo map    = {0};
      int dmabuf_fd = 0;

      caps = gst_sample_get_caps (sample);
      if (!caps)
        {
	  printf("could not get snapshot format\n");
        }
      gst_caps_get_structure (caps, 0);
      buffer = gst_sample_get_buffer (sample);
      gst_buffer_map (buffer, &map, GST_MAP_READ);

      ExtractFdFromNvBuffer((void *)map.data, &dmabuf_fd);
      printf("dmabuf_fd %d %lu %lu %lu %lu %lu\n", dmabuf_fd, buffer->pts, buffer->dts, buffer->duration,
	     buffer->offset, buffer->offset_end);
      NvBufferParams buf_params;
      NvBufferGetParams (dmabuf_fd, &buf_params);
      NvBufferParamsEx buf_params_ex;
      NvBufferGetParamsEx(dmabuf_fd, &buf_params_ex);

      ROS_INFO_STREAM("params: " << nv2ros::params_to_string(buf_params));


      ros::Time current_stamp = start_time + ros::Duration(buffer->pts/1000000000, buffer->pts%1000000000);
      ROS_INFO_STREAM("CURRENT TIME: " << current_stamp.sec << ", " << current_stamp.nsec);

      nv2ros::NvImageMessage nv_img(dmabuf_fd, buf_params, buf_params_ex, current_stamp);
      static nv2ros::NvImageMessage nv_rgba(nv_img.get_width(), nv_img.get_height(), current_stamp);
      NvBufferRect rect;
      rect.top = 0;
      rect.left = 0;
      rect.width = nv_img.get_width();
      rect.height = nv_img.get_height();
      NvBufferTransformParams transform_params{0};
      transform_params.transform_flag = NVBUFFER_TRANSFORM_FLIP;
      transform_params.transform_flip = NvBufferTransform_None;
      transform_params.transform_filter = NvBufferTransform_Filter_Nearest;
      transform_params.src_rect = rect;
      transform_params.dst_rect = rect;
      nv_img.transform(nv_rgba, &transform_params);
      
      void* pdata = NULL;
      nv_rgba.mem_map(&pdata);
      nv_rgba.mem_sync_for_cpu(&pdata);
      cv::Mat img = cv::Mat(nv_rgba.get_height(), nv_rgba.get_width(), CV_8UC4, pdata, nv_rgba.get_pitch());
      cv::Mat img_rgb8; // ivan
      cv::cvtColor(img, img_rgb8, CV_BGRA2BGR); //ivan
      sensor_msgs::Image img_msg;
      std_msgs::Header header;
      header.stamp = current_stamp;
      img_bridge = cv_bridge::CvImage(header, sensor_msgs::image_encodings::RGB8, img_rgb8); //ivan
      img_bridge.toImageMsg(img_msg);

      bag.write(output_topic, img_msg.header.stamp, img_msg);
      
      //pub->publish(dmabuf_fd, buf_params, buf_params_ex, current_stamp);
      
      //renderer->render(dmabuf_fd);
      //NvReleaseFd(dmabuf_fd);
      frame_count++;

      gst_buffer_unmap(buffer, &map);

      gst_sample_unref (sample);
    }
  else
    {
      g_print ("could not make snapshot\n");
    }

  return GST_FLOW_OK;
}

ros::Time get_start_time(std::string filename){
  filename = basename(strdup(filename.c_str()));
  ROS_INFO_STREAM("filename: " << filename);
  std::stringstream ss(filename);
  string name;
  std::getline(ss, name, '.');
  ROS_INFO_STREAM("name: " << name);

  std::vector<std::string> tokens(1);
  std::stringstream ss2(name);
  while(getline(ss2, tokens.back(), '_')){
    ROS_INFO_STREAM("token: " << tokens[tokens.size()-1]);
    tokens.push_back("");
  }
  
  return ros::Time(std::atoi(tokens[tokens.size()-3].c_str()),
		   std::atoi(tokens[tokens.size()-2].c_str()));
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "gst2nv");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");
  std::vector<std::string> topic_names;
  topic_names.push_back("nv_image1");
  pub = new nv2ros::Publisher(nh, topic_names, 4);
  //std::string filename = "/home/airlab/ws/src/nvidia_to_ros/src/a.mp4";
  //std::string filename = "/home/airlab/ws/src/nvidia_to_ros/src/Bourne_Trailer.mp4";
  std::string filename = pnh.param("input_filename", std::string(""));
  output_topic = pnh.param("output_topic", std::string("image"));
  std::string output_bag_filename = pnh.param("output_bag_filename", std::string(""));
  bag.open(output_bag_filename, rosbag::bagmode::Write);

  start_time = get_start_time(filename);
  
  
  gst_init (&argc, &argv);

  GMainLoop *main_loop;
  main_loop = g_main_loop_new (NULL, FALSE);
  ostringstream launch_stream;
  int w = 1920;
  int h = 816;
  GstAppSinkCallbacks callbacks = {appsink_eos, NULL, new_buffer};

  cv::VideoCapture cap(filename);
  cv::Mat mat;
  cap.read(mat);
  w = mat.cols;
  h = mat.rows;

  ROS_INFO_STREAM("rows: " << mat.rows << " cols: " << mat.cols << " time: " << cap.get(cv::CAP_PROP_POS_MSEC));

  launch_stream
    //    << "nvcamerasrc ! "
    //    << "video/x-raw(memory:NVMM), width="<< w <<", height="<< h <<", framerate=30/1 ! " 
    //<< "filesrc location=/home/airlab/ws/src/nvidia_to_ros/src/Bourne_Trailer.mp4 ! decodebin ! "
    << "filesrc location=" << filename << " ! decodebin ! "
    << "nvvidconv ! "
    << "video/x-raw(memory:NVMM), format=I420, width="<< w <<", height="<< h <<" ! "
    << "appsink name=mysink ";

  launch_string = launch_stream.str();

  g_print("Using launch string: %s\n", launch_string.c_str());

  GError *error = nullptr;
  gst_pipeline  = (GstPipeline*) gst_parse_launch(launch_string.c_str(), &error);

  if (gst_pipeline == nullptr) {
    g_print( "Failed to parse launch: %s\n", error->message);
    return -1;
  }
  if(error) g_error_free(error);

  GstElement *appsink_ = gst_bin_get_by_name(GST_BIN(gst_pipeline), "mysink");
  gst_app_sink_set_callbacks (GST_APP_SINK(appsink_), &callbacks, NULL, NULL);

  //renderer = NvEglRenderer::createEglRenderer("renderer0",
  //					      w, h, 0, 0);
//renderer->setFPS(24);

  gst_element_set_state((GstElement*)gst_pipeline, GST_STATE_PLAYING); 

  while (eos == 0) {
    sleep(1);
    sleep_count++;
  }
  //sleep(90);
  //g_main_loop_run (main_loop);

  gst_element_set_state((GstElement*)gst_pipeline, GST_STATE_NULL);
  gst_object_unref(GST_OBJECT(gst_pipeline));
  g_main_loop_unref(main_loop);

//delete renderer;

  g_print("going to exit, decode %d frames in %d seconds \n", frame_count, sleep_count);
  return 0;
}
