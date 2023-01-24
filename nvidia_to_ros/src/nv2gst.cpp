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

// https://forums.developer.nvidia.com/t/opencv-gpu-mat-into-gstreamer-without-downloading-to-cpu/186703/14
// TODO merge above and below includes
#include <cstdlib>
#include <gst/gst.h>
#include <gst/gstinfo.h>
#include <gst/app/gstappsrc.h>
#include <glib-unix.h>
#include <dlfcn.h>

#include <cstring>
#include <iostream>
#include <sstream>
#include <thread>
#include <string.h>

#include "nvbuf_utils.h"
#include <cuda.h>
#include <cuda_runtime.h>
#include <cudaEGL.h>


using namespace Argus;
using namespace EGLStream;

static GstPipeline *gst_pipeline = nullptr;
static std::string launch_string;
static GstElement *appsrc_;
GMainLoop *main_loop;

bool inited = false;
GstClockTime timestamp = 0;
std::string filename;
static int w = 1224;
static int h = 1028;
//static int w = 2448;//1224;
//static int h = 2048;//1028;
EGLDisplay egl_display;
static FILE *fp;
static guint size;

static void
notify_to_destroy (gpointer user_data)
{
    GST_INFO ("NvBufferDestroy(%d)", *(int *)user_data);
    //NvBufferDestroy(*(int *)user_data);
    g_free(user_data);
}

int dmabuf_fd;
NvBufferParams par;
void* nv_data_pointer;
ros::Time current_stamp;

static gboolean feed_function(gpointer d) {
  static ros::Time current_time = ros::Time::now();
  static ros::Time first_time = current_stamp;
  ros::Time now = ros::Time::now();
  //ROS_INFO_STREAM("time diff: " << (now - current_time).toSec());
  current_time = now;
  //ROS_INFO_STREAM("feed function");
    GstBuffer *buffer;
    GstFlowReturn ret;
    GstMapInfo map = {0};
    //int dmabuf_fd = msgs[0].get_dmabuf_fd();
    gpointer data = NULL, user_data = NULL;
    //NvBufferParams par = msgs[0].get_params();
    GstMemoryFlags flags = (GstMemoryFlags)0;
    void *ptr;

    /*
    NvBufferCreate(&dmabuf_fd, w, h, NvBufferLayout_Pitch, NvBufferColorFormat_ABGR32);
    NvBufferMemMap(dmabuf_fd, 0, NvBufferMem_Read_Write, &ptr);
    NvBufferMemSyncForCpu(dmabuf_fd, 0, &ptr);
    //memset(ptr, 255, size);
    //fread(ptr, size, 1, fp);
    NvBufferMemSyncForDevice(dmabuf_fd, 0, &ptr);
    NvBufferMemUnMap(dmabuf_fd, 0, &ptr);
    NvBufferGetParams (dmabuf_fd, &par);
    ROS_INFO_STREAM("FEED payload type: " << par.payloadType << " pixel: " << par.pixel_format);
    /*/
    
    //CUDA process
    /*
    {
        EGLImageKHR egl_image;
	ROS_INFO("1");
        egl_image = NvEGLImageFromFd(egl_display, dmabuf_fd);
	ROS_INFO("2");
        CUresult status;
        CUeglFrame eglFrame;
        CUgraphicsResource pResource = NULL;
        cudaFree(0);
        status = cuGraphicsEGLRegisterImage(&pResource,
                    egl_image,
                    CU_GRAPHICS_MAP_RESOURCE_FLAGS_NONE);
        if (status != CUDA_SUCCESS)
        {
            printf("cuGraphicsEGLRegisterImage failed: %d \n",status);
        }
        status = cuGraphicsResourceGetMappedEglFrame(&eglFrame, pResource, 0, 0);
        status = cuCtxSynchronize();


        status = cuCtxSynchronize();
        status = cuGraphicsUnregisterResource(pResource);
        NvDestroyEGLImage(egl_display, egl_image);
    }
    */
    user_data = g_malloc(sizeof(int));
    GST_INFO ("NvBufferCreate %d", dmabuf_fd);
    *(int *)user_data = dmabuf_fd;
    //NvBufferGetParams (dmabuf_fd, &par);
    data = g_malloc(par.nv_buffer_size);

    buffer = gst_buffer_new_wrapped_full(flags,
                                         data,
                                         par.nv_buffer_size,
                                         0,
                                         par.nv_buffer_size,
                                         user_data,
                                         notify_to_destroy);
    buffer->pts = (current_stamp - first_time).toNSec();//timestamp;
    //buffer->dts = 100;//timestamp;
    //buffer->duration = 101;//current_stamp.nsec;
    //buffer->offset = 102;
    //buffer->offset_end = 103;

    gst_buffer_map (buffer, &map, GST_MAP_WRITE);
    //memcpy(map.data, par.nv_buffer , par.nv_buffer_size);
    memcpy(map.data, nv_data_pointer , par.nv_buffer_size);
    gst_buffer_unmap(buffer, &map);

    g_signal_emit_by_name (appsrc_, "push-buffer", buffer, &ret);
    gst_buffer_unref(buffer);

    timestamp += 33333333;
    return G_SOURCE_CONTINUE;
}

void init(std::string filename, int width, int height){
  ROS_INFO("2");
  
  main_loop = g_main_loop_new (NULL, FALSE);
  std::ostringstream launch_stream;
  
  ROS_INFO("3");

  egl_display = eglGetDisplay(EGL_DEFAULT_DISPLAY);
  eglInitialize(egl_display, NULL, NULL);
  launch_stream
    << "appsrc name=mysource ! "
    << "video/x-raw(memory:NVMM),width="<< width <<",height="<< height <<",framerate=30/1,format=RGBA ! "  //ivan
    << "nvvidconv ! video/x-raw(memory:NVMM),format=NV12 ! "
    // // original code
    // << "nvv4l2h264enc ! h264parse ! qtmux ! filesink location=" << filename;
    
    // 2448 x 2048 res at 24 fps.   Each frame: 2448*2048*8*3 = 120324096 [bit] = 15.040512 [MB]
    << "nvv4l2h264enc bitrate=150000000 control-rate=1 vbv-size=10000000 ! h264parse ! qtmux ! filesink location=" << filename; //ivan

    // 1224 x 1028 res at 10 fps

    // 640 x 512 res at 10 fps


  ROS_INFO("3");

  launch_string = launch_stream.str();

  g_print("Using launch string: %s\n", launch_string.c_str());

  GError *error = nullptr;
  gst_pipeline  = (GstPipeline*) gst_parse_launch(launch_string.c_str(), &error);

  if (gst_pipeline == nullptr) {
    g_print( "Failed to parse launch: %s\n", error->message);
    exit(1);
    //return -1;
  }
  if(error) g_error_free(error);
  
  ROS_INFO("4");

  appsrc_ = gst_bin_get_by_name(GST_BIN(gst_pipeline), "mysource");
  gst_app_src_set_stream_type(GST_APP_SRC(appsrc_), GST_APP_STREAM_TYPE_STREAM);

  gst_element_set_state((GstElement*)gst_pipeline, GST_STATE_PLAYING); 

  size = (w*h*4);

  ROS_INFO("5");
  
}


void multi_callback(std::vector<nv2ros::NvImageMessage> msgs){
  /*
  dmabuf_fd = msgs[0].get_dmabuf_fd();
  par = msgs[0].get_params();
  ROS_INFO_STREAM("payload type: " << par.payloadType << " pixel: " << par.pixel_format);
  msgs[0].mem_map(&nv_data_pointer);
  msgs[0].mem_sync_for_cpu(&nv_data_pointer);
  msgs[0].mem_sync_for_device(&nv_data_pointer);
  msgs[0].mem_unmap(&nv_data_pointer);
  nv_data_pointer = par.nv_buffer;
  //*/

  //*
  //*
  if(!inited){
    w = msgs[0].get_width();
    h = msgs[0].get_height();
    int index = filename.find_last_of(".");
    std::string fn = filename.substr(0, index) + "_" + std::to_string(msgs[0].get_stamp().sec) + "_" + std::to_string(msgs[0].get_stamp().nsec) + "." + filename.substr(index+1);
    init(fn, msgs[0].get_width(), msgs[0].get_height());
    inited = true;
  }
  
  static nv2ros::NvImageMessage test_image(msgs[0].get_width(), msgs[0].get_height(), msgs[0].get_stamp());
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

  test_image.mem_map(&nv_data_pointer);
  test_image.mem_sync_for_cpu(&nv_data_pointer);
  test_image.mem_sync_for_device(&nv_data_pointer);
  test_image.mem_unmap(&nv_data_pointer);
  //*/

  //*
  dmabuf_fd = test_image.get_dmabuf_fd();
  par = test_image.get_params();
  nv_data_pointer = par.nv_buffer;
  current_stamp = msgs[0].get_stamp();
  //*/
  
  /*
  dmabuf_fd = msgs[0].get_dmabuf_fd();
  par = msgs[0].get_params();
  nv_data_pointer = par.nv_buffer;
  //*/
  //ROS_INFO_STREAM("payload type: " << par.payloadType << " pixel: " << par.pixel_format);
  //*/
  
  feed_function(nullptr);
  
  //msgs[1].mem_unmap(&nv_data_pointer);
}

int main(int argc, char * argv[])
{
  //sleep(2);
  ros::init(argc, argv, "nv2gst");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

  std::string topic = pnh.param("topic", std::string(""));
  
  std::vector<std::string> topic_names;
  topic_names.push_back(topic);
  //topic_names.push_back("nv_image2");
  nv2ros::Subscriber sub(nh, topic_names, 1, multi_callback);

  filename = pnh.param("filename", std::string("output.mp4"));

  ROS_INFO("1");
  
  gst_init (&argc, &argv);

  //init(filename, w, h);
  /*
  ROS_INFO("2");

  GMainLoop *main_loop;
  main_loop = g_main_loop_new (NULL, FALSE);
  std::ostringstream launch_stream;
  
  ROS_INFO("3");

  egl_display = eglGetDisplay(EGL_DEFAULT_DISPLAY);
  eglInitialize(egl_display, NULL, NULL);
  launch_stream
    << "appsrc name=mysource ! "
    << "video/x-raw(memory:NVMM),width="<< w <<",height="<< h <<",framerate=30/1,format=RGBA ! "
    << "nvvidconv ! video/x-raw(memory:NVMM),format=NV12 ! "
    //<< "nvv4l2h264enc ! h264parse ! matroskamux ! filesink location=a.mkv ";
    << "nvv4l2h264enc ! h264parse ! qtmux ! filesink location=" << filename;
  
  ROS_INFO("3");

  launch_string = launch_stream.str();

  g_print("Using launch string: %s\n", launch_string.c_str());

  GError *error = nullptr;
  gst_pipeline  = (GstPipeline*) gst_parse_launch(launch_string.c_str(), &error);

  if (gst_pipeline == nullptr) {
    g_print( "Failed to parse launch: %s\n", error->message);
    return -1;
  }
  if(error) g_error_free(error);
  
  ROS_INFO("4");

  appsrc_ = gst_bin_get_by_name(GST_BIN(gst_pipeline), "mysource");
  gst_app_src_set_stream_type(GST_APP_SRC(appsrc_), GST_APP_STREAM_TYPE_STREAM);

  gst_element_set_state((GstElement*)gst_pipeline, GST_STATE_PLAYING); 

  size = (w*h*4);

  ROS_INFO("5");
  */
    
  ros::spin();
  
  //fp = fopen ("1080.yuv", "rb");
  /*
  for (int i=0; i<150; i++) {
    feed_function(nullptr);
  }
  */
  //fclose(fp);

  // Wait for EOS message
  gst_element_send_event ((GstElement*)gst_pipeline, gst_event_new_eos ());
  GstBus *bus = gst_pipeline_get_bus(GST_PIPELINE(gst_pipeline));
  gst_bus_poll(bus, GST_MESSAGE_EOS, GST_CLOCK_TIME_NONE);

  gst_element_set_state((GstElement*)gst_pipeline, GST_STATE_NULL);
  gst_object_unref(GST_OBJECT(gst_pipeline));
  g_main_loop_unref(main_loop);
  eglTerminate(egl_display);

  g_print("going to exit \n");
  return 0;
}
