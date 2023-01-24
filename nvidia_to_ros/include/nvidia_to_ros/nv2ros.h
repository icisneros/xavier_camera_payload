#include <opencv2/opencv.hpp>
#include <ros/ros.h>
#include <sensor_msgs/fill_image.h>
#include <nvbuf_utils.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CompressedImage.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/SetCameraInfo.h>
#include <sensor_msgs/image_encodings.h>
#include <std_msgs/ByteMultiArray.h>

#include <stdio.h>
#include <stdlib.h>
#include <sstream>
#include <fstream>
#include <iostream>
#include <fcntl.h>
#include <algorithm>
#include <string>
#include <functional>
#include <unordered_map>
#include <map>

#include "nv_server_client.h"

#ifdef NON_JETSON
#include <nvidia_to_ros/ImageAndCameraInfo.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#endif

namespace nv2ros {

  // debug logging
  enum Color{black=0, red, green, yellow, blue, magenta, cyan, white, unused, none};
  enum Mod{reset=0, bold=1, underline=4};

  std::string color(Color foreground=none, Mod modifier=reset, Color background=none){
    int fg_color = (int)foreground + 30;
    int bg_color = (int)background + 40;
    int mod_color = (int)modifier;

    return std::string("\033[") + std::to_string(mod_color) +  ";" +
      std::to_string(fg_color) + ";" +
      std::to_string(bg_color) + "m";
  }

  
  class Subscriber;

  std::string params_to_string(const NvBufferParams& params);
  
  enum PubSubMessageTypes {IMAGE_REQUEST, IMAGE_RESPONSE, IMAGE_TRANSFER_STATUS};
  
  struct NvImageInfo {
    ros::Time stamp;
    int fd_id, image_id;
    NvBufferParams params;
    NvBufferParamsEx params_ex;

    NvImageInfo()
      : stamp(0, 0)
      , fd_id(0)
      , image_id(0){

    }
  };



  struct ImageRequest : public Serializable {
    int fd_id, image_id;
    bool send_fd;

    std::vector<uint8_t> serialize() override;
    void deserialize(std::vector<uint8_t> bytes) override;
  };

  struct ImageResponse : public Serializable  {
    bool valid;
    
    std::vector<uint8_t> serialize() override;
    void deserialize(std::vector<uint8_t> bytes) override;
  };

  struct ImageTransferStatus : public Serializable  {
    bool complete;
    int fd_id;
    
    std::vector<uint8_t> serialize() override;
    void deserialize(std::vector<uint8_t> bytes) override;
  };

  // TODO remove
  struct Params {
    NvBufferParams params;
    NvBufferParamsEx params_ex;
  };


  // TODO remove
  struct ParamsResponse {
    bool success;
  };

  // ================================================================================
  // ------------------------------ NvImageMessage ----------------------------------
  // ================================================================================
  
  class NvImageMessage {
  private:
  public:
    int dmabuf_fd;
    NvBufferParams params;
    NvBufferParamsEx params_ex;
    ros::Time stamp;
    sensor_msgs::CameraInfo camera_info;

    #ifdef NON_JETSON
    nvidia_to_ros::ImageAndCameraInfo image_and_info;
  private:
    #endif

    NvImageMessage(int dmabuf_fd, NvBufferParams params, NvBufferParamsEx params_ex, ros::Time stamp);

    #ifdef NON_JETSON
    friend class Subscriber;
  public:
    #endif
    
    NvImageMessage(int width, int height, ros::Time stamp,
		   NvBufferColorFormat color_format=NvBufferColorFormat_ABGR32,
		   NvBufferLayout layout=NvBufferLayout_Pitch,
		   NvBufferPayloadType payload_type=NvBufferPayload_SurfArray,
		   NvBufferTag tag=NvBufferTag_NONE);
    ~NvImageMessage();

    void mem_map(void** pointer);
    void mem_unmap(void** pointer);
    void mem_sync_for_cpu(void** pointer);
    void mem_sync_for_device(void** pointer);
    void transform(NvImageMessage& destination, NvBufferTransformParams* transform_params);
    int get_width();
    int get_height();
    int get_pitch();
    NvBufferParams get_params();
    NvBufferParamsEx get_params_ex();
    ros::Time get_stamp();
    
    void hold();
    void release();
    
    int get_dmabuf_fd();

    void set_stamp(ros::Time stamp);
  };
  
  void check(int status, std::string function_name);
  int new_nv_buffer(int width, int height);


  // ================================================================================
  // ------------------------------ PublisherQueue ----------------------------------
  // ================================================================================

  struct FileDescriptor {
    int fd;
    int id;
  };
  
  struct QueueImage {
    ros::Time stamp;
    FileDescriptor fd;
    int image_id;
    int users;

    QueueImage();
  };

  struct QueueElement {
    ros::Time stamp;
    std::vector<QueueImage> images;
    
    QueueElement(int image_count, ros::Time stamp);
    bool is_complete();
    bool operator<(const QueueElement& e) const;
    bool operator==(const QueueElement& e) const;
  };
  
  class PublisherQueue {
  private:
    std::vector<FileDescriptor> available_fds;
    int next_fd_id;
    int next_image_id;
    int queue_size;
    int image_count;
    
    std::list<QueueElement> queue;
    std::mutex queue_mutex;
    std::unordered_map<int, int> id_to_fd;
    std::mutex id_to_fd_mutex;
    
    void prune_queue_elements();
    QueueElement& get_queue_element(ros::Time stamp);
    FileDescriptor get_available_fd(NvBufferParams params);

    void print();
    
  public:
    PublisherQueue(int queue_size, int image_count);
    bool add(int dmabuf_fd, NvBufferParams params, NvBufferParamsEx params_ex, ros::Time stamp, int index,
	     NvImageInfo* info);
    int get_fd(int fd_id);
    bool check_valid_and_lock(int fd_id, int image_id);
    void unlock(int fd_id);
    // TODO add checks for 0 return values where they are called
  };
  
  // ================================================================================
  // -------------------------------- Publisher -------------------------------------
  // ================================================================================
  
  class Publisher {
  private:
    static Server* server;
    std::vector<ros::Publisher> pubs;
    int queue_size;
    PublisherQueue queue;

    void message_callback(Message message, int cfd);
    void publish(int dmabuf_fd, NvBufferParams params, NvBufferParamsEx params_ex, ros::Time stamp, int index=0);
    
  public:
    Publisher(ros::NodeHandle nh, std::vector<std::string> topic_names, int queue_size=1);
    Publisher(ros::NodeHandle nh, std::string topic_name, int queue_size=1);
    void publish(NvImageMessage& image, int index=0);
  };

  // ================================================================================
  // -------------------------------- Subscriber ------------------------------------
  // ================================================================================  

  class SubscriberQueue {
  private:
    int queue_size;
    int images;

    std::map<ros::Time, std::vector<NvImageInfo> > queue;
    
  public:
    SubscriberQueue(){}
    SubscriberQueue(int queue_size, int images);
    bool add(NvImageInfo info, int index);
    std::vector<NvImageInfo> get(ros::Time stamp);
    void clear();
  };
  
  class Subscriber {
  private:
    SubscriberQueue queue;
    std::vector<std::string> topic_names;
    std::function<void(std::vector<NvImageMessage>)> callback_function;
    Client* client;
    std::unordered_map<int, int> fds;

    std::vector<ros::Subscriber> subs;

    void reset();

    void callback(const std_msgs::ByteMultiArray::ConstPtr& msg, int index);

    void single_callback_function(std::function<void(NvImageMessage)> func, std::vector<NvImageMessage> messages);

    #ifdef NON_JETSON
    void one_callback(const nvidia_to_ros::ImageAndCameraInfoConstPtr& msg);
    void two_callback(const nvidia_to_ros::ImageAndCameraInfoConstPtr& msg1, const nvidia_to_ros::ImageAndCameraInfoConstPtr& msg2);
    void three_callback(const nvidia_to_ros::ImageAndCameraInfoConstPtr& msg1, const nvidia_to_ros::ImageAndCameraInfoConstPtr& msg2,
			const nvidia_to_ros::ImageAndCameraInfoConstPtr& msg3);
    void four_callback(const nvidia_to_ros::ImageAndCameraInfoConstPtr& msg1, const nvidia_to_ros::ImageAndCameraInfoConstPtr& msg2,
		       const nvidia_to_ros::ImageAndCameraInfoConstPtr& msg3, const nvidia_to_ros::ImageAndCameraInfoConstPtr& msg4);
    void five_callback(const nvidia_to_ros::ImageAndCameraInfoConstPtr& msg1, const nvidia_to_ros::ImageAndCameraInfoConstPtr& msg2,
		       const nvidia_to_ros::ImageAndCameraInfoConstPtr& msg3, const nvidia_to_ros::ImageAndCameraInfoConstPtr& msg4,
		       const nvidia_to_ros::ImageAndCameraInfoConstPtr& msg5);
    void six_callback(const nvidia_to_ros::ImageAndCameraInfoConstPtr& msg1, const nvidia_to_ros::ImageAndCameraInfoConstPtr& msg2,
		      const nvidia_to_ros::ImageAndCameraInfoConstPtr& msg3, const nvidia_to_ros::ImageAndCameraInfoConstPtr& msg4,
		      const nvidia_to_ros::ImageAndCameraInfoConstPtr& msg5, const nvidia_to_ros::ImageAndCameraInfoConstPtr& msg6);
    #endif
    
  public:
    Subscriber(ros::NodeHandle nh, std::vector<std::string> topic_names, int queue_size,
	       std::function<void(std::vector<NvImageMessage>)> callback_function);
    Subscriber(ros::NodeHandle nh, std::vector<std::string> topic_names, int queue_size, void(*fp)(std::vector<NvImageMessage>));
    template<class T>
    Subscriber(ros::NodeHandle nh, std::vector<std::string> topic_names, int queue_size,
	       void(T::*fp)(std::vector<NvImageMessage>), T* object)
      : Subscriber(nh, topic_names, queue_size, std::bind(fp, object, std::placeholders::_1)){
      
    }
    
    Subscriber(ros::NodeHandle nh, std::string topic_name, int queue_size, std::function<void(NvImageMessage)> callback_function);
    Subscriber(ros::NodeHandle nh, std::string topic_name, int queue_size, void(*fp)(NvImageMessage));
    template<class T>
    Subscriber(ros::NodeHandle nh, std::string topic_name, int queue_size, void(T::*fp)(NvImageMessage), T* object)
      : Subscriber(nh, topic_name, queue_size, std::bind(fp, object, std::placeholders::_1)){
      
    }

    Subscriber(ros::NodeHandle nh, std::string topic_name, int queue_size,
	       std::function<void(std::vector<NvImageMessage>)> callback_function);
    Subscriber(ros::NodeHandle nh, std::string topic_name, int queue_size, void(*fp)(std::vector<NvImageMessage>));
    template<class T>
    Subscriber(ros::NodeHandle nh, std::string topic_name, int queue_size,
	       void(T::*fp)(std::vector<NvImageMessage>), T* object)
      : Subscriber(nh, topic_name, queue_size, std::bind(fp, object, std::placeholders::_1)){
      
    }
  };
  
}
