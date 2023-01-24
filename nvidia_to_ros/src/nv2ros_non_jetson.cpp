#include <nvidia_to_ros/nv2ros.h>
#include <cv_bridge/cv_bridge.h>

namespace nv2ros{

  /*
  std::string get_sensor_msgs_image_encoding(NvBufferColorFormat format){
    if(format == NvBufferColorFormat_ABGR32)
      return sensor_msgs::image_encodings::RGBA8;
    else if(format == NvBufferColorFormat_GRAY8)
      return sensor_msgs::image_encodings::GRAY8;
    else
      ROS_INFO_STREAM("NvBufferColorFormat " << format << " not supported. Add it in get_sensor_msgs_image_encoding"
		      << " in nv2ros_non_jetson.cpp");
    
    return sensor_msgs::image_encodings::GRAY8;
  }
  */
  
  // ================================================================================
  // ------------------------------ NvImageMessage ----------------------------------
  // ================================================================================

  std::string type2str(int type) {
    std::string r;
    
    uchar depth = type & CV_MAT_DEPTH_MASK;
    uchar chans = 1 + (type >> CV_CN_SHIFT);
    
    switch ( depth ) {
    case CV_8U:  r = "8U"; break;
    case CV_8S:  r = "8S"; break;
    case CV_16U: r = "16U"; break;
    case CV_16S: r = "16S"; break;
    case CV_32S: r = "32S"; break;
    case CV_32F: r = "32F"; break;
    case CV_64F: r = "64F"; break;
    default:     r = "User"; break;
    }

    r += "C";
    r += (chans+'0');

    return r;
  }

  std::string cv_to_image_encoding(int type){
    if(type == CV_8UC1)
      return sensor_msgs::image_encodings::MONO8;
    else if(type == CV_8UC4)
      return sensor_msgs::image_encodings::RGBA8;
    else
      ROS_INFO_STREAM("cv mat type " << type << " not supperted. Add support for it in cv_to_image_encoding"
		      << " in nv2ros_non_jetson.cpp");
  }
  
  void set_nv_image_message_mat(NvImageMessage& img, cv::Mat& mat){
    cv_bridge::CvImage img_bridge = cv_bridge::CvImage(img.image_and_info.header, cv_to_image_encoding(mat.type()), mat);
    img_bridge.toImageMsg(img.image_and_info.image);
    
    img.params.width[0] = img.image_and_info.image.width;
    img.params.height[0] = img.image_and_info.image.height;
    img.params.pitch[0] = img.image_and_info.image.step;
  }
  
  // This is used as a way of initializing a blank NvImageMessage so it can be filled in by the Subscriber in the non jetson version
  NvImageMessage::NvImageMessage(int dmabuf_fd, NvBufferParams params, NvBufferParamsEx params_ex, ros::Time stamp)
    : stamp(stamp){
    
  }
  
  NvImageMessage::NvImageMessage(int width, int height, ros::Time stamp,
				 NvBufferColorFormat color_format,
				 NvBufferLayout layout,
				 NvBufferPayloadType payload_type,
				 NvBufferTag tag)
    : stamp(stamp){
    image_and_info.header.stamp = stamp;
    image_and_info.image.header.stamp = stamp;
    image_and_info.image.width = width;
    image_and_info.image.height = height;

    if(color_format == NvBufferColorFormat_ABGR32){
      image_and_info.image.step = image_and_info.image.width*4;
      image_and_info.image.encoding = sensor_msgs::image_encodings::RGBA8;
    }
    else if(color_format == NvBufferColorFormat_GRAY8){
      image_and_info.image.step = image_and_info.image.width;
      image_and_info.image.encoding = sensor_msgs::image_encodings::MONO8;
    }
    else{
      ROS_INFO_STREAM("color_format " << color_format << " not supperted. Add support for it in the NvImageMessage constructor"
		      << " in nv2ros_non_jetson.cpp");
    }
    
    params.width[0] = width;
    params.height[0] = height;
    params.pitch[0] = image_and_info.image.step;
    params.pixel_format = color_format;
    params.payloadType = payload_type;

    image_and_info.image.data.resize(image_and_info.image.step * image_and_info.image.height);
  }
  
  NvImageMessage::~NvImageMessage(){
    
  }
  
  void NvImageMessage::mem_map(void** pointer){
    *pointer = &image_and_info.image.data[0];
  }
  
  void NvImageMessage::mem_unmap(void** pointer){
    *pointer = NULL;
  }
  
  void NvImageMessage::mem_sync_for_cpu(void** pointer){
    // nothing to do
  }
  
  void NvImageMessage::mem_sync_for_device(void** pointer){
    // nothing to do
  }
  
  void NvImageMessage::transform(NvImageMessage& destination, NvBufferTransformParams* transform_params){
    cv_bridge::CvImagePtr ptr = cv_bridge::toCvCopy(image_and_info.image, destination.image_and_info.image.encoding);
    
    // crop the source image first
    cv::Mat cropped;
    if(transform_params->transform_flag & NVBUFFER_TRANSFORM_CROP_SRC)
      cropped = ptr->image(cv::Range(transform_params->src_rect.top,
				     transform_params->src_rect.top + transform_params->src_rect.height),
			   cv::Range(transform_params->src_rect.left,
				     transform_params->src_rect.left + transform_params->src_rect.width));
    else
      cropped = ptr->image;
    
    // next resize, this conversion of nvidia filter flags to opencv interpolation flags is just a guess
    int interpolation = cv::INTER_LINEAR;
    if(transform_params->transform_filter == NvBufferTransform_Filter_Nearest)
      interpolation = cv::INTER_NEAREST;
    else if(transform_params->transform_filter == NvBufferTransform_Filter_Bilinear)
      interpolation = cv::INTER_LINEAR;
    else if(transform_params->transform_filter == NvBufferTransform_Filter_5_Tap)
      interpolation = cv::INTER_AREA;
    else if(transform_params->transform_filter == NvBufferTransform_Filter_10_Tap)
      interpolation = cv::INTER_AREA;
    else if(transform_params->transform_filter == NvBufferTransform_Filter_Smart)
      interpolation = cv::INTER_LANCZOS4;
    else if(transform_params->transform_filter == NvBufferTransform_Filter_Nicest)
      interpolation = cv::INTER_CUBIC;
    cv::Mat resized;
    cv::resize(cropped, resized, cv::Size(destination.get_width(), destination.get_height()), 0, 0, interpolation);
    
    // next flip
    if(transform_params->transform_flag & NVBUFFER_TRANSFORM_FLIP){
      if(transform_params->transform_flip == NvBufferTransform_Rotate90)
	cv::rotate(resized, resized, cv::ROTATE_90_COUNTERCLOCKWISE);
      else if(transform_params->transform_flip == NvBufferTransform_Rotate180)
	cv::rotate(resized, resized, cv::ROTATE_180);
      else if(transform_params->transform_flip == NvBufferTransform_Rotate270)
	cv::rotate(resized, resized, cv::ROTATE_90_CLOCKWISE);
      else if(transform_params->transform_flip == NvBufferTransform_FlipX)
	cv::flip(resized, resized, 1);
      else if(transform_params->transform_flip == NvBufferTransform_FlipY)
	cv::flip(resized, resized, 0);
      else if(transform_params->transform_flip == NvBufferTransform_Transpose){
	cv::flip(resized, resized, 1);
	cv::rotate(resized, resized, cv::ROTATE_90_COUNTERCLOCKWISE);
      }
      else if(transform_params->transform_flip == NvBufferTransform_InvTranspose){
	cv::flip(resized, resized, 1);
	cv::rotate(resized, resized, cv::ROTATE_90_CLOCKWISE);
      }
      
      if(resized.cols != destination.get_width() || resized.rows != destination.get_height())
	cv::resize(resized, resized, cv::Size(destination.get_width(), destination.get_height()), 0, 0, interpolation);
    }
    
    // next do the destination "crop"
    if(transform_params->transform_flag & NVBUFFER_TRANSFORM_CROP_DST){
      cv::Mat new_image(resized.rows, resized.cols, resized.type());
      std::memset(new_image.ptr(), 0, new_image.total()*new_image.elemSize());
      
      cv::resize(resized, resized, cv::Size(transform_params->dst_rect.width, transform_params->dst_rect.height), interpolation);
      cv::Rect roi(transform_params->dst_rect.left, transform_params->dst_rect.top,
		   transform_params->dst_rect.width, transform_params->dst_rect.height);
      resized.copyTo(new_image(roi));
      resized = new_image;
    }
    
    // set the destination image
    set_nv_image_message_mat(destination, resized);
  }
  
  int NvImageMessage::get_width(){
    return params.width[0];
  }
  
  int NvImageMessage::get_height(){
    return params.height[0];
  }
  
  int NvImageMessage::get_pitch(){
    return params.pitch[0];
  }
  
  NvBufferParams NvImageMessage::get_params(){
    return params;
  }
  
  NvBufferParamsEx NvImageMessage::get_params_ex(){
    return params_ex;
  }
  
  ros::Time NvImageMessage::get_stamp(){
    return stamp;
  }
  
  void NvImageMessage::hold(){
    // TODO
  }
  
  void NvImageMessage::release(){
    // TODO
  }
  
  
  int NvImageMessage::get_dmabuf_fd(){
    return -1;
  }
  
  
  void NvImageMessage::set_stamp(ros::Time stamp){
    this->stamp = stamp;
    image_and_info.header.stamp = stamp;
    image_and_info.image.header.stamp = stamp;
  }
  
  // ================================================================================
  // -------------------------------- Publisher -------------------------------------
  // ================================================================================
  
  Publisher::Publisher(ros::NodeHandle nh, std::vector<std::string> topic_names, int queue_size)
    : queue(0, 0)
    , queue_size(queue_size){
    
    for(int i = 0; i < topic_names.size(); i++)
      pubs.push_back(nh.advertise<nvidia_to_ros::ImageAndCameraInfo>(topic_names[i], queue_size));
  }
  
  Publisher::Publisher(ros::NodeHandle nh, std::string topic_name, int queue_size)
    : Publisher(nh, std::vector<std::string>(1, topic_name), queue_size){
    
  }
  
  void Publisher::publish(NvImageMessage& image, int index){
    pubs[index].publish(image.image_and_info);
  }

  PublisherQueue::PublisherQueue(int queue_size, int image_count){
    
  }
  
  
  // ================================================================================
  // -------------------------------- Subscriber ------------------------------------
  // ================================================================================

  SubscriberQueue::SubscriberQueue(int queue_size, int images){
    
  }
  
  Subscriber::Subscriber(ros::NodeHandle nh, std::string topic_name, int queue_size,
			 std::function<void(NvImageMessage)> callback_function)
    : Subscriber(nh, std::vector<std::string>(1, topic_name), queue_size,
		 std::bind(&Subscriber::single_callback_function, this, callback_function, std::placeholders::_1)){
    
  }
  
  Subscriber::Subscriber(ros::NodeHandle nh, std::string topic_name, int queue_size, void(*fp)(NvImageMessage))
    : Subscriber(nh, topic_name, queue_size, std::bind(fp, std::placeholders::_1)){
    
  }
  
  Subscriber::Subscriber(ros::NodeHandle nh, std::vector<std::string> topic_names, int queue_size,
			 std::function<void(std::vector<NvImageMessage>)> callback_function)
    : topic_names(topic_names)
    , queue(queue_size, topic_names.size())
    , callback_function(callback_function){
    std::vector<message_filters::Subscriber<nvidia_to_ros::ImageAndCameraInfo>*> mf_subs;
    
    if(topic_names.size() == 1){
      subs.push_back(nh.subscribe(topic_names[0], queue_size, &Subscriber::one_callback, this));
    }
    else{
      for(int i = 0; i < topic_names.size(); i++)
	mf_subs.push_back(new message_filters::Subscriber<nvidia_to_ros::ImageAndCameraInfo>(nh, topic_names[i], queue_size));
    }
    
    if(topic_names.size() == 2){
      message_filters::TimeSynchronizer<nvidia_to_ros::ImageAndCameraInfo, nvidia_to_ros::ImageAndCameraInfo>* sync =
	new message_filters::TimeSynchronizer<nvidia_to_ros::ImageAndCameraInfo,
					      nvidia_to_ros::ImageAndCameraInfo>(*mf_subs[0],
										 *mf_subs[1],
										 queue_size);
      sync->registerCallback(boost::bind(&Subscriber::two_callback, this, _1, _2));
    }
    else if(topic_names.size() == 3){
      message_filters::TimeSynchronizer<nvidia_to_ros::ImageAndCameraInfo,
					nvidia_to_ros::ImageAndCameraInfo,
					nvidia_to_ros::ImageAndCameraInfo>* sync =
	new message_filters::TimeSynchronizer<nvidia_to_ros::ImageAndCameraInfo,
					      nvidia_to_ros::ImageAndCameraInfo,
					      nvidia_to_ros::ImageAndCameraInfo>(*mf_subs[0],
										 *mf_subs[1],
										 *mf_subs[2],
										 queue_size);
      sync->registerCallback(boost::bind(&Subscriber::three_callback, this, _1, _2, _3));
    }
    else if(topic_names.size() == 4){
      message_filters::TimeSynchronizer<nvidia_to_ros::ImageAndCameraInfo,
					nvidia_to_ros::ImageAndCameraInfo,
					nvidia_to_ros::ImageAndCameraInfo,
					nvidia_to_ros::ImageAndCameraInfo>* sync =
	new message_filters::TimeSynchronizer<nvidia_to_ros::ImageAndCameraInfo,
					      nvidia_to_ros::ImageAndCameraInfo,
					      nvidia_to_ros::ImageAndCameraInfo,
					      nvidia_to_ros::ImageAndCameraInfo>(*mf_subs[0],
										 *mf_subs[1],
										 *mf_subs[2],
										 *mf_subs[3], 
										 queue_size);
      sync->registerCallback(boost::bind(&Subscriber::four_callback, this, _1, _2, _3, _4));
    }
    else if(topic_names.size() == 5){
      message_filters::TimeSynchronizer<nvidia_to_ros::ImageAndCameraInfo,
					nvidia_to_ros::ImageAndCameraInfo,
					nvidia_to_ros::ImageAndCameraInfo,
					nvidia_to_ros::ImageAndCameraInfo,
					nvidia_to_ros::ImageAndCameraInfo>* sync =
	new message_filters::TimeSynchronizer<nvidia_to_ros::ImageAndCameraInfo,
					      nvidia_to_ros::ImageAndCameraInfo,
					      nvidia_to_ros::ImageAndCameraInfo,
					      nvidia_to_ros::ImageAndCameraInfo,
					      nvidia_to_ros::ImageAndCameraInfo>(*mf_subs[0],
										 *mf_subs[1],
										 *mf_subs[2],
										 *mf_subs[3],
										 *mf_subs[4],
										 queue_size);
      sync->registerCallback(boost::bind(&Subscriber::five_callback, this, _1, _2, _3, _4, _5));
    }
    else if(topic_names.size() == 6){
      message_filters::TimeSynchronizer<nvidia_to_ros::ImageAndCameraInfo,
					nvidia_to_ros::ImageAndCameraInfo,
					nvidia_to_ros::ImageAndCameraInfo,
					nvidia_to_ros::ImageAndCameraInfo,
					nvidia_to_ros::ImageAndCameraInfo,
					nvidia_to_ros::ImageAndCameraInfo>* sync =
	new message_filters::TimeSynchronizer<nvidia_to_ros::ImageAndCameraInfo,
					      nvidia_to_ros::ImageAndCameraInfo,
					      nvidia_to_ros::ImageAndCameraInfo,
					      nvidia_to_ros::ImageAndCameraInfo,
					      nvidia_to_ros::ImageAndCameraInfo,
					      nvidia_to_ros::ImageAndCameraInfo>(*mf_subs[0],
										 *mf_subs[1],
										 *mf_subs[2],
										 *mf_subs[3],
										 *mf_subs[4],
										 *mf_subs[5],
										 queue_size);
      sync->registerCallback(boost::bind(&Subscriber::six_callback, this, _1, _2, _3, _4, _5, _6));
    }
  }
  
  Subscriber::Subscriber(ros::NodeHandle nh, std::vector<std::string> topic_names, int queue_size,
			 void(*fp)(std::vector<NvImageMessage>))
    : Subscriber(nh, topic_names, queue_size, std::bind(fp, std::placeholders::_1)){
    
  }
  
  void Subscriber::single_callback_function(std::function<void(NvImageMessage)> func, std::vector<NvImageMessage> messages){
    func(messages[0]);
  }
  
  void init_helper(NvImageMessage& img, const nvidia_to_ros::ImageAndCameraInfoConstPtr& msg){
    img.image_and_info = *msg;
    img.stamp = msg->header.stamp;
    img.camera_info = msg->camera_info;
    img.params.width[0] = msg->image.width;
    img.params.height[0] = msg->image.height;
    img.params.pitch[0] = msg->image.step;
    img.params.payloadType = NvBufferPayload_SurfArray;
    img.params.layout[0] = NvBufferLayout_Pitch;
    if(msg->image.encoding == sensor_msgs::image_encodings::RGBA8 ||
       msg->image.encoding == sensor_msgs::image_encodings::TYPE_8UC4){
      img.params.pixel_format = NvBufferColorFormat_ABGR32;
    }
    else if(msg->image.encoding == sensor_msgs::image_encodings::MONO8 ||
	    msg->image.encoding == sensor_msgs::image_encodings::TYPE_8UC1){
      img.params.pixel_format = NvBufferColorFormat_GRAY8;
    }
    else{
      ROS_ERROR_STREAM("image encoding " << msg->image.encoding << " not supported. Add support for it in the init_helper function"
		       << " in nv2ros_non_jetson.cpp");
    }
  }
  
  void Subscriber::one_callback(const nvidia_to_ros::ImageAndCameraInfoConstPtr& msg){
    NvImageMessage img(0, NvBufferParams(), NvBufferParamsEx(), msg->header.stamp);
    init_helper(img, msg);

    std::vector<NvImageMessage> imgs;
    imgs.push_back(img);
    
    callback_function(imgs);
  }
  
  void Subscriber::two_callback(const nvidia_to_ros::ImageAndCameraInfoConstPtr& msg1,
				const nvidia_to_ros::ImageAndCameraInfoConstPtr& msg2){
    NvImageMessage img1(0, NvBufferParams(), NvBufferParamsEx(), msg1->header.stamp);
    NvImageMessage img2(0, NvBufferParams(), NvBufferParamsEx(), msg2->header.stamp);
    init_helper(img1, msg1);
    init_helper(img2, msg2);

    std::vector<NvImageMessage> imgs;
    imgs.push_back(img1);
    imgs.push_back(img2);
    
    callback_function(imgs);
  }
  
  void Subscriber::three_callback(const nvidia_to_ros::ImageAndCameraInfoConstPtr& msg1,
				  const nvidia_to_ros::ImageAndCameraInfoConstPtr& msg2,
				  const nvidia_to_ros::ImageAndCameraInfoConstPtr& msg3){
    NvImageMessage img1(0, NvBufferParams(), NvBufferParamsEx(), msg1->header.stamp);
    NvImageMessage img2(0, NvBufferParams(), NvBufferParamsEx(), msg2->header.stamp);
    NvImageMessage img3(0, NvBufferParams(), NvBufferParamsEx(), msg3->header.stamp);
    init_helper(img1, msg1);
    init_helper(img2, msg2);
    init_helper(img3, msg3);

    std::vector<NvImageMessage> imgs;
    imgs.push_back(img1);
    imgs.push_back(img2);
    imgs.push_back(img3);
    
    callback_function(imgs);
  }
  
  void Subscriber::four_callback(const nvidia_to_ros::ImageAndCameraInfoConstPtr& msg1,
				 const nvidia_to_ros::ImageAndCameraInfoConstPtr& msg2,
				 const nvidia_to_ros::ImageAndCameraInfoConstPtr& msg3,
				 const nvidia_to_ros::ImageAndCameraInfoConstPtr& msg4){
    NvImageMessage img1(0, NvBufferParams(), NvBufferParamsEx(), msg1->header.stamp);
    NvImageMessage img2(0, NvBufferParams(), NvBufferParamsEx(), msg2->header.stamp);
    NvImageMessage img3(0, NvBufferParams(), NvBufferParamsEx(), msg3->header.stamp);
    NvImageMessage img4(0, NvBufferParams(), NvBufferParamsEx(), msg4->header.stamp);
    init_helper(img1, msg1);
    init_helper(img2, msg2);
    init_helper(img3, msg3);
    init_helper(img4, msg4);

    std::vector<NvImageMessage> imgs;
    imgs.push_back(img1);
    imgs.push_back(img2);
    imgs.push_back(img3);
    imgs.push_back(img4);
    
    callback_function(imgs);
  }
  
  void Subscriber::five_callback(const nvidia_to_ros::ImageAndCameraInfoConstPtr& msg1,
				 const nvidia_to_ros::ImageAndCameraInfoConstPtr& msg2,
				 const nvidia_to_ros::ImageAndCameraInfoConstPtr& msg3,
				 const nvidia_to_ros::ImageAndCameraInfoConstPtr& msg4,
				 const nvidia_to_ros::ImageAndCameraInfoConstPtr& msg5){
    NvImageMessage img1(0, NvBufferParams(), NvBufferParamsEx(), msg1->header.stamp);
    NvImageMessage img2(0, NvBufferParams(), NvBufferParamsEx(), msg2->header.stamp);
    NvImageMessage img3(0, NvBufferParams(), NvBufferParamsEx(), msg3->header.stamp);
    NvImageMessage img4(0, NvBufferParams(), NvBufferParamsEx(), msg4->header.stamp);
    NvImageMessage img5(0, NvBufferParams(), NvBufferParamsEx(), msg5->header.stamp);
    init_helper(img1, msg1);
    init_helper(img2, msg2);
    init_helper(img3, msg3);
    init_helper(img4, msg4);
    init_helper(img5, msg5);

    std::vector<NvImageMessage> imgs;
    imgs.push_back(img1);
    imgs.push_back(img2);
    imgs.push_back(img3);
    imgs.push_back(img4);
    imgs.push_back(img5);
    
    callback_function(imgs);

  }
  
  void Subscriber::six_callback(const nvidia_to_ros::ImageAndCameraInfoConstPtr& msg1,
				const nvidia_to_ros::ImageAndCameraInfoConstPtr& msg2,
				const nvidia_to_ros::ImageAndCameraInfoConstPtr& msg3,
				const nvidia_to_ros::ImageAndCameraInfoConstPtr& msg4,
				const nvidia_to_ros::ImageAndCameraInfoConstPtr& msg5,
				const nvidia_to_ros::ImageAndCameraInfoConstPtr& msg6){
    NvImageMessage img1(0, NvBufferParams(), NvBufferParamsEx(), msg1->header.stamp);
    NvImageMessage img2(0, NvBufferParams(), NvBufferParamsEx(), msg2->header.stamp);
    NvImageMessage img3(0, NvBufferParams(), NvBufferParamsEx(), msg3->header.stamp);
    NvImageMessage img4(0, NvBufferParams(), NvBufferParamsEx(), msg4->header.stamp);
    NvImageMessage img5(0, NvBufferParams(), NvBufferParamsEx(), msg5->header.stamp);
    NvImageMessage img6(0, NvBufferParams(), NvBufferParamsEx(), msg6->header.stamp);
    init_helper(img1, msg1);
    init_helper(img2, msg2);
    init_helper(img3, msg3);
    init_helper(img4, msg4);
    init_helper(img5, msg5);
    init_helper(img6, msg6);

    std::vector<NvImageMessage> imgs;
    imgs.push_back(img1);
    imgs.push_back(img2);
    imgs.push_back(img3);
    imgs.push_back(img4);
    imgs.push_back(img5);
    imgs.push_back(img6);
    
    callback_function(imgs);
  }
  
  
};
