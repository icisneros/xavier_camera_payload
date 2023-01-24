#include <nvidia_to_ros/nv2ros.h>

namespace nv2ros{

  std::string params_to_string(const NvBufferParams& params){
    std::ostringstream stream;
    stream << "dmabuf_fd: " << params.dmabuf_fd << std::endl
	   << "nv_buffer: " << params.nv_buffer << std::endl
	   << "memsize: " << params.memsize << std::endl
	   << "nv_buffer_size: " << params.nv_buffer_size << std::endl
      	   << "num_planes: " << params.num_planes << std::endl;
    
    for(int i = 0; i < params.num_planes; i++){
      stream << "plane " << i << std::endl
	     << "\twidth: " << params.width[i] << std::endl
	     << "\theight: " << params.height[i] << std::endl
	     << "\tpitch: " << params.pitch[i] << std::endl
	     << "\toffset: " << params.offset[i] << std::endl
	     << "\tpsize: " << params.psize[i] << std::endl
	     << "\tlayout: " << params.layout[i] << std::endl;
    }

    stream << "payloadType: ";
    if(params.payloadType == NvBufferPayload_SurfArray)
      stream << "NvBufferPayload_SurfArray" << std::endl;
    if(params.payloadType == NvBufferPayload_MemHandle)
      stream << "NvBufferPayload_MemHandle" << std::endl;

    stream << "pixel_format: ";
    switch(params.pixel_format){
    case NvBufferColorFormat_YUV420:
      stream << "NvBufferColorFormat_YUV420";
      break;
    case NvBufferColorFormat_YVU420:
      stream << "NvBufferColorFormat_YVU420";
      break;
    case NvBufferColorFormat_YUV422:
      stream << "NvBufferColorFormat_YUV422";
      break;
    case NvBufferColorFormat_YUV420_ER:
      stream << "NvBufferColorFormat_YUV420_ER";
      break;
    case NvBufferColorFormat_YVU420_ER:
      stream << "NvBufferColorFormat_YVU420_ER";
      break;
    case NvBufferColorFormat_NV12:
      stream << "NvBufferColorFormat_NV12";
      break;
    case NvBufferColorFormat_NV12_ER:
      stream << "NvBufferColorFormat_NV12_ER";
      break;
    case NvBufferColorFormat_NV21:
      stream << "NvBufferColorFormat_NV21";
      break;
    case NvBufferColorFormat_NV21_ER:
      stream << "NvBufferColorFormat_NV21_ER";
      break;
    case NvBufferColorFormat_UYVY:
      stream << "NvBufferColorFormat_UYVY";
      break;
    case NvBufferColorFormat_UYVY_ER:
      stream << "NvBufferColorFormat_UYVY_ER";
      break;
    case NvBufferColorFormat_VYUY:
      stream << "NvBufferColorFormat_VYUY";
      break;
    case NvBufferColorFormat_VYUY_ER:
      stream << "NvBufferColorFormat_VYUY_ER";
      break;
    case NvBufferColorFormat_YUYV:
      stream << "NvBufferColorFormat_YUYV";
      break;
    case NvBufferColorFormat_YUYV_ER:
      stream << "NvBufferColorFormat_YUYV_ER";
      break;
    case NvBufferColorFormat_YVYU:
      stream << "NvBufferColorFormat_YVYU";
      break;
    case NvBufferColorFormat_YVYU_ER:
      stream << "NvBufferColorFormat_YVYU_ER";
      break;
    case NvBufferColorFormat_ABGR32:
      stream << "NvBufferColorFormat_ABGR32";
      break;
    case NvBufferColorFormat_XRGB32:
      stream << "NvBufferColorFormat_XRGB32";
      break;
    case NvBufferColorFormat_ARGB32:
      stream << "NvBufferColorFormat_ARGB32";
      break;
    case NvBufferColorFormat_NV12_10LE:
      stream << "NvBufferColorFormat_NV12_10LE";
      break;
    case NvBufferColorFormat_NV12_10LE_709:
      stream << "NvBufferColorFormat_NV12_10LE_709";
      break;
    case NvBufferColorFormat_NV12_10LE_709_ER:
      stream << "NvBufferColorFormat_NV12_10LE_709_ER";
      break;
    case NvBufferColorFormat_NV12_10LE_2020:
      stream << "NvBufferColorFormat_NV12_10LE_2020";
      break;
    case NvBufferColorFormat_NV21_10LE:
      stream << "NvBufferColorFormat_NV21_10LE";
      break;
    case NvBufferColorFormat_NV12_12LE:
      stream << "NvBufferColorFormat_NV12_12LE";
      break;
    case NvBufferColorFormat_NV12_12LE_2020:
      stream << "NvBufferColorFormat_NV12_12LE_2020";
      break;
    case NvBufferColorFormat_NV21_12LE:
      stream << "NvBufferColorFormat_NV21_12LE";
      break;
    case NvBufferColorFormat_YUV420_709:
      stream << "NvBufferColorFormat_YUV420_709";
      break;
    case NvBufferColorFormat_YUV420_709_ER:
      stream << "NvBufferColorFormat_YUV420_709_ER";
      break;
    case NvBufferColorFormat_NV12_709:
      stream << "NvBufferColorFormat_NV12_709";
      break;
    case NvBufferColorFormat_NV12_709_ER:
      stream << "NvBufferColorFormat_NV12_709_ER";
      break;
    case NvBufferColorFormat_YUV420_2020:
      stream << "NvBufferColorFormat_YUV420_2020";
      break;
    case NvBufferColorFormat_NV12_2020:
      stream << "NvBufferColorFormat_NV12_2020";
      break;
    case NvBufferColorFormat_SignedR16G16:
      stream << "NvBufferColorFormat_SignedR16G16";
      break;
    case NvBufferColorFormat_A32:
      stream << "NvBufferColorFormat_A32";
      break;
    case NvBufferColorFormat_YUV444:
      stream << "NvBufferColorFormat_YUV444";
      break;
    case NvBufferColorFormat_GRAY8:
      stream << "NvBufferColorFormat_GRAY8";
      break;
    case NvBufferColorFormat_NV16:
      stream << "NvBufferColorFormat_NV16";
      break;
    case NvBufferColorFormat_NV16_10LE:
      stream << "NvBufferColorFormat_NV16_10LE";
      break;
    case NvBufferColorFormat_NV24:
      stream << "NvBufferColorFormat_NV24";
      break;
    case NvBufferColorFormat_NV24_10LE:
      stream << "NvBufferColorFormat_NV24_10LE";
      break;
    case NvBufferColorFormat_NV16_ER:
      stream << "NvBufferColorFormat_NV16_ER";
      break;
    case NvBufferColorFormat_NV24_ER:
      stream << "NvBufferColorFormat_NV24_ER";
      break;
    case NvBufferColorFormat_NV16_709:
      stream << "NvBufferColorFormat_NV16_709";
      break;
    case NvBufferColorFormat_NV24_709:
      stream << "NvBufferColorFormat_NV24_709";
      break;
    case NvBufferColorFormat_NV16_709_ER:
      stream << "NvBufferColorFormat_NV16_709_ER";
      break;
    case NvBufferColorFormat_NV24_709_ER:
      stream << "NvBufferColorFormat_NV24_709_ER";
      break;
    case NvBufferColorFormat_NV24_10LE_709:
      stream << "NvBufferColorFormat_NV24_10LE_709";
      break;
    case NvBufferColorFormat_NV24_10LE_709_ER:
      stream << "NvBufferColorFormat_NV24_10LE_709_ER";
      break;
    case NvBufferColorFormat_NV24_10LE_2020:
      stream << "NvBufferColorFormat_NV24_10LE_2020";
      break;
    case NvBufferColorFormat_NV24_12LE_2020:
      stream << "NvBufferColorFormat_NV24_12LE_2020";
      break;
    case NvBufferColorFormat_RGBA_10_10_10_2_709:
      stream << "NvBufferColorFormat_RGBA_10_10_10_2_709";
      break;
    case NvBufferColorFormat_RGBA_10_10_10_2_2020:
      stream << "NvBufferColorFormat_RGBA_10_10_10_2_2020";
      break;
    case NvBufferColorFormat_BGRA_10_10_10_2_709:
      stream << "NvBufferColorFormat_BGRA_10_10_10_2_709";
      break;
    case NvBufferColorFormat_BGRA_10_10_10_2_2020:
      stream << "NvBufferColorFormat_BGRA_10_10_10_2_2020";
      break;
    case NvBufferColorFormat_Invalid:
      stream << "NvBufferColorFormat_Invalid";
      break;
    }

    return stream.str();
  }
  
  void check(int status, std::string function_name){
    if(status)
      ROS_ERROR_STREAM(function_name << " failed");
  }

  int new_nv_buffer(int width, int height){
    int dmabuf_fd;
  
    NvBufferCreateParams input_params = {0};
    input_params.payloadType = NvBufferPayload_SurfArray;
    input_params.width = width;
    input_params.height = height;
    input_params.layout = NvBufferLayout_Pitch;
    input_params.colorFormat = NvBufferColorFormat_ABGR32;
    input_params.nvbuf_tag = NvBufferTag_NONE;
  
    check(NvBufferCreateEx(&dmabuf_fd, &input_params), "NvBufferCreateEx");
  
    return dmabuf_fd;
  }

  
  std::vector<uint8_t> ImageRequest::serialize(){
    SerializationHelper sh;
    sh.push(fd_id);
    sh.push(image_id);
    sh.push(send_fd);

    return sh.get_bytes();
  }
  
  void ImageRequest::deserialize(std::vector<uint8_t> bytes){
    SerializationHelper sh(bytes);
    send_fd = sh.pop<bool>();
    image_id = sh.pop<int>();
    fd_id = sh.pop<int>();
  }
    
  std::vector<uint8_t> ImageResponse::serialize(){
    SerializationHelper sh;
    sh.push(valid);
    
    return sh.get_bytes();
  }
  
  void ImageResponse::deserialize(std::vector<uint8_t> bytes){
    SerializationHelper sh(bytes);
    valid = sh.pop<bool>();
  }
    
  std::vector<uint8_t> ImageTransferStatus::serialize(){
    SerializationHelper sh;
    sh.push(complete);
    sh.push(fd_id);
    
    return sh.get_bytes();
  }
  
  void ImageTransferStatus::deserialize(std::vector<uint8_t> bytes){
    SerializationHelper sh(bytes);
    fd_id = sh.pop<int>();
    complete = sh.pop<bool>();
  }

  // ================================================================================
  // ------------------------------ PublisherQueue ----------------------------------
  // ================================================================================
  
  PublisherQueue::PublisherQueue(int queue_size, int image_count)
    : queue_size(std::max(1, queue_size))
    , image_count(image_count)
    , next_fd_id(1)
    , next_image_id(1){
    
  }

  void PublisherQueue::prune_queue_elements(){
    ros::Time latest(0, 0);
    std::list<ros::Time> complete_stamps;
    for(std::list<QueueElement>::iterator it = queue.begin(); it != queue.end(); it++){
      QueueElement& e = *it;
      latest = std::max(latest, e.stamp);
      if(e.is_complete())
	complete_stamps.push_back(e.stamp);
    }
    
    complete_stamps.sort();
    complete_stamps.remove(latest);
    for(std::list<ros::Time>::iterator it = complete_stamps.begin(); it != complete_stamps.end(); it++){
      ros::Time stamp = *it;
      if(complete_stamps.size() > queue_size){
	std::vector<FileDescriptor> released_fds;
	queue.remove_if([stamp, &released_fds](const QueueElement& e){
	    for(int i = 0; i < e.images.size(); i++)
	      if(e.images[i].users > 0)
		return false;
	    
	    bool should_remove = stamp == e.stamp;
	    if(should_remove)
	      for(int i = 0; i < e.images.size(); i++)
	        released_fds.push_back(e.images[i].fd); // TODO check for fd == 0?
	    
	    return should_remove;
	  });
	//*
	for(int i = 0; i < released_fds.size(); i++){
	  //ROS_INFO_STREAM("releasing fd " << released_fds[i].fd << " " << released_fds[i].id);
	  available_fds.push_back(released_fds[i]);
	}
	//*/
      }
    }

    // TODO test this
    int count = 0;
    std::list<QueueElement>::iterator it = queue.end();
    while(it != queue.begin()){
      it--;
      QueueElement& e = *it;
      if(e.stamp != latest)
	count++;

      if(count > queue_size){
	for(int j = 0; j < e.images.size(); j++)
	  if(e.images[j].fd.fd != 0)
	    available_fds.push_back(e.images[j].fd);
	it = queue.erase(it);
      }
    }
  }

  QueueElement& PublisherQueue::get_queue_element(ros::Time stamp){
    for(std::list<QueueElement>::iterator it = queue.begin(); it != queue.end(); it++){
      QueueElement& e = *it;
      if(e.stamp == stamp)
	return e;
    }

    
    queue.emplace_back(image_count, stamp);
    return queue.back();
  }
  
  FileDescriptor PublisherQueue::get_available_fd(NvBufferParams params){
    FileDescriptor fd;
    
    if(available_fds.empty()){
      // TODO if info doesn't match an fd's params, then destroy the fd and remake it with the right params
      //ROS_INFO("creating new nv buffer");
      NvBufferCreateParams input_params = {0};
      input_params.payloadType = params.payloadType;
      input_params.width = params.width[0];
      input_params.height = params.height[0];
      input_params.layout = NvBufferLayout_Pitch; // TODO should i use params.layout[0], check if it is equal to ..._Pitch
      input_params.colorFormat = params.pixel_format;
      input_params.nvbuf_tag = NvBufferTag_NONE;
      
      check(NvBufferCreateEx(&fd.fd, &input_params), "NvBufferCreateEx");
      fd.id = next_fd_id++;
      
      id_to_fd_mutex.lock();
      id_to_fd[fd.id] = fd.fd;
      id_to_fd_mutex.unlock();
    }
    else{
      fd = available_fds.back();
      available_fds.pop_back();
    }

    return fd;
  }

  void PublisherQueue::print(){
    int i = 0;
    for(std::list<QueueElement>::iterator it = queue.begin(); it != queue.end(); it++){
      QueueElement& e = *it;
      ROS_INFO_STREAM(i << ": " << e.stamp << " complete: " << e.is_complete());
      for(int j = 0; j < e.images.size(); j++)
      	ROS_INFO_STREAM("\t" << j << ": " << e.images[j].fd.id << ", " << e.images[j].image_id);
      i++;
    }
  }

  // TODO think about add being called from multiple threads
  bool PublisherQueue::add(int dmabuf_fd, NvBufferParams params, NvBufferParamsEx params_ex, ros::Time stamp, int index,
			   NvImageInfo* info){
    queue_mutex.lock();

    // remove old elements from the queue and get the current element
    //ROS_INFO_STREAM("queue before prune");
    //print();
    prune_queue_elements();
    //ROS_INFO_STREAM("queue after prune");
    //print();
    QueueElement& queue_element = get_queue_element(stamp);
    //ROS_INFO_STREAM("queue_element: " << stamp);

    // initialize attributes for the new image
    QueueImage& queue_image = queue_element.images[index];
    if(queue_image.users > 0){
      //ROS_INFO_STREAM(queue_image.fd.fd << ", " << queue_image.fd.id << ", " << queue_image.image_id
      //	      <<" in use " << queue_image.users);
      queue_mutex.unlock();
      return false;
    }
    queue_image.stamp = stamp;
    queue_image.fd = get_available_fd(params);
    //ROS_INFO_STREAM("current fd: " << queue_image.fd.fd << " " << queue_image.fd.id);
    queue_image.image_id = next_image_id++;
    queue_image.users++; // use this so that it isn't pruned while copying
    //ROS_INFO_STREAM("users++ " << queue_image.fd.id << " " << queue_image.image_id);
    queue_mutex.unlock();

    // fill out info
    info->stamp = stamp;
    info->fd_id = queue_image.fd.id;
    info->image_id = queue_image.image_id;
    check(NvBufferGetParams(queue_image.fd.fd, &info->params), "NvBufferGetParams");
    check(NvBufferGetParamsEx(queue_image.fd.fd, &info->params_ex), "NvBufferGetParamsEx");
    
    // copy the image
    NvBufferRect rect;
    rect.top = 0;
    rect.left = 0;
    rect.width = params.width[0];
    rect.height = params.height[0];
    NvBufferTransformParams transform_params{0};
    transform_params.transform_flag = NVBUFFER_TRANSFORM_FLIP;
    transform_params.transform_flip = NvBufferTransform_None;
    transform_params.transform_filter = NvBufferTransform_Filter_Nearest;
    transform_params.src_rect = rect;
    transform_params.dst_rect = rect;
    check(NvBufferTransformEx(dmabuf_fd, &params_ex, queue_image.fd.fd, &(info->params_ex), &transform_params),
	  "NvBufferTransformEx");
    // TODO do i have to sync to cpu or device after transform?

    queue_mutex.lock();
    //ROS_INFO_STREAM("users-- " << queue_image.fd.id << " " << queue_image.image_id);
    queue_image.users--;
    queue_mutex.unlock();
    
    return true;
  }
  
  int PublisherQueue::get_fd(int fd_id){
    std::lock_guard<std::mutex> lock(id_to_fd_mutex);
    return id_to_fd[fd_id]; // TODO handle if it doesn't exist, this will crash
  }
  
  bool PublisherQueue::check_valid_and_lock(int fd_id, int image_id){
    std::lock_guard<std::mutex> lock(queue_mutex);
    
    //ROS_INFO_STREAM("target: " << fd_id << " " << image_id);
    //print();
    for(std::list<QueueElement>::iterator it = queue.begin(); it != queue.end(); it++){
      QueueElement& queue_element = *it;
      for(int j = 0; j < queue_element.images.size(); j++){
	QueueImage& image = queue_element.images[j];
	if(image.fd.id == fd_id && image.image_id == image_id){
	  //ROS_INFO_STREAM(j << "qe " << queue_element.stamp << " im " << image.fd.id << " " << image.image_id <<" users++");
	  image.users++;
	  return true;
	}
      }
    }
    
    return false;
  }
  
  void PublisherQueue::unlock(int fd_id){
    std::lock_guard<std::mutex> lock(queue_mutex);
    
    //ROS_INFO_STREAM("unlock: " << fd_id);
    for(std::list<QueueElement>::iterator it = queue.begin(); it != queue.end(); it++){
      QueueElement& queue_element = *it;
      for(int j = 0; j < queue_element.images.size(); j++){
	QueueImage& image = queue_element.images[j];
	if(image.fd.id == fd_id){
	  //ROS_INFO_STREAM(j << "qe " << queue_element.stamp << " im " << image.fd.id << " " << image.image_id <<" users--");
	  image.users--;
	  if(image.users < 0){
	    image.users = 0;
	    //ROS_INFO_STREAM("IMAGE USERS WAS NEGATIVE");
	  }
	}
      }
    }
  }

  // --- QueueImage ---

  QueueImage::QueueImage(){
    fd.fd = 0;
    fd.id = 0;
    image_id = 0;
    users = 0;
  }
  
  // --- QueueElement ---
  
  QueueElement::QueueElement(int image_count, ros::Time stamp)
    : images(image_count)
    , stamp(stamp){
    
  }
  
  bool QueueElement::is_complete(){
    for(int i = 0; i < images.size(); i++)
      if(images[i].fd.fd == 0)
	return false;
    
    return true;
  }

  bool QueueElement::operator<(const QueueElement& e) const {
    return stamp < e.stamp;
  }

  bool QueueElement::operator==(const QueueElement& e) const {
    return stamp == e.stamp;
  }
  
  // ================================================================================
  // -------------------------------- Publisher -------------------------------------
  // ================================================================================
  
  Server* Publisher::server = NULL;

  Publisher::Publisher(ros::NodeHandle nh, std::vector<std::string> topic_names, int queue_size)
    : queue_size(queue_size)
    , queue(queue_size, topic_names.size()){
    
    for(int i = 0; i < topic_names.size(); i++)
      pubs.push_back(nh.advertise<std_msgs::ByteMultiArray>(topic_names[i], queue_size));
    
    if(server == NULL)
      server = new Server(topic_names, std::bind(&Publisher::message_callback, this,
						 std::placeholders::_1, std::placeholders::_2));
  }
  
  Publisher::Publisher(ros::NodeHandle nh, std::string topic_name, int queue_size)
    : Publisher(nh, std::vector<std::string>(1, topic_name), queue_size){
  }
  
  void Publisher::publish(int dmabuf_fd, NvBufferParams params, NvBufferParamsEx params_ex, ros::Time stamp, int index){
    //ROS_INFO_STREAM(color((Color)(index+1)) << "publish " << index << ": " << stamp << color());
    //ROS_INFO("publish start");
    

    //ROS_INFO("converting msg");
    std_msgs::ByteMultiArray msg;
    NvImageInfo info;
    bool added = queue.add(dmabuf_fd, params, params_ex, stamp, index, &info);
    if(!added)
      return;
    msg.data.resize(sizeof(info) + params.nv_buffer_size);
    std::memcpy(&(msg.data[0]), &info, sizeof(info));
    std::memcpy(&(msg.data[sizeof(info)]), params.nv_buffer, params.nv_buffer_size);
    
    std::string buffer_text;
    for(int i = 0; i < params.nv_buffer_size; i++)
      buffer_text += std::to_string((int)((uint8_t*)params.nv_buffer)[i]) + " ";
    //ROS_INFO_STREAM("buffer: " << buffer_text);

    //ROS_INFO("publishing msg");
    pubs[index].publish(msg);
  }
  
  void Publisher::message_callback(Message message, int cfd){
    switch(message.message_type){
    case IMAGE_REQUEST:
      {
	//ROS_INFO_STREAM(cfd << " received image request");
	//for(int i = 0; i < message.payload_size; i++)
	//  ROS_INFO_STREAM("request payload " << i << " " << (int)message.payload[i]);
	//ImageRequest ir = PubSubMessageHelper::get_image_request(message);
	ImageRequest image_request;
	image_request.deserialize(message.payload);
	//ROS_INFO_STREAM("REQUESTING " << image_request.fd_id << " " << image_request.image_id);
	if(queue.check_valid_and_lock(image_request.fd_id, image_request.image_id)){
	  //ROS_INFO("valid");
	  //Message response = PubSubMessageHelper::get_image_response_message(true);
	  ImageResponse image_response;
	  image_response.valid = true;
	  Message response(IMAGE_RESPONSE, image_response);
	  //ROS_INFO("sending response");
	  server->send(cfd, response);
	  if(image_request.send_fd){
	    //ROS_INFO("sending fd");
	    server->send_fd(cfd, queue.get_fd(image_request.fd_id));
	  }
	}
	else{
	  ROS_INFO_STREAM("invalid: " << image_request.fd_id << " " << image_request.image_id);
	  //Message response = PubSubMessageHelper::get_image_response_message(false);
	  ImageResponse ir;
	  ir.valid = false;
	  Message response(IMAGE_RESPONSE, ir);
	  //for(int i = 0; i < response.payload_size; i++)
	  //  ROS_INFO_STREAM("payload " << i << " " << (int)response.payload[i]);
	  //ROS_INFO("sending response");
	  server->send(cfd, response);
	}
      }
      break;
    case IMAGE_TRANSFER_STATUS:
      {
	//ROS_INFO_STREAM(cfd << " received image transfer status");
        //ImageTransferStatus its = PubSubMessageHelper::get_image_transfer_status(message);
	ImageTransferStatus its;
	its.deserialize(message.payload);
	if(its.complete){
	  //ROS_INFO_STREAM("unlocking " << its.fd_id);
	  queue.unlock(its.fd_id);
	}
      }
      break;
    }
  }

  void Publisher::publish(NvImageMessage& image, int index){
    publish(image.dmabuf_fd, image.params, image.params_ex, image.stamp, index);
  }

  // ================================================================================
  // -------------------------------- Subscriber ------------------------------------
  // ================================================================================

  SubscriberQueue::SubscriberQueue(int queue_size, int images)
    : queue_size(queue_size)
    , images(images){

  }
  
  bool SubscriberQueue::add(NvImageInfo info, int index){
    if(queue.count(info.stamp) == 0)
      queue[info.stamp] = std::vector<NvImageInfo>(images);
    
    std::vector<NvImageInfo>& vec = queue[info.stamp];
    vec[index] = info;
    
    bool complete = true;
    for(int i = 0; i < vec.size(); i++){
      //ROS_INFO_STREAM(i << " " << vec[i].stamp);
      if(vec[i].stamp != info.stamp){
	//ROS_INFO_STREAM("not complete " << vec[i].stamp << " " << info.stamp);
	complete = false;
	break;
      }
    }
    
    while(queue.size() > queue_size || (complete && !queue.empty() && queue.begin()->first < info.stamp))
      queue.erase(queue.begin());
    
    return complete;
  }
  
  std::vector<NvImageInfo> SubscriberQueue::get(ros::Time stamp){
    std::vector<NvImageInfo> ret = queue[stamp];
    queue.erase(stamp);
    return ret;
  }

  void SubscriberQueue::clear(){
    queue.clear();
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
    client = new Client(topic_names);


    for(int i = 0; i < topic_names.size(); i++)
      subs.push_back(nh.subscribe<std_msgs::ByteMultiArray>(topic_names[i], queue_size,
							    std::bind(&Subscriber::callback, this, std::placeholders::_1, i),
							    ros::VoidConstPtr(),
							    ros::TransportHints().tcpNoDelay()));
  }
  
  Subscriber::Subscriber(ros::NodeHandle nh, std::vector<std::string> topic_names, int queue_size,
			 void(*fp)(std::vector<NvImageMessage>))
    : Subscriber(nh, topic_names, queue_size, std::bind(fp, std::placeholders::_1)){

  }
  
  Subscriber::Subscriber(ros::NodeHandle nh, std::string topic_name, int queue_size,
			 std::function<void(std::vector<NvImageMessage>)> callback_function)
    : Subscriber(nh, std::vector<std::string>(1, topic_name), queue_size, callback_function){
    
  }
  
  Subscriber::Subscriber(ros::NodeHandle nh, std::string topic_name, int queue_size, void(*fp)(std::vector<NvImageMessage>))
    : Subscriber(nh, std::vector<std::string>(1, topic_name), queue_size, fp){
    
  }
  
  void Subscriber::callback(const std_msgs::ByteMultiArray::ConstPtr& msg, int index){
    NvImageInfo current_info;
    //ROS_INFO("here");
    std::memcpy(&current_info, &(msg->data[0]), sizeof(current_info));
    //ROS_INFO_STREAM("here2: " << current_info.params.nv_buffer_size << " " << current_info.params.nv_buffer);
    current_info.params.nv_buffer = new uint8_t[current_info.params.nv_buffer_size];
    //ROS_INFO_STREAM("here3: " << current_info.params.nv_buffer_size << " " << current_info.params.nv_buffer
    //		    << " " << (msg->data.size() - current_info.params.nv_buffer_size) << " " << sizeof(current_info));
    std::string test_text;
    for(int i = msg->data.size() - current_info.params.nv_buffer_size; i < msg->data.size(); i++){
      uint8_t v;
      std::memcpy(&v, &(msg->data[i]), 1);
      test_text += std::to_string((int)v) + " ";
    }
    //ROS_INFO_STREAM("test: " << test_text);
    std::memcpy(current_info.params.nv_buffer,
		&(msg->data[msg->data.size() - current_info.params.nv_buffer_size]),
		current_info.params.nv_buffer_size);
    //ROS_INFO("here4");

    std::string buffer_text;
    for(int i = 0; i < current_info.params.nv_buffer_size; i++)
      buffer_text += std::to_string((int)((uint8_t*)current_info.params.nv_buffer)[i]) + " ";
    //ROS_INFO_STREAM("buffer: " << buffer_text);
    //ROS_INFO_STREAM("bytemultiarray callback " << index << " fd_id " << current_info.fd_id << " image_id " << current_info.image_id);

    //ROS_INFO_STREAM(color((Color)(index+1), bold) << "receive " << index << ": " << current_info.stamp << color());
    bool complete = queue.add(current_info, index);
    if(complete){
      std::vector<NvImageInfo> infos = queue.get(current_info.stamp);

      std::vector<NvImageMessage> image_messages;
      bool all_valid = true;
      int free_size = topic_names.size();
      for(int i = 0; i < infos.size(); i++){
	NvImageInfo info = infos[i];
	// figure out if we need the fd
	bool send_fd;
	int dmabuf_fd;
	if(fds.count(info.fd_id) != 0){
	  dmabuf_fd = fds[info.fd_id];
	  send_fd = false;
	}
	else
	  send_fd = true;

	// send the request
	//ROS_INFO_STREAM("sending image request " << info.fd_id << ", " << info.image_id << ", " << send_fd);
	ImageRequest image_request;
	image_request.fd_id = info.fd_id;
	image_request.image_id = info.image_id;
	image_request.send_fd = send_fd;
	Message request(IMAGE_REQUEST, image_request);
	bool client_success = client->send(request);
	if(!client_success){
	  reset();
	  return;
	}

	// read the response
	//ROS_INFO("reading image response");
	Message response;
	client_success = client->read(&response, response.get_non_payload_byte_count());
	if(!client_success){
	  reset();
	  return;
	}
	response.payload.resize(response.payload_size);
	client_success = client->read(&response.payload[0], response.payload_size);
	if(!client_success){
	  reset();
	  return;
	}
	ImageResponse image_response;
	image_response.deserialize(response.payload);

	if(image_response.valid){
	  //ROS_INFO("image response valid");
	  if(send_fd){
	    client_success = client->recv_fd(&fds[info.fd_id]);
	    if(!client_success){
	      reset();
	      return;
	    }
	    dmabuf_fd = fds[info.fd_id];
	  }

	  NvImageMessage msg(dmabuf_fd, info.params, info.params_ex, info.stamp);
	  //msg.dmabuf_fd = dmabuf_fd;//local_dmabuf_fd;
	  //msg.params = info.params;
	  //msg.params_ex = info.params_ex;
	  image_messages.push_back(msg);
	}
	else{
	  all_valid = false;
	  free_size = i-1;
	  break;
	  //ROS_INFO("image response INVALID");
	}
      }

      if(all_valid)
	callback_function(image_messages);

      for(int i = 0; i < free_size; i++){
	//ROS_INFO_STREAM("sending image transfer status for fd_id " << infos[i].fd_id);
	ImageTransferStatus its;
	its.complete = true;
	its.fd_id = infos[i].fd_id;
	Message status(IMAGE_TRANSFER_STATUS, its);
	bool client_success = client->send(status);
	if(!client_success){
	  reset();
	  return;
	}
      }
    }
    
  }

  void Subscriber::reset(){
    fds.clear();
    queue.clear();
  }
  
  void Subscriber::single_callback_function(std::function<void(NvImageMessage)> func, std::vector<NvImageMessage> messages){
    func(messages[0]);
  }
  
  // ================================================================================
  // ------------------------------ NvImageMessage ----------------------------------
  // ================================================================================
  
  NvImageMessage::NvImageMessage(int dmabuf_fd, NvBufferParams params, NvBufferParamsEx params_ex, ros::Time stamp)
    : dmabuf_fd(dmabuf_fd)
    , params(params)
    , params_ex(params_ex)
    , stamp(stamp){
    
  }
  
  NvImageMessage::NvImageMessage(int width, int height, ros::Time stamp,
				 NvBufferColorFormat color_format,
				 NvBufferLayout layout,
				 NvBufferPayloadType payload_type,
				 NvBufferTag tag)
    : stamp(stamp){
    
    NvBufferCreateParams input_params = {0};
    input_params.payloadType = payload_type;
    input_params.width = width;
    input_params.height = height;
    input_params.layout = layout;
    input_params.colorFormat = color_format;
    input_params.nvbuf_tag = tag;
  
    check(NvBufferCreateEx(&dmabuf_fd, &input_params), "NvBufferCreateEx nvim");
    check(NvBufferGetParams(dmabuf_fd, &params), "NvBufferGetParams nvim");
    check(NvBufferGetParamsEx(dmabuf_fd, &params_ex), "NvBufferGetParamsEx nvim");
  }
  
  NvImageMessage::~NvImageMessage(){
    
  }

  void NvImageMessage::mem_map(void** pointer){
    nv2ros::check(NvBufferMemMapEx(dmabuf_fd, &params_ex, 0, NvBufferMem_Read_Write, pointer), "NvBufferMemMapEx nvim");
  }
  
  void NvImageMessage::mem_unmap(void** pointer){
    nv2ros::check(NvBufferMemUnMapEx(dmabuf_fd, &params_ex, 0, pointer), "NvBufferMemUnMapEx nvim");
  }
  
  void NvImageMessage::mem_sync_for_cpu(void** pointer){
    nv2ros::check(NvBufferMemSyncForCpuEx(dmabuf_fd, &params_ex, 0, pointer), "NvBufferMemSyncForCpuEx nvim");
  }
  
  void NvImageMessage::mem_sync_for_device(void** pointer){
    nv2ros::check(NvBufferMemSyncForDeviceEx(dmabuf_fd, &params_ex, 0, pointer), "NvBufferMemSyncForDeviceEx");
  }
  
  void NvImageMessage::transform(NvImageMessage& destination, NvBufferTransformParams* transform_params){
    nv2ros::check(NvBufferTransformEx(dmabuf_fd, &params_ex,
				      destination.dmabuf_fd, &destination.params_ex,
				      transform_params),
		  "NvBufferTransformEx nvim");
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
    return dmabuf_fd;
  }

  void NvImageMessage::set_stamp(ros::Time stamp){
    this->stamp = stamp;
  }
  
}
