#include <nvidia_to_ros/nv_server_client.h>
#include <iostream>

std::vector<int> list_servers(){
  std::vector<int> servers;
  
  DIR* dir = opendir(Server::SERVER_DIRECTORY.c_str());
  if(dir == NULL){
    ROS_INFO("dir is NULL");
    return servers;
  }
  struct dirent* entry = NULL;
  while((entry = readdir(dir)) != NULL){
    std::string name(entry->d_name);
    if(name.find(".") == std::string::npos)
      servers.push_back(atoi(entry->d_name));
  }
  closedir(dir);

  return servers;
}

void check_errno(std::string text){
  if(errno != 0){
    char* error_message = strerror(errno);
    ROS_ERROR_STREAM(text << " errno: " << errno << " " << error_message);
    errno = 0;
  }
}

// ================================================================================
// ---------------------------------- Server --------------------------------------
// ================================================================================

const std::string Server::SERVER_DIRECTORY = "/tmp/image_servers/";
const std::string Server::SEM_NAME = "image.sem";

Server::Server(std::vector<std::string> topic_names, std::function<void(Message, int)> message_callback)
  : topic_names(topic_names)
  , message_callback(message_callback){
  
  mkdir(Server::SERVER_DIRECTORY.c_str(), 0775);

  ROS_INFO("server trying to lock");
  mutex.lock();
  ROS_INFO("server locked");
  
  std::vector<int> servers = list_servers();
  std::string socket_name = servers.empty() ? "1" : std::to_string(servers.back() + 1);//topic_names[0];
  ROS_INFO_STREAM("CREATING SERVER: " << socket_name);
  //std::replace(socket_name.begin(), socket_name.end(), '/', '%');
  
  struct sockaddr_un addr;
  socklen_t peer_addr_size;
  std::string socket_filename = Server::SERVER_DIRECTORY + socket_name;
  std::cout << "socket_filename: " << socket_filename << std::endl;

  sfd = socket(AF_UNIX, SOCK_STREAM, 0);
  if(sfd == -1)
    handle_error("Failed to create socket");

  if(unlink (socket_filename.c_str()) == -1 && errno != ENOENT)
    handle_error ("Removing socket file failed");

  memset(&addr, 0, sizeof(struct sockaddr_un));
  addr.sun_family = AF_UNIX;
  strncpy(addr.sun_path, socket_filename.c_str(), sizeof(addr.sun_path) - 1);

  if(bind(sfd, (struct sockaddr *) &addr, sizeof(struct sockaddr_un)) == -1)
    handle_error("Failed to bind to socket");

  if(listen(sfd, 5) == -1)
    handle_error("Failed to listen on socket");

  ROS_INFO("server unlocking");
  mutex.unlock();

  accept_thread = new std::thread(&Server::accept_clients, this);
  client_thread = new std::thread(&Server::poll_clients, this);
}

Server::~Server(){
  close(sfd);
}

void Server::send_fd(int client_fd, int fd) {
  struct msghdr msg = {0};
  struct cmsghdr *cmsg;
  char buf[CMSG_SPACE(sizeof(int))], *dup = "hello world";
  memset(buf, '\0', sizeof(buf));
  struct iovec io = { .iov_base = &dup, .iov_len = sizeof(dup) };

  msg.msg_iov = &io;
  msg.msg_iovlen = 1;
  msg.msg_control = buf;
  msg.msg_controllen = sizeof (buf);

  cmsg = CMSG_FIRSTHDR(&msg);
  cmsg->cmsg_level = SOL_SOCKET;
  cmsg->cmsg_type = SCM_RIGHTS;
  cmsg->cmsg_len = CMSG_LEN(sizeof(int));

  int *fdptr = (int *)CMSG_DATA(cmsg);
  *fdptr = fd;

  if (sendmsg(client_fd, &msg, 0) == -1)
    handle_error("Error sending client 1 stdout");
}

void Server::send(int client_fd, void* buffer, int length){
  ::send(client_fd , buffer, length, 0);
}

void Server::send(int client_fd, Message message){
  int size = message.get_total_byte_count();
  uint8_t* buffer = new uint8_t[size];
  std::memcpy(buffer, &message, message.get_non_payload_byte_count());
  message.payload.resize(message.payload_size);
  std::memcpy(buffer + message.get_non_payload_byte_count(), &message.payload[0], message.payload_size);
  ::send(client_fd, buffer, size, 0);
  delete buffer;
}

void Server::send_topic_names(int client_fd){
  std::string joined_topic_names;
  for(int i = 0; i < topic_names.size(); i++)
    joined_topic_names += topic_names[i] + "|";

  ROS_INFO_STREAM("TOPIC NAMES: " << joined_topic_names);
  
  int size = joined_topic_names.length();
  ::send(client_fd, &size, sizeof(size), 0);
  ::send(client_fd, &(joined_topic_names[0]), size, 0);
}

void Server::read(int client_fd, void* pointer, int length){
  int valread = ::read(client_fd, pointer, length);
}


void Server::accept_clients(){
  while(true){
    int client_fd = accept(sfd, NULL, NULL);
    send_topic_names(client_fd);
  
    client_socket_fds_mutex.lock();
    client_socket_fds.push_back(client_fd);
    //for(int i = 0; i < client_socket_fds.size(); i++)
    //  ROS_INFO_STREAM("cfd: " << i << " " << client_socket_fds[i]);
    client_socket_fds_mutex.unlock();
  }
}

void Server::poll_clients(){
  std::vector<Message> messages;
  std::vector<int> cfds;
  
  while(true){ // TODO check a running flag
    bool cleanup = false;
    
    client_socket_fds_mutex.lock();
    if(client_socket_fds.size() != pollfds.size()){
      pollfds.clear();
      for(int i = 0; i < client_socket_fds.size(); i++){
	struct pollfd pfd;
	pfd.fd = client_socket_fds[i];
	pfd.events = POLLIN;
	pollfds.push_back(pfd);
      }
    }
    client_socket_fds_mutex.unlock();
    
    int p = poll(&pollfds[0], pollfds.size(), 100);
    switch(p){
    case POLLERR:
      ROS_ERROR_STREAM("poll error " << strerror(errno));
      break;
    default:
      for(int i = 0; i < pollfds.size(); i++){
	if(pollfds[i].revents & POLLPRI ||
	   pollfds[i].revents & POLLRDHUP ||
	   pollfds[i].revents & POLLERR ||
	   pollfds[i].revents & POLLHUP){
	  close(pollfds[i].fd);
	  client_socket_fds[i] = -1;
	  cleanup = true;
	  
	  ROS_INFO_STREAM("p: " << pollfds[i].revents << " " << POLLIN
			  << " POLLIN: " << (pollfds[i].revents & POLLIN)
			  << " POLLPRI: " << (pollfds[i].revents & POLLPRI)
			  << " POLLOUT: " << (pollfds[i].revents & POLLOUT)
			  << " POLLDHUP: " << (pollfds[i].revents & POLLRDHUP)
			  << " POLLERR: " << (pollfds[i].revents & POLLERR)
			  << " POLLHUP: " << (pollfds[i].revents & POLLHUP)
			  << " POLLNVAL: " << (pollfds[i].revents & POLLNVAL));
	}
	else if(pollfds[i].revents & POLLNVAL){
	  client_socket_fds[i] = -1;
	  cleanup = true;
	  
	  ROS_INFO_STREAM("p: " << pollfds[i].revents << " " << POLLIN
			  << " POLLIN: " << (pollfds[i].revents & POLLIN)
			  << " POLLPRI: " << (pollfds[i].revents & POLLPRI)
			  << " POLLOUT: " << (pollfds[i].revents & POLLOUT)
			  << " POLLDHUP: " << (pollfds[i].revents & POLLRDHUP)
			  << " POLLERR: " << (pollfds[i].revents & POLLERR)
			  << " POLLHUP: " << (pollfds[i].revents & POLLHUP)
			  << " POLLNVAL: " << (pollfds[i].revents & POLLNVAL));
	}
	else if(pollfds[i].revents & POLLIN){
	  // TODO handle case where the full message isn't read in one call to read, ie check read return value
	  // TODO handle case where read doesn't return the full message, will this happen?
	  Message message;
	  int read_bytes = ::read(pollfds[i].fd, &message, message.get_non_payload_byte_count());
	  message.payload.resize(message.payload_size);
	  //ROS_INFO_STREAM("read bytes: " << read_bytes << " message type " << message.message_type
	  // 		  << " payload size " << message.payload_size);
	  read_bytes = ::read(pollfds[i].fd, &message.payload[0], message.payload_size);
	  //ROS_INFO_STREAM("read bytes: " << read_bytes);
	  messages.push_back(message);
	  //ROS_INFO("push1");
	  cfds.push_back(pollfds[i].fd);
	  //ROS_INFO("push2");
	}
      }
    }

    if(cleanup){
      for(int i = 0; i < client_socket_fds.size(); i++){
	client_socket_fds.erase(client_socket_fds.begin()+i);
	i--;
      }
    }
    
    //ROS_INFO_STREAM("messages: " << messages.size());
    for(int i = 0; i < messages.size(); i++)
      message_callback(messages[i], cfds[i]);
    messages.clear();
    cfds.clear();
  }
}

// ================================================================================
// ---------------------------------- Client --------------------------------------
// ================================================================================

Client::Client(std::vector<std::string> topic_names)
  : topic_names(topic_names){

  sfd = -1;
  connect_thread = NULL;
  connection_thread_running = false;
  start_connection_thread();
  //int ret = 0;
  //while ((ret = connect(sfd, (struct sockaddr *) &addr, sizeof(struct sockaddr_un))) == -1)
  //std::cout << "Failed to connect to socket" << std::endl;
  //std::cout << "CLIENT CONNECT RETURN: " << ret << std::endl;
}

Client::~Client(){
  if (close(sfd) == -1)
    handle_error("Failed to close socket");
}

void Client::start_connection_thread(){
  connect_mutex.lock();
  if(!connection_thread_running){
    ROS_INFO("deleting thread");
    if(connect_thread != NULL)
      connect_thread->join();
    delete connect_thread;
    ROS_INFO("done");
    connect_thread = new std::thread(&Client::connect_function, this);
  }
  else
    ROS_INFO("connection thread already running");
  connect_mutex.unlock();
}

void Client::connect_function(){
  connect_mutex.lock();
  connection_thread_running = true;
  connect_mutex.unlock();
  bool connected = false;

  if(sfd != -1)
    close(sfd);
  
  struct sockaddr_un addr;
  sfd = socket(AF_UNIX, SOCK_STREAM, 0);
  if(sfd == -1)
    handle_error("Failed to create socket");

  memset(&addr, 0, sizeof(struct sockaddr_un));
  addr.sun_family = AF_UNIX;
  
  ROS_INFO("client connecting...");
  while(!connected){
    mutex.lock();
    std::vector<int> servers = list_servers();
    for(int i = 0; i < servers.size(); i++){
      std::string socket_filename = Server::SERVER_DIRECTORY + std::to_string(servers[i]);
      strncpy(addr.sun_path, socket_filename.c_str(), sizeof(addr.sun_path) - 1);
      
      if(connect(sfd, (struct sockaddr *) &addr, sizeof(struct sockaddr_un)) == -1){
	ROS_INFO_STREAM("Failed to connect to " << socket_filename << ". Deleting...");
	unlink(socket_filename.c_str());
      }
      else{
	std::vector<std::string> topics = read_topic_names();
	for(int i = 0; i < topics.size(); i++)
	  ROS_INFO_STREAM("TOPIC " << i << ": " << topics[i]);
	for(int i = 0; i < topic_names.size(); i++){
	  bool found = false;
	  for(int j = 0; j < topics.size(); j++){
	    if(topic_names[i] == topics[j]){
	      found = true;
	      break;
	    }
	  }

	  if(!found){
	    ROS_INFO_STREAM("topic " << topic_names[i] << " not found");
	    // TODO handle
	  }
	}
	ROS_INFO_STREAM("Connected to " << socket_filename);
	connected = true;
      }
    }
    mutex.unlock();

  }
  ROS_INFO("connection thread exiting");
  
  connect_mutex.lock();
  connection_thread_running = false;
  connect_mutex.unlock();
}

bool Client::recv_fd(int* fd) {
  size_t nread;
  struct msghdr msg = {0};
  struct cmsghdr *cmsg;
  char buf[CMSG_SPACE(sizeof(int))], dup[256];
  memset(buf, '\0', sizeof(buf));
  struct iovec io = { .iov_base = &dup, .iov_len = sizeof(dup) };

  msg.msg_iov = &io;
  msg.msg_iovlen = 1;
  msg.msg_control = buf;
  msg.msg_controllen = sizeof(buf);

  if ( recvmsg(sfd, &msg, 0) < 0 ){
    handle_error("Failed to receive mesage");
    start_connection_thread();
    return false;
  }

  cmsg = CMSG_FIRSTHDR(&msg);
  //std::cout << "NULL? " << cmsg << std::endl;

  *fd = *(int *)CMSG_DATA(cmsg);

  return true;
}


bool Client::send(void* buffer, int length){
  int ret = ::send(sfd , buffer, length, 0);
  if(ret == -1){
    ROS_INFO_STREAM("send returned -1, errno: " << errno);
    start_connection_thread();
    return false;
  }

  return true;
}

bool Client::send(Message message){
  int size = message.get_total_byte_count();
  uint8_t* buffer = new uint8_t[size];
  std::memcpy(buffer, &message, message.get_non_payload_byte_count());
  std::memcpy(buffer + message.get_non_payload_byte_count(), &message.payload[0], message.payload_size);
  int ret = ::send(sfd, buffer, size, 0);
  delete buffer;
  if(ret == -1){
    ROS_INFO_STREAM("send returned -1, errno: " << errno);
    start_connection_thread();
    return false;
  }

  return true;
}

bool Client::read(void* pointer, int length){
  int valread = ::read(sfd, pointer, length);
  if(valread == 0){
    ROS_INFO("read returned 0");
    start_connection_thread();
    return false;
  }

  return true;
}

std::vector<std::string> Client::read_topic_names(){
  std::vector<std::string> topic_names;
  int size;
  int valread = ::read(sfd, &size, sizeof(size));
  if(valread == 0)
    ROS_INFO("read returned 0 (read_topic_anmes)");
  std::string joined_topic_names(size, ' ');
  ROS_INFO("2read...");
  valread = ::read(sfd, &(joined_topic_names[0]), size);
  if(valread == 0)
    ROS_INFO("read returned 0 (read_topic_anmes)");

  topic_names.push_back(std::string(""));
  for(int i = 0; i < joined_topic_names.length(); i++){
    if(joined_topic_names[i] != '|')
      topic_names.back() += joined_topic_names[i];
    else
      topic_names.push_back(std::string(""));
  }
  topic_names.pop_back();

  return topic_names;
}


// ================================================================================
// ----------------------------------- Mutex --------------------------------------
// ================================================================================

Mutex::Mutex(){
  ROS_INFO_STREAM("sem name: " << (std::string("/") + Server::SEM_NAME).c_str());
  errno = 0;
  sem_fd = sem_open((std::string("/")  + Server::SEM_NAME).c_str(), O_CREAT, 0644, 1);
  check_errno("sem_open");
}

void Mutex::lock(){
  errno = 0;
  sem_wait(sem_fd);
  check_errno("sem_wait");
  has_lock = true;
}

void Mutex::unlock(){
  errno = 0;
  sem_post(sem_fd);
  check_errno("sem_post");
  has_lock = false;
}

bool Mutex::owns_lock(){
  return has_lock;
}
