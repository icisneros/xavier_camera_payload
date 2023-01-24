#ifndef _NV_SERVER_CLIENT_H_
#define _NV_SERVER_CLIENT_H_

#include <ros/ros.h>
#include <unistd.h>
#include <stdio.h>
#include <dirent.h>
#include <sys/socket.h>
#include <sys/stat.h>
#include <arpa/inet.h>
#include <stdlib.h>
#include <netinet/in.h>
#include <string>
#include <sys/uio.h>
#include <cstring>
#include <sys/un.h>
#include <thread>
#include <mutex>
#include <vector>
#include <poll.h>
#include <errno.h>
#include <fcntl.h>
#include <semaphore.h>

// TODO put everything nv2ros namespace

// TODO make this a function
#define handle_error(msg) do { perror(msg); exit(EXIT_FAILURE); } while(0)

  
class Serializable {
public:
  virtual std::vector<uint8_t> serialize(){
    ROS_ERROR_STREAM("BASE SERIALIZE FUNCTION CALLED");
  }
  
  virtual void deserialize(std::vector<uint8_t> bytes){
    ROS_ERROR_STREAM("BASE DESERIALIZE FUNCTION CALLED");
  }
};


class SerializationHelper {
private:
  std::vector<uint8_t> bytes;
  
public:
  SerializationHelper(){}
  SerializationHelper(std::vector<uint8_t> b)
    : bytes(b){
    
  }
    
  template<typename T>
  void push(T t){
    int index = bytes.size();
    bytes.resize(bytes.size() + sizeof(t), 0);
    std::memcpy(&bytes[index], &t, sizeof(t));
  }

  template<typename T>
  T pop(){
    T t;
    std::memcpy(&t, &bytes[bytes.size() - sizeof(t)], sizeof(t));
    bytes.erase(bytes.end() - sizeof(t), bytes.end());

    return t;
  }

  std::vector<uint8_t> get_bytes(){
    return bytes;
  }
    
};

struct Message {
  int message_type, payload_size;
  std::vector<uint8_t> payload;
  
  // TODO move implementations to cpp file
  Message(int message_type, Serializable& s)
    : message_type(message_type){
    payload = s.serialize();
    payload_size = payload.size();
  }

  Message(){
    
  }
  
  int get_total_byte_count(){
    return sizeof(message_type) + sizeof(payload_size) + payload.size();
  }
  
  int get_non_payload_byte_count(){
    return sizeof(message_type) + sizeof(payload_size);
  }
};


// ================================================================================
// ----------------------------------- Mutex --------------------------------------
// ================================================================================

class Mutex {
private:
  sem_t* sem_fd;
  bool has_lock;
  
public:
  Mutex();
  void lock();
  void unlock();
  bool owns_lock();
};

// ================================================================================
// ---------------------------------- Server --------------------------------------
// ================================================================================

class Server {
private:
  std::vector<std::string> topic_names;
  std::function<void(Message, int)> message_callback;
  int sfd;
  Mutex mutex;
  std::thread* accept_thread;
  std::thread* client_thread;
  std::mutex client_socket_fds_mutex;
  std::vector<int> client_socket_fds;
  std::vector<struct pollfd> pollfds;
  
  void accept_clients();
  void poll_clients();
  
public:
  Server(std::vector<std::string> topic_names, std::function<void(Message, int)> message_callback=NULL);
  ~Server();
  void send_fd(int client_fd, int fd);
  void send(int client_fd, void* buffer, int length);
  void send_topic_names(int client_fd);
  void send(int client_fd, Message message);
  void read(int client_fd, void* pointer, int length);
  
  std::vector<Message> get_messages();

  static const std::string SERVER_DIRECTORY;
  static const std::string SEM_NAME;
};

// ================================================================================
// ---------------------------------- Client --------------------------------------
// ================================================================================

class Client {
private:
  int sfd;
  Mutex mutex;
  std::vector<std::string> topic_names;

  bool connection_thread_running;
  std::mutex connect_mutex;
  std::thread* connect_thread;
  void start_connection_thread();
  void connect_function();
  void set_connected(bool status);
  
public:
  Client(std::vector<std::string> topic_names);
  ~Client();
  bool recv_fd(int* fd);
  bool send(void* buffer, int length);
  bool send(Message message);
  bool read(void* pointer, int length);
  std::vector<std::string> read_topic_names();
};

#endif
