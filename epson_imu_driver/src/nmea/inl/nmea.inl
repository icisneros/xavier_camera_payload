namespace nmea {

/// @brief default constructure for params
template<typename T>
NmeaConnection<T>::Params::Params():
  use_ethernet(true),
  velodyne_ip("192.168.1.201"),
  velodyne_port(10110),
  nmea_baudrate(9600),
  nmea_port("/dev/ttyUSB1"),
  use_chrony(true),
  chrony_sock("/var/run/chrony.sock"),
  chrony_latency(0.01)
{}

/// @brief default constructor
template<typename T>
NmeaConnection<T>::NmeaConnection(const NmeaConnection<T>::Params& params)
  : use_chrony_(false) {

  // set the params
  set_params(params);

  // create the connection type
  create();
}

 /// @brief exposed send, hiding the connection type
template<typename T>
void NmeaConnection<T>::send(std::string msg) {
  // send the message over the connection
  _send_message(msg);
  
  /// TODO
  // if using chrony, send chrony message
  // if (use_chrony_) {
  //   struct timeval tv;
  //   tv.tv_sec = floor(computerTime);
  //   tv.tv_usec = fmod(computerTime, 1.0) * 1.0e6;
  //   chrony->Send(tv, computerSec - computerTime + chrony_latency);
  // }
  return;
}

/// @brief setup the parameters
template<typename T>
void NmeaConnection<T>::set_params(const NmeaConnection<T>::Params& params) {
  // velodyne params
  use_ethernet_ = params.use_ethernet;
  velodyne_ip_ = params.velodyne_ip;
  velodyne_port_ = params.velodyne_port;
  // nmea params
  nmea_baudrate_ = params.nmea_baudrate;
  nmea_port_ = params.nmea_port;
  /// chrony params
  use_chrony_ = params.use_chrony;
  chrony_sock_ = params.chrony_sock;
  chrony_latency_ = params.chrony_latency;
}

// /////////////////////////////////////////////////////////////////////////////
/// @brief create the connection type
// /////////////////////////////////////////////////////////////////////////////

/// @brief udp connection
template <> void NmeaConnection<UDPSend>::create() {
  conn_ = new UDPSend(velodyne_ip_, velodyne_port_);
}
/// @brief serial connection
template <> void NmeaConnection<TimeoutSerial>::create() {
  conn_ = new TimeoutSerial(nmea_port_, nmea_baudrate_);
  conn_->setTimeout(boost::posix_time::seconds(5));  	
}

// /////////////////////////////////////////////////////////////////////////////
/// @brief send the nmea message over the connection
// /////////////////////////////////////////////////////////////////////////////

/// @brief over udp connection
template <> void NmeaConnection<UDPSend>::_send_message(std::string msg) {
  conn_->send(msg);
}

/// @brief over serial connection
template <> void NmeaConnection<TimeoutSerial>::_send_message(std::string msg) {
  std::cout << "Sending a SERIAL connection in port : " 
				    << nmea_port_ << " and on baudrate: " << nmea_baudrate_ << std::endl;
  std::cout << "sending: " << msg << std::endl;          
  conn_->writeString(msg);
}


}; // namespace nmea

