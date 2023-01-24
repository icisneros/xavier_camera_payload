#include <boost/array.hpp>
#include <boost/bind.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/asio.hpp>
#include <boost/thread.hpp>

using boost::asio::ip::udp;

class UDPSend {
private:
	int remote_port;
	udp::socket * sock;
	udp::endpoint remote_endpoint;
	boost::asio::io_service io_service;
	boost::system::error_code error;
public:
	UDPSend(std::string remote_ip, int port);
	void send(std::string str);
};
