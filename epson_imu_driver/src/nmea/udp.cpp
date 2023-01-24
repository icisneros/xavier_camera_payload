#include "udp.h"

using namespace boost::asio;

UDPSend::UDPSend(std::string remote_ip, int port) : remote_port(port)
{
	remote_endpoint = udp::endpoint(ip::address::from_string(remote_ip), remote_port);
	try {
		sock = new udp::socket(io_service, udp::endpoint(udp::v4(), 4000));
	} catch (std::exception& e) {
		printf("%s", e.what());
	}
}

void UDPSend::send(std::string str) 
{
	boost::shared_ptr<std::string> message(new std::string(str));
	printf("Message: %s", str.c_str());
    sock->send_to(boost::asio::buffer(str), remote_endpoint, 0, error);
    printf("Result code: %s\n", error.message().c_str());
}

