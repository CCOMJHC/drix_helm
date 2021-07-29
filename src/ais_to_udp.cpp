#include "ros/ros.h"
#include "mdt_msgs/StampedString.h"

#include <sys/types.h>
#include <sys/socket.h>
#include <netdb.h>
#include <unistd.h>

int out_socket = -1;

void aisCallback(const mdt_msgs::StampedString::ConstPtr& msg)
{
  send(out_socket, &(msg->data.front()), msg->data.size(), 0);
}

void connect(std::string const &host, uint16_t port)
{
  struct addrinfo hints = {0}, *addresses;
    
  hints.ai_family = AF_UNSPEC;
  hints.ai_socktype = SOCK_DGRAM;
  hints.ai_protocol = IPPROTO_UDP;
  
  std::string port_string = std::to_string(port);
  
  int ret = getaddrinfo(host.c_str(), port_string.c_str(), &hints, &addresses);
  if(ret != 0)
      throw std::runtime_error(gai_strerror(ret));
    
  int error {0};
  for (struct addrinfo *address = addresses; address != nullptr; address = address->ai_next)
  {
    out_socket = socket(address->ai_family, address->ai_socktype, address->ai_protocol);
    if(out_socket == -1)
    {
      error = errno;
      continue;
    }
        
    if(connect(out_socket, address->ai_addr, address->ai_addrlen) == 0)
    {
      break;
    }
      
    error = errno;
    close(out_socket);
    out_socket = -1;
  }
  freeaddrinfo(addresses);
  if(out_socket == -1)
    throw std::runtime_error(strerror(error));

}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "ais_to_udp");
  ros::NodeHandle n;

  std::string host = ros::param::param<std::string>("~host", "localhost");
  uint16_t port = ros::param::param("~port", 4010);

  connect(host, port);

  ros::Subscriber ais_sub = n.subscribe("/raw_ais", 5, aisCallback);
  ros::spin();

  return 0;
}