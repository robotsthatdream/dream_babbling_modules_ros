#ifndef _MODULES_FINDER_H_
#define _MODULES_FINDER_H_

#include <unistd.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <ifaddrs.h>
#include <arpa/inet.h>
#include <fcntl.h>

#include <thread>
#include <mutex>

#include <ros/ros.h>

#include "modules.h"

/**
 * @brief This class is used to scan a network for modules and then dispatch incoming packets to them.
 * 
 */
class ModulesFinder
{
public:
  /**
   * @brief Constructor.
   * 
   * @param nh The node's handle
   */
  ModulesFinder(ros::NodeHandle* nh);

  /**
   * @brief Sets the interface to listen on.
   * 
   * @param iface A string containing the interface's name (like wlan0, wlan1, eth0, etc..)
   * @return void
   */
  void setInterface(const std::string& iface);
  
  /**
   * @brief Sets the local udp port.
   * 
   * @param port The port to use
   * @return void
   */
  void setLocalPort(const int& port);
  
  /**
   * @brief Sets the modules' udp port.
   * 
   * @param port The port to use
   * @return void
   */
  void setDistantPort(const int& port);

  /**
   * @brief Configures the socket and opens it. Must be called after setInterface and setLocalPort.
   * 
   * @return void
   */
  void open();
  
  /**
   * @brief Closes the socket.
   * 
   * @return void
   */
  void close();

  /**
   * @brief Scans the network for modules and add them to the _modules list if they aren't in it already. Must be called after setDistantPort and open.
   * 
   * @return void
   */
  void scan();

  /**
   * @brief Unlocks the thread used to process incoming packets.
   * 
   * @return void
   */
  void start();
  
  /**
   * @brief Stops the processing thread. Once stopped it cannot be started again.
   * 
   * @return void
   */
  void stop();

private:
  /**
   * @brief This method is executed in the _socket_reader_thread, it dispatches all incoming packets to the corresponding modules.
   * 
   * @return void
   */
  void _process();
  
  ros::NodeHandle* _nh;

  int _sockfd;

  std::string _iface_name;
  int _port;
  int _dist_port;

  struct sockaddr_in* _local_sa;
  struct sockaddr_in* _local_netmask;

  std::map<uint32_t,Module*> _modules;

  std::mutex _socket_mutex;
  volatile bool _thread_loop_condition;
  std::thread _socket_reader_thread;
};

#endif // _MODULES_FINDER_H_
