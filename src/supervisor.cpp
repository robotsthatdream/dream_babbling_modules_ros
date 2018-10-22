#include "ros/ros.h"
#include <ros/rate.h>
#include <ros/node_handle.h>

#include "modules_finder.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "modules_node");
    ros::NodeHandle n("modules");
    ros::NodeHandle private_node_handle("~");

    ModulesFinder mf(&n);

    std::string iface;
    private_node_handle.param<std::string>("iface", iface, std::string("wlan0"));
    mf.setInterface(iface);

    int port;
    private_node_handle.param<int>("port", port, BAB_LOCAL_UDP_PORT);
    mf.setLocalPort(port);

    int dist_port;
    private_node_handle.param<int>("dist_port", dist_port, BAB_ARDUINO_UDP_PORT);
    mf.setDistantPort(port);

    mf.open();

    ros::Rate rate(100);
    mf.start();
    unsigned int i = 0;

    while (ros::ok()) {
        if (++i % 100 == 0) {
            mf.scan();
            i = 0;
        }
        ros::spinOnce();
        rate.sleep();
    }

    mf.stop();

    mf.close();

    return 0;
}
