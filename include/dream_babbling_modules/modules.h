#ifndef _MODULES_H_
#define _MODULES_H_

#include "modules_defs.h"

#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <std_msgs/ColorRGBA.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Point.h>

#include <sys/socket.h>
#include <netinet/in.h>

#include <time.h>

/**
 * @brief Base class for all modules
 *
 */
class Module
{
public:
    /**
     * @brief Constructor
     *
     * @param mac The module's mac address; The array must be at least 6 bytes long.
     * @param module_sa sockaddr of the module (basically the module's ip address).
     * @param sockfd The socket's file descriptor.
     * @param nh The current ROS node handle.
     * @param timeout The maximum amount of time we can wait after a module's last message before we assume it's dead.
     */
    Module ( uint8_t *mac, struct sockaddr module_sa, int sockfd, ros::NodeHandle *nh, double timeout = 0 );

    /**
     * @brief Virtual destructor.
     *
     */
    virtual ~Module();

    /**
     * @brief Basic packet processing method, all it does is refreshing the timeout.
     *
     * @param msg The received message, as an array of char.
     * @param sz The size of msg.
     * @return 1 in case of success, a negative value indicates an error.
     */
    virtual int process ( char *msg, ssize_t sz );

    /**
     * @brief This method is called to enable a module (ie. make it start streaming data, processing messages, etc...).
     *
     * @return -1 in case of errors (check errno)
     */
    int start();

    /**
     * @brief This method is called to disable a module.
     *
     * @return -1 in case of errors (check errno)
     */
    int stop();


    /**
     * @brief Check a module's current state.
     *
     * @return false if the module has timed out, true otherwise.
     */
    bool isOnline();

protected:
    void updateLastMessageTime();

    struct sockaddr _sa;
    int _sockfd;

    ros::NodeHandle *_nh;

    double _last_message_time;
    double _timeout;
};

/**
 * @brief Module using a pushbutton
 *
 */
class ButtonModule: public Module
{
public:
    ButtonModule ( uint8_t *mac, struct sockaddr module_sa, int sockfd, ros::NodeHandle *nh );

    ~ButtonModule() {
    }

    int process ( char *msg, ssize_t sz );

protected:
    void _buttonLedSubCallback ( const std_msgs::Bool::ConstPtr &led );

    ros::Publisher _button_state_pub;
    ros::Subscriber _button_led_sub;
};

/**
 * @brief Module using a tactile display
 *
 */
class ScreenModule: public Module
{
public:
    ScreenModule ( uint8_t *mac, struct sockaddr module_sa, int sockfd, ros::NodeHandle *nh );

    ~ScreenModule() {
    }

    int process ( char *msg, ssize_t sz );

protected:
    void _colorSubCallback ( const std_msgs::ColorRGBA::ConstPtr &color );

    void _fileSubCallback ( const std_msgs::String::ConstPtr &filename );

    ros::Subscriber _color_sub;
    ros::Subscriber _file_sub;
    ros::Publisher _tactile_pos_pub;
    ros::Publisher _tactile_pressed_pub;
};

/**
 * @brief Module using a RFID sensor
 *
 */
class RFIDModule: public Module
{
public:
    RFIDModule ( uint8_t *mac, struct sockaddr module_sa, int sockfd, ros::NodeHandle *nh );

    ~RFIDModule() {
    }

    int process ( char *msg, ssize_t sz );

protected:
    ros::Publisher _tag_uid_pub;
    ros::Publisher _tag_present_pub;
};

class JoystickModule: public Module
{
public:
    JoystickModule ( uint8_t *mac, struct sockaddr module_sa, int sockfd, ros::NodeHandle *nh );

    ~JoystickModule() {
    }

    int process ( char *msg, ssize_t sz );

protected:
    ros::Publisher _button_state_pub;
    ros::Publisher _joystick_state_pub;
};

class LeverModule: public Module
{
public:
    LeverModule ( uint8_t *mac, struct sockaddr module_sa, int sockfd, ros::NodeHandle *nh );

    ~LeverModule() {
    }

    int process ( char *msg, ssize_t sz );

protected:
    ros::Publisher _lever_state_pub;
};

/**
 * @brief Factory used to create modules from the identifier they return when asked what they are
 *
 */
class ModuleFactory
{
public:
    static Module *fromID ( int id, uint8_t *mac, struct sockaddr module_sa, int sockfd, ros::NodeHandle *nh );
};

#endif // _MODULES_H_
