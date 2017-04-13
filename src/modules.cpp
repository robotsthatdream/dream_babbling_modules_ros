#include "dream_babbling_modules/modules.h"

#include <geometry_msgs/Vector3.h>

Module::Module(uint8_t *mac, struct sockaddr module_sa, int sockfd, ros::NodeHandle *nh, double timeout) : _sa(module_sa), _sockfd(sockfd), _nh(nh), _timeout(timeout)
{
}

Module::~Module()
{
};

int Module::process(char *msg, ssize_t sz)
{
    updateLastMessageTime();
    return 1;
}

int Module::start()
{
    updateLastMessageTime();
    char msg[4];
    msg[0] = BAB_CMD_START;
    return sendto(_sockfd, msg, 1, 0, &_sa, sizeof(_sa));
}

int Module::stop()
{
    char msg[4];
    msg[0] = BAB_CMD_STOP;
    return sendto(_sockfd, msg, 1, 0, &_sa, sizeof(_sa));
}

bool Module::isOnline()
{
    struct timespec tp;
    clock_gettime(CLOCK_MONOTONIC, &tp);
    double timediff = tp.tv_sec + tp.tv_nsec / 1000000000.0 - _last_message_time;
    if (_timeout > 0 && (timediff > _timeout)) {
        uint32_t ip = reinterpret_cast<struct sockaddr_in *>(&_sa)->sin_addr.s_addr;
        std::cout << "Module " << ip << " disconnected" << std::endl;
        return false;
    }
    return true;
}

void Module::updateLastMessageTime()
{
    struct timespec tp;
    clock_gettime(CLOCK_MONOTONIC, &tp);
    _last_message_time = tp.tv_sec + tp.tv_nsec / 1000000000.0;
}

ButtonModule::ButtonModule(uint8_t *mac, struct sockaddr module_sa, int sockfd, ros::NodeHandle *nh) : Module(mac, module_sa, sockfd, nh, 1.0)
{
    char hex_mac[32] = {0};
    sprintf(hex_mac, "%02x_%02x_%02x_%02x_%02x_%02x", mac[5], mac[4], mac[3], mac[2], mac[1], mac[0]);
    const std::string button_state_pub_name = std::string("ButtonModule_").append(std::string(hex_mac).append("/button_pressed"));
    _button_state_pub = _nh->advertise<std_msgs::Bool> (button_state_pub_name, 5);
    const std::string button_led_sub_name = std::string("ButtonModule_").append(std::string(hex_mac).append("/led"));
    _button_led_sub = _nh->subscribe<std_msgs::Bool> (button_led_sub_name, 5, &ButtonModule::_buttonLedSubCallback, this);
}

int ButtonModule::process(char *msg, ssize_t sz)
{
    if (sz < 0) {
        return -1;
    }

    if (msg[0] == BAB_CMD_REPORT_STATE) {
        std_msgs::Bool tmp;
        tmp.data = msg[1] != 0;
        _button_state_pub.publish(tmp);
    }

    return Module::process(msg, sz);
}

void ButtonModule::_buttonLedSubCallback(const std_msgs::Bool::ConstPtr &led)
{
    char msg[2] = {0};
    msg[0] = BAB_CMD_LED;
    msg[1] = led->data ? 1 : 0;
    if (sendto(_sockfd, msg, 2, 0, &_sa, sizeof(_sa)) < 0) {
        std::cerr << strerror(errno) << std::endl;
    }
}

ScreenModule::ScreenModule(uint8_t *mac, struct sockaddr module_sa, int sockfd, ros::NodeHandle *nh) : Module(mac, module_sa, sockfd, nh, 10.0)
{
    char hex_mac[32] = {0};
    sprintf(hex_mac, "%02x_%02x_%02x_%02x_%02x_%02x", mac[5], mac[4], mac[3], mac[2], mac[1], mac[0]);
    const std::string color_sub_name = std::string("ScreenModule_").append(std::string(hex_mac).append("/color"));
    _color_sub = _nh->subscribe<std_msgs::ColorRGBA> (color_sub_name, 5, &ScreenModule::_colorSubCallback, this);
    const std::string file_sub_name = std::string("ScreenModule_").append(std::string(hex_mac).append("/file"));
    _file_sub = _nh->subscribe<std_msgs::String> (file_sub_name, 5, &ScreenModule::_fileSubCallback, this);
    const std::string tactile_pos_pub_name = std::string("ScreenModule_").append(std::string(hex_mac).append("/tactile_pos"));
    _tactile_pos_pub = _nh->advertise<geometry_msgs::Point> (tactile_pos_pub_name, 5);
    const std::string tactile_pressed_pub_name = std::string("ScreenModule_").append(std::string(hex_mac).append("/tactile_pressed"));
    _tactile_pressed_pub = _nh->advertise<std_msgs::Bool> (tactile_pressed_pub_name, 5);
}

int ScreenModule::process(char *msg, ssize_t sz)
{
    if (sz < 0) {
        return -1;
    }

    if (msg[0] == BAB_CMD_TACTILE_POS) {
        geometry_msgs::Point tmp;
	uint16_t x, y;
	memcpy(&x,&msg[1],sizeof(uint16_t));
	memcpy(&y,&msg[3],sizeof(uint16_t));
        tmp.x = x;
	tmp.y = y;
	tmp.z = 0;
        _tactile_pos_pub.publish(tmp);
	
	std_msgs::Bool pressed;
	pressed.data = fabs(x) > 30 && fabs(y) > 30;
	_tactile_pressed_pub.publish(pressed);
    }
    return Module::process(msg, sz);
}

void ScreenModule::_colorSubCallback(const std_msgs::ColorRGBA::ConstPtr &color)
{
    char msg[4] = {0};
    msg[0] = BAB_CMD_DISPLAY_COLOR;
    msg[1] = static_cast<uint8_t>(color->r);
    msg[2] = static_cast<uint8_t>(color->g);
    msg[3] = static_cast<uint8_t>(color->b);
    if (sendto(_sockfd, msg, 4, 0, &_sa, sizeof(_sa)) < 0) {
        std::cerr << strerror(errno) << std::endl;
        return;
    }
}

void ScreenModule::_fileSubCallback(const std_msgs::String::ConstPtr &filename)
{
    char msg[64] = {0};
    if (filename->data.length() > 62) {
        return;
    }
    msg[0] = BAB_CMD_DISPLAY_FILE;
    strcpy(&msg[1], filename->data.c_str());
    if (sendto(_sockfd, msg, filename->data.length() + 2, 0, &_sa, sizeof(_sa)) < 0) {
        std::cerr << strerror(errno) << std::endl;
        return;
    }
}

RFIDModule::RFIDModule(uint8_t *mac, struct sockaddr module_sa, int sockfd, ros::NodeHandle *nh) : Module(mac, module_sa, sockfd, nh, 1.0)
{
    char hex_mac[32] = {0};
    sprintf(hex_mac, "%02x_%02x_%02x_%02x_%02x_%02x", mac[5], mac[4], mac[3], mac[2], mac[1], mac[0]);
    const std::string tag_uid_pub_name = std::string("RFIDModule_").append(std::string(hex_mac).append("/tag_uid"));
    _tag_uid_pub = _nh->advertise<std_msgs::UInt64> (tag_uid_pub_name, 5);
    const std::string tag_present_pub_name = std::string("RFIDModule_").append(std::string(hex_mac).append("/tag_present"));
    _tag_present_pub = _nh->advertise<std_msgs::Bool> (tag_present_pub_name, 5);
}

int RFIDModule::process(char *msg, ssize_t sz)
{
  if(sz < 0) {
    return -1;
  }
  
    if (msg[0] == BAB_CMD_REPORT_STATE) {
        uint8_t uid_sz = msg[1];
        std_msgs::UInt64 tmp;
	std_msgs::Bool tmp2;
        tmp.data = 0;
        if (uid_sz > 0) {
            for (unsigned int i = 0; i < uid_sz; i++) {
                tmp.data = (tmp.data << 8) | (msg[2 + i] & 0xff);
            }
        }
        tmp2.data = tmp.data != 0;
        _tag_uid_pub.publish(tmp);
	_tag_present_pub.publish(tmp2);
    }
    return Module::process(msg, sz);
}

JoystickModule::JoystickModule(uint8_t *mac, struct sockaddr module_sa, int sockfd, ros::NodeHandle *nh) : Module(mac, module_sa, sockfd, nh, 1.0)
{
    char hex_mac[32] = {0};
    sprintf(hex_mac, "%02x_%02x_%02x_%02x_%02x_%02x", mac[5], mac[4], mac[3], mac[2], mac[1], mac[0]);
    const std::string button_state_pub_name = std::string("JoystickModule_").append(std::string(hex_mac).append("/button_pressed"));
    _button_state_pub = _nh->advertise<std_msgs::Bool> (button_state_pub_name, 5);
    const std::string joystick_state_pub_name = std::string("JoystickModule_").append(std::string(hex_mac).append("/joystick_position"));
    _joystick_state_pub = _nh->advertise<geometry_msgs::Vector3> (joystick_state_pub_name, 5);
}

int JoystickModule::process(char *msg, ssize_t sz)
{
  if(sz < 6) {
    return -1;
  }
  
    if (msg[0] == BAB_CMD_REPORT_STATE) {
        std_msgs::Bool tmp;
	geometry_msgs::Vector3 tmp2;
	uint16_t x,y;
	
        memcpy(&x,&msg[2],sizeof(uint16_t));
        memcpy(&y,&msg[4],sizeof(uint16_t));
	
	tmp.data = msg[1] == 1;
	tmp2.x = x;
	tmp2.y = y;
	tmp2.z = 0;
	
        _button_state_pub.publish(tmp);
	_joystick_state_pub.publish(tmp2);
    }
    return Module::process(msg, sz);
}

Module *ModuleFactory::fromID(int id, uint8_t *mac, struct sockaddr module_sa, int sockfd, ros::NodeHandle *nh)
{
    switch (id) {
    case BAB_MODULE_TYPE_BUTTON:
        return new ButtonModule(mac, module_sa, sockfd, nh);
        break;
    case BAB_MODULE_TYPE_SCREEN:
        return new ScreenModule(mac, module_sa, sockfd, nh);
        break;
    case BAB_MODULE_TYPE_RFID:
        return new RFIDModule(mac, module_sa, sockfd, nh);
        break;
    case BAB_MODULE_TYPE_JOYSTICK:
        return new JoystickModule(mac, module_sa, sockfd, nh);
        break;
    default:
        return NULL;
        break;
    }
}
