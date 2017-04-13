#ifndef _MODULES_DEFS_H_
#define _MODULES_DEFS_H_

#define BAB_CMD_WHOAREYOU 0
#define BAB_CMD_WHOIAM 1
#define BAB_CMD_START 2
#define BAB_CMD_STOP 3
#define BAB_CMD_REPORT_STATE 4
#define BAB_CMD_PING 5
#define BAB_CMD_DISPLAY_COLOR 6
#define BAB_CMD_DISPLAY_FILE 7
#define BAB_CMD_LED 8
#define BAB_CMD_TACTILE_POS 9

#define BAB_TYPE_MODULE 0

#define BAB_MODULE_TYPE_BUTTON 0
#define BAB_MODULE_TYPE_SCREEN 1
#define BAB_MODULE_TYPE_RFID 2
#define BAB_MODULE_TYPE_JOYSTICK 3

#define BAB_LOCAL_UDP_PORT 25000
#define BAB_ARDUINO_UDP_PORT 25000

#define BAB_WIFI_SSID "dream_babbling"
#define BAB_WIFI_PWD "dreambab"

#define BAB_MAX_PACKET_SIZE 64

extern const char* id_to_cmd[];
extern const char* id_to_type[];
extern const char* id_to_module_type[];

#endif // _MODULES_DEFS_H_
