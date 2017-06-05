
#ifndef __USER_CONFIG_H__
#define __USER_CONFIG_H__
#include "ets_sys.h"
#include "osapi.h"
#include "user_interface.h"
#include "espconn.h"

#define NOW_TIME()  READ_PERI_REG(0x3ff20C00)
#define USER_PARAMETERS_SECTOR 0x7E
#define on 0xff
#define off 0x00
#define LOCAL_PORT  5000
#define REMOTE_PORT  5001
typedef enum {
    FLASH_ALLWAYS_ON = 0,
    FLASH_OFF,
    SLOW_FLASH,
    QUICK_FLASH
} FLASH_STATUSES;

typedef enum{
    ADJUST = 0xAAAAAAAA,
    ACKNOWLEDGE = 0xBBBBBBBB,
    LOOKUP = 0xCCCCCCCC,
    FOUND = 0xDDDDDDDD,

}UDP_PROTOCOL_HEADER;
struct  UDP_PROTOCOL
{
    uint32 header;
    uint32 id;
    uint8 status;
    uint8 brightness;
    uint8 CRC[2];
};
LOCAL struct espconn esp_conn;
LOCAL esp_udp espudp;
LOCAL bool FADE = false;
LOCAL bool FADE_IN = 0xff;
LOCAL uint32 CHIPID = 0;
LOCAL bool clicked = false;
LOCAL uint32 ACROSS_ZERO_TURN_ON_TIME = 0;
LOCAL uint8 BRIGHT_LEVEL = 50;
LOCAL uint8 BRIGHT_LEVEL_INTERVAL = 10;
LOCAL uint8 TRIGGER_DELAY = 100;
LOCAL uint8 LIGHT_STATUS = on;
LOCAL FLASH_STATUSES FLASH_STATUS = FLASH_ALLWAYS_ON;
#endif

