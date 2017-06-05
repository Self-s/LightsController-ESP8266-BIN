/*
 * ESPRSSIF MIT License
 *
 * Copyright (c) 2016 <ESPRESSIF SYSTEMS (SHANGHAI) PTE LTD>
 *
 * Permission is hereby granted for use on ESPRESSIF SYSTEMS ESP8266 only, in which case,
 * it is free of charge, to any person obtaining a copy of this software and associated
 * documentation files (the "Software"), to deal in the Software without restriction, including
 * without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense,
 * and/or sell copies of the Software, and to permit persons to whom the Software is furnished
 * to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all copies or
 * substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS
 * FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR
 * COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER
 * IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
 * CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 *
 */

#include "ets_sys.h"
#include "osapi.h"
#include "user_interface.h"
#include "spi_flash.h"
#include "espconn.h"
#include "hw_timer.h"
#include "smartconfig.h"
#include "user_config.h"


/******************************************************************************
 * FunctionName : user_rf_cal_sector_set
 * Description  : SDK just reversed 4 sectors, used for rf init data and paramters.
 *                We add this function to force users to set rf cal sector, since
 *                we don't know which sector is free in user's application.
 *                sector map for last several sectors : ABCCC
 *                A : rf cal
 *                B : rf init data
 *                C : sdk parameters
 * Parameters   : none
 * Returns      : rf cal sector
*******************************************************************************/
uint32 ICACHE_FLASH_ATTR
user_rf_cal_sector_set(void)
{
    enum flash_size_map size_map = system_get_flash_size_map();
    uint32 rf_cal_sec = 0;

    switch (size_map) {
        case FLASH_SIZE_4M_MAP_256_256:
            rf_cal_sec = 128 - 5;
            break;

        case FLASH_SIZE_8M_MAP_512_512:
            rf_cal_sec = 256 - 5;
            break;

        case FLASH_SIZE_16M_MAP_512_512:
        case FLASH_SIZE_16M_MAP_1024_1024:
            rf_cal_sec = 512 - 5;
            break;

        case FLASH_SIZE_32M_MAP_512_512:
        case FLASH_SIZE_32M_MAP_1024_1024:
            rf_cal_sec = 1024 - 5;
            break;

        default:
            rf_cal_sec = 0;
            break;
    }

    return rf_cal_sec;
}

void ICACHE_FLASH_ATTR
user_rf_pre_init(void)
{
}
void ICACHE_FLASH_ATTR user_restore(){
    ETS_FRC1_INTR_DISABLE();
    ETS_GPIO_INTR_DISABLE();
    spi_flash_erase_sector(USER_PARAMETERS_SECTOR);
    system_restart();
}
void ICACHE_FLASH_ATTR light_up()
{
    FLASH_STATUS = FLASH_ALLWAYS_ON;
    LIGHT_STATUS = on;
    FADE_IN = 0xff;
    BRIGHT_LEVEL = 50;
    gpio_pin_intr_state_set(2,GPIO_PIN_INTR_ANYEDGE);
}
void ICACHE_FLASH_ATTR light_off()
{
    FLASH_STATUS = FLASH_OFF;
    LIGHT_STATUS = off;
    gpio_pin_intr_state_set(2,GPIO_PIN_INTR_DISABLE);
    GPIO_DIS_OUTPUT(13);
    BRIGHT_LEVEL = 0;
}
os_timer_t restore_timer;
void restore_timer_callback(){
    if(GPIO_INPUT_GET(5)==0 && BRIGHT_LEVEL==90)
        user_restore();
}

void hw_timer_callback(void)
{
	static uint16 timer_cnt = 0;
    uint32 now = NOW_TIME();
	
    if(++timer_cnt>60000) timer_cnt = 1;

    if(LIGHT_STATUS){
    	if(now > ACROSS_ZERO_TURN_ON_TIME && now < ACROSS_ZERO_TURN_ON_TIME+TRIGGER_DELAY){
    		GPIO_OUTPUT_SET(13,1);
    	}else 
            GPIO_DIS_OUTPUT(13);
        if(FADE){
            if(timer_cnt%500==0){
                if(FADE_IN==0xff){
                    BRIGHT_LEVEL++;
                    if(BRIGHT_LEVEL>90) {
                        BRIGHT_LEVEL = 90;
                        FADE = false;
                        os_timer_disarm(&restore_timer);
                        os_timer_setfn(&restore_timer,
                            (os_timer_func_t*)&restore_timer_callback,NULL);
                        os_timer_arm(&restore_timer,3000,false);
                    }
                }
                else{
                    BRIGHT_LEVEL--;
                    if(BRIGHT_LEVEL<20) BRIGHT_LEVEL=20;
                }
            }
        }
    }
	switch(FLASH_STATUS){     
        case FLASH_OFF:
            GPIO_OUTPUT_SET(4,1);
        break;
    	case FLASH_ALLWAYS_ON:
    		GPIO_OUTPUT_SET(4,0);
    	break;
    	case SLOW_FLASH:
    		if(timer_cnt%5000==0)
    			GPIO_OUTPUT_SET(4,~GPIO_INPUT_GET(4));
    	break;
    	case QUICK_FLASH:
    		if(timer_cnt%2000==0)
    			GPIO_OUTPUT_SET(4,~GPIO_INPUT_GET(4));
    	break;
    }
}
os_timer_t click_status_timer;
void click_status_timer_callback(){
    if(GPIO_INPUT_GET(5)==1){
        if(LIGHT_STATUS == on) light_off();
        else light_up();
    }else{
        if(LIGHT_STATUS == on)
            FADE = true;
        else{
            light_up();
            FADE = true;
        }
    }
    clicked = false;
}
void gpio_intr_callback()
{
	uint32 gpio_status = GPIO_REG_READ(GPIO_STATUS_ADDRESS);
	GPIO_REG_WRITE(GPIO_STATUS_W1TC_ADDRESS,gpio_status);
	if(gpio_status&(BIT(5))){
	   if(GPIO_INPUT_GET(5)==0){
            if(!clicked){
                clicked = true;
                os_timer_disarm(&click_status_timer);
                os_timer_setfn(&click_status_timer,
                    (os_timer_func_t*)&click_status_timer_callback,NULL);
                os_timer_arm(&click_status_timer,200,false);
            }
        }else {
            FADE = false;
            FADE_IN = ~FADE_IN;
        }
    }else if(LIGHT_STATUS && gpio_status&(BIT(2)))
        ACROSS_ZERO_TURN_ON_TIME = NOW_TIME()+(100-BRIGHT_LEVEL)*100;
}
void udp_recv_callback(void *arg, char *data, unsigned short len)
{
    uint8 CRC[2];
    uint8 *response;
    bool send = false;
    if(len > 11){
        CRC[0] = data[0]&data[2]&data[4]&data[6]&data[8];
        CRC[1] = data[1]|data[3]|data[5]|data[7]|data[9];
        // os_printf("CRC[0] %x CRC[1] %x CHIPID %x\r\n",CRC[0],CRC[1],CHIPID);
        if(CRC[0] == data[10] && CRC[1] == data[11]){
            struct UDP_PROTOCOL *cmd = (struct UDP_PROTOCOL*)data;
             // os_printf("%x %x %x %x sizeof %d",
             //     cmd->header,cmd->id,cmd->status,cmd->brightness,sizeof(*cmd));
             if(cmd->id==CHIPID || cmd->id==0xffffffff){
                switch (cmd->header){
                    case ADJUST:
                        if(cmd->status==0xff){
                            BRIGHT_LEVEL = cmd->brightness;
                            if(BRIGHT_LEVEL<20){
                                BRIGHT_LEVEL = 20;
                            }else{
                                if(BRIGHT_LEVEL>90) 
                                    BRIGHT_LEVEL=90;
                                if(LIGHT_STATUS == off) light_up();
                            }
                        }else {
                            light_off();
                        }
                        cmd->header = ACKNOWLEDGE;
                        cmd->status = LIGHT_STATUS;
                        cmd->brightness = BRIGHT_LEVEL;
                        response = (uint8*)cmd;
                        response[10] = response[0]&response[2]&response[4]&response[6]&response[8];
                        response[11] = response[1]|response[3]|response[5]|response[7]|response[9];
                        send = true;
                    break;
                    case LOOKUP:
                        cmd->header = FOUND;
                        cmd->id = CHIPID;
                        cmd->status = LIGHT_STATUS;
                        cmd->brightness = BRIGHT_LEVEL;
                        response = (uint8*)cmd;
                        response[10] = response[0]&response[2]&response[4]&response[6]&response[8];
                        response[11] = response[1]|response[3]|response[5]|response[7]|response[9];      
                        send = true;
                    break;
                    default:
                    break;
                }
                if(send){
                    esp_conn.proto.udp->local_port = LOCAL_PORT;
                    esp_conn.proto.udp->remote_port = REMOTE_PORT;
                    esp_conn.proto.udp->remote_ip[0] = 255 ;
                    esp_conn.proto.udp->remote_ip[1] = 255 ;
                    esp_conn.proto.udp->remote_ip[2] = 255 ;
                    esp_conn.proto.udp->remote_ip[3] = 255 ;
                    espconn_sendto(&esp_conn,response,12);
                }
            }
        }
    }
}

void ICACHE_FLASH_ATTR udp_connecetion(){
    esp_conn.type = ESPCONN_UDP;
    esp_conn.state = ESPCONN_NONE;
    esp_conn.proto.udp = &espudp;
    esp_conn.proto.udp->local_port = LOCAL_PORT;
    esp_conn.proto.udp->remote_port = REMOTE_PORT;
    esp_conn.proto.udp->remote_ip[0] = 255 ;
    esp_conn.proto.udp->remote_ip[1] = 255 ;
    esp_conn.proto.udp->remote_ip[2] = 255 ;
    esp_conn.proto.udp->remote_ip[3] = 255 ;
    espconn_create(&esp_conn);
    espconn_regist_recvcb(&esp_conn,&udp_recv_callback);
}

void ICACHE_FLASH_ATTR smartconfig_done_callback(sc_status status,void * pdata)
{
    switch(status){
        case SC_STATUS_LINK:
            FLASH_STATUS = SLOW_FLASH;
            struct station_config *sta_conf = pdata;
            wifi_set_opmode(STATION_MODE);
            wifi_station_set_config(sta_conf);
            wifi_station_disconnect();
            wifi_station_connect();
            break;
        case SC_STATUS_LINK_OVER:
            smartconfig_stop();
            uint32 first_init = 0x00000000;
            spi_flash_erase_sector(USER_PARAMETERS_SECTOR);
            spi_flash_write(USER_PARAMETERS_SECTOR*4096,&first_init,4);
            break;


    }
}
void ICACHE_FLASH_ATTR first_init_func(void)
{
    FLASH_STATUS = QUICK_FLASH;
    smartconfig_set_type(SC_TYPE_ESPTOUCH);
    esptouch_set_timeout(60);
    smartconfig_start(smartconfig_done_callback);
}
void ICACHE_FLASH_ATTR wifi_set_event_handler_callback(System_Event_t *evt)
{
    switch(evt->event){
        case EVENT_STAMODE_CONNECTED:
            FLASH_STATUS = SLOW_FLASH;
        break;
        case EVENT_STAMODE_DISCONNECTED:
            FLASH_STATUS = QUICK_FLASH;
        break;
        case EVENT_STAMODE_GOT_IP:
            FLASH_STATUS = FLASH_ALLWAYS_ON;
        break;
    }
}

void ICACHE_FLASH_ATTR system_done()
{
    uint32 first_init;
    CHIPID = system_get_chip_id();
    wifi_softap_dhcps_stop();
    wifi_set_broadcast_if(1);
    wifi_set_opmode(STATION_MODE);
    wifi_set_event_handler_cb(wifi_set_event_handler_callback);
    spi_flash_read(USER_PARAMETERS_SECTOR*4096,(uint32*)&first_init,4);
    if(first_init==0xffffffff)
        first_init_func();
    udp_connecetion();
}

void ICACHE_FLASH_ATTR user_init(void)
{
    PIN_PULLUP_DIS(PERIPHS_IO_MUX_GPIO2_U);
    PIN_FUNC_SELECT(PERIPHS_IO_MUX_GPIO2_U,FUNC_GPIO2);
    GPIO_DIS_OUTPUT(2);

    PIN_PULLUP_EN(PERIPHS_IO_MUX_GPIO5_U);
    PIN_FUNC_SELECT(PERIPHS_IO_MUX_GPIO5_U,FUNC_GPIO5);

    ETS_GPIO_INTR_DISABLE();
    ETS_GPIO_INTR_ATTACH(gpio_intr_callback,NULL);
    gpio_pin_intr_state_set(5,GPIO_PIN_INTR_ANYEDGE);
    gpio_pin_intr_state_set(2,GPIO_PIN_INTR_ANYEDGE);
    ETS_GPIO_INTR_ENABLE();

    PIN_PULLUP_DIS(PERIPHS_IO_MUX_MTCK_U);
    PIN_FUNC_SELECT(PERIPHS_IO_MUX_MTCK_U,FUNC_GPIO13);
    GPIO_DIS_OUTPUT(13);

    hw_timer_init(NMI_SOURCE,1);
    hw_timer_set_func(&hw_timer_callback);
    hw_timer_arm(100);

    system_init_done_cb(system_done);
}

