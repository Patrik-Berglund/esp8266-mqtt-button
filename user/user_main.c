/*
* Copyright (c) 2014-2015, Tuan PM <tuanpm at live dot com>
* All rights reserved.
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions are met:
*
* * Redistributions of source code must retain the above copyright notice,
* this list of conditions and the following disclaimer.
* * Redistributions in binary form must reproduce the above copyright
* notice, this list of conditions and the following disclaimer in the
* documentation and/or other materials provided with the distribution.
* * Neither the name of Redis nor the names of its contributors may be used
* to endorse or promote products derived from this software without
* specific prior written permission.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
* AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
* IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
* ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
* LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
* CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
* SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
* INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
* CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
* ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
* POSSIBILITY OF SUCH DAMAGE.
*/

#include "ets_sys.h"
#include "driver/uart.h"
#include "osapi.h"
#include "mqtt.h"
#include "wifi.h"
#include "config.h"
#include "debug.h"
#include "gpio.h"
#include "user_interface.h"
#include "mem.h"

MQTT_Client mqttClient;

void ICACHE_FLASH_ATTR wifiConnectCb(uint8_t status){
	if(status == STATION_GOT_IP){
		MQTT_Connect(&mqttClient);
	} else {
		MQTT_Disconnect(&mqttClient);
	}
}

void ICACHE_FLASH_ATTR mqttConnectedCb(uint32_t *args){
	MQTT_Client* client = (MQTT_Client*)args;
	INFO("MQTT: Connected\n");
}

void ICACHE_FLASH_ATTR mqttDisconnectedCb(uint32_t *args){
	MQTT_Client* client = (MQTT_Client*)args;
	INFO("MQTT: Disconnected\n");
}

void ICACHE_FLASH_ATTR mqttPublishedCb(uint32_t *args){
	MQTT_Client* client = (MQTT_Client*)args;
	INFO("MQTT: Published\n");
}

void ICACHE_FLASH_ATTR mqttParseMessage(const char* topic, uint32_t topic_len, const char *data, uint32_t data_len){
	// if(strcmpi(topic, MQTT_TOPIC_NEXA_BRIDGE_SEND) == 0){
    //     parseNexaSendMessage(data, data_len);
	// }
	// else{
	// 	INFO("MQTT: Unknown topic\n");
	// }
}

void ICACHE_FLASH_ATTR mqttDataCb(uint32_t *args, const char* topic, uint32_t topic_len, const char *data, uint32_t data_len){
	char *topicBuf = (char*)os_zalloc(topic_len+1);
	char *dataBuf = (char*)os_zalloc(data_len+1);

	MQTT_Client* client = (MQTT_Client*)args;

	os_memcpy(topicBuf, topic, topic_len);
	topicBuf[topic_len] = 0;

	os_memcpy(dataBuf, data, data_len);
	dataBuf[data_len] = 0;

	INFO("MQTT: Receive topic: %s, data: %s\n", topicBuf, dataBuf);

	mqttParseMessage(topicBuf, topic_len, dataBuf, data_len);

	os_free(topicBuf);
	os_free(dataBuf);
}

void ICACHE_FLASH_ATTR gpioSetup(void){
    gpio_init();
    //PIN_FUNC_SELECT(PERIPHS_IO_MUX_GPIO2_U, FUNC_GPIO2);
	//gpio_output_set(0, BIT2, BIT2, 0);
}

void ICACHE_FLASH_ATTR user_init(void){
	system_timer_reinit();

	uart_init(BIT_RATE_115200, BIT_RATE_115200);
	os_delay_us(1000000);

	INFO("SDK version: %s\n", system_get_sdk_version());
	INFO("System init ...\n");

	CFG_Load();

	gpioSetup();

	MQTT_InitConnection(&mqttClient, sysCfg.mqtt_host, sysCfg.mqtt_port, sysCfg.security);
	MQTT_InitClient(&mqttClient, sysCfg.device_id, sysCfg.mqtt_user, sysCfg.mqtt_pass, sysCfg.mqtt_keepalive, 1);
	MQTT_InitLWT(&mqttClient, "/lwt", "offline", 0, 0);
	MQTT_OnConnected(&mqttClient, mqttConnectedCb);
	MQTT_OnDisconnected(&mqttClient, mqttDisconnectedCb);
	MQTT_OnPublished(&mqttClient, mqttPublishedCb);
	MQTT_OnData(&mqttClient, mqttDataCb);

	WIFI_Connect(sysCfg.sta_ssid, sysCfg.sta_pwd, wifiConnectCb);

	INFO("\nSystem started ...\n");
}