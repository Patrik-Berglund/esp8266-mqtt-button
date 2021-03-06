#ifndef _USER_CONFIG_H_
#define _USER_CONFIG_H_

#define CFG_HOLDER	0x00FF55AA	/* Change this value to load default configurations */
#define CFG_LOCATION	0x3C	/* Please don't change or if you know what you doing */
#define CLIENT_SSL_ENABLE

/*DEFAULT CONFIGURATIONS*/
#define MQTT_BUTTON_TOPIC "/mqtt/button"

#define MQTT_HOST			"test.mosquitto.org" //or "mqtt.yourdomain.com"
#define MQTT_PORT			1883
#define MQTT_BUF_SIZE		1024
#define MQTT_KEEPALIVE		120	 /*second*/

#define MQTT_CLIENT_ID		"DEVS_%08X"
#define MQTT_USER			""
#define MQTT_PASS			""

#define STA_SSID "SSID"
#define STA_PASS "Password"
#define STA_TYPE AUTH_WPA2_PSK

#define MQTT_RECONNECT_TIMEOUT 	5	/*second*/

#define DEFAULT_SECURITY	0
#define QUEUE_BUFFER_SIZE	2048

#define PROTOCOL_NAMEv311	
/*MQTT version 3.1 compatible with Mosquitto v0.15*/
/*MQTT version 3.11 compatible with https://eclipse.org/paho/clients/testing/*/


void ICACHE_FLASH_ATTR wifiConnectCb(uint8_t status);

void ICACHE_FLASH_ATTR mqttConnectedCb(uint32_t *args);
void ICACHE_FLASH_ATTR mqttDisconnectedCb(uint32_t *args);
void ICACHE_FLASH_ATTR mqttPublishedCb(uint32_t *args);

bool ICACHE_FLASH_ATTR testBit32( uint32_t A[],  int32_t k );
bool ICACHE_FLASH_ATTR testBit64( uint64_t A[],  int32_t k );
bool testBit8(uint8_t A[],  int32_t k );

void ICACHE_FLASH_ATTR mqttParseMessage(const char* topic, uint32_t topic_len, const char *data, uint32_t data_len);
void ICACHE_FLASH_ATTR mqttDataCb(uint32_t *args, const char* topic, uint32_t topic_len, const char *data, uint32_t data_len);
void ICACHE_FLASH_ATTR gpioSetup(void);
void ICACHE_FLASH_ATTR user_init(void);

#endif
