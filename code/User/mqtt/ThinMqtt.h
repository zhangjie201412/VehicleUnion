#ifndef __THIN_MQTT_H__
#define __THIN_MQTT_H__

#include "stm32f10x.h"
#include "drivers.h"

#define bool uint8_t

#define BUFFER_MAX      1024
#define ID_MAX          32

typedef struct {
    char id[ID_MAX];
    int32_t  status;
    char buffer[BUFFER_MAX];
    //register callback function
    int (*mqtt_read)(char *, int);
    int (*mqtt_write)(char *, int);
} ThinMqttClient;

void thin_mqtt_init();
void thin_mqtt_release();
bool thin_mqtt_connect(ThinMqttClient *client);
bool thin_mqtt_disconnect(ThinMqttClient *client);
bool thin_mqtt_ping(ThinMqttClient *client);
bool thin_mqtt_publish(ThinMqttClient *client, const char *topic,
        const char *message);
bool thin_mqtt_subscribe(ThinMqttClient *client, const char *topic);


#endif
