#ifndef __THIN_MQTT_H__
#define __THIN_MQTT_H__

#include "stm32f10x.h"
#include "drivers.h"

#define bool uint8_t

extern QueueHandle_t xMqttQueue;

typedef struct __MqttMsg {
    char id[16];
    char topic[128];
    char message[128];
} MqttMsg;

void vTaskMqttProcess(void *pvParameters);

void thin_mqtt_init(void);
void thin_mqtt_release(void);
bool thin_mqtt_publish(const char *id, const char *topic, const char *message);


#endif
