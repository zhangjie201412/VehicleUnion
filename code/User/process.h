#ifndef __PROCESS_H__
#define __PROCESS_H__

#include "stm32f10x.h"
#include "drivers.h"

/* Command reply from server */
#define MQTT_RPL_PING           "PING"
#define MQTT_RPL_LOGIN          "LOGIN"

/* Command sent from server */
#define MQTT_CMD_CTRL           "CTRL"

// /SN123456/CTRL/OBD/RESET
#define MQTT_CTRL_TYPE_RESET    "RESET"
#define MQTT_CTRL_TYPE_CLR_CODE "CLR_CODE"
#define MQTT_CTRL_TYPE_CHK_CODE "CHK_CODE"
#define MQTT_CTRL_TYPE_INTERVAL "INTERVAL"

/* Command sent to server */
#define MQTT_CMD_UPLOAD         "UPLOAD"

#define SYSTEM_OBD              "OBD"
// /SN123456/UPLOAD/ENG/RPM
#define SYSTEM_ENG              "ENG"
#define SYSTEM_BCM              "BCM"
#define SYSTEM_GPS              "GPS"
#define SYSTEM_ALARM            "ALARM"

void vTaskProcessTask(void *pvParameters);

void process_init(void);

#endif
