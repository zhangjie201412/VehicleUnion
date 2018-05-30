#include "process.h"
#include "ThinMqtt.h"
#include "misc.h"

static void process(MqttMsg *msg)
{
    char delims[] = "/";
    char *result = NULL;
    uint8_t index = 0;
    char ctrl[10];
    char system[10];
    char type[10];

    logi("get message from %s: %s - %s", msg->id, msg->topic,
            msg->message);
    result = strtok(msg->topic, delims);
    while(result != NULL) {
        logi("index: %d, result: %s", index, result);
        if(index == 0) {
            memset(ctrl, 0x00, 10);
            snprintf(ctrl, 10, "%s", result);
        } else if(index == 1) {
            memset(system, 0x00, 10);
            snprintf(system, 10, "%s", result);
        } else if(index == 2) {
            memset(type, 0x00, 10);
            snprintf(type, 10, "%s", result);
        }
        index ++;
        result = strtok(NULL, delims);
    }
    if(index == 3) {
        if(strcmp(ctrl, MQTT_CMD_CTRL) == 0) {
            if(strcmp(system, SYSTEM_OBD) == 0) {
                if(strcmp(type, MQTT_CTRL_TYPE_RESET) == 0) {
                    logi("RESET!!");
                    software_reboot();
                }
            }
        }
    }
}

void vTaskProcessTask(void *pvParameters)
{
    MqttMsg *msg;
    BaseType_t ret;

    logi("%s: E", __func__);
    xMqttQueue = xQueueCreate(2, sizeof(MqttMsg *));
    if(xMqttQueue == 0) {
        loge("%s: failed to create mqtt queue", __func__);
    }
    while(TRUE) {
        logi("%s: wait for message", __func__);
        ret = xQueueReceive(xMqttQueue,
                (void *)&msg,
                portMAX_DELAY);
        if(ret == pdPASS) {
            process(msg);
        } else {
            loge("%s: failed to get message from queue.", __func__);
        }
    }
}

void process_init(void)
{

}
