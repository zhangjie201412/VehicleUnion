#include "ThinMqtt.h"
#include <string.h>

#include "l206.h"
#include "misc.h"

QueueHandle_t xMqttQueue = NULL;
static MqttMsg g_msg;

static uint8_t thin_mqtt_parse(char *buf, MqttMsg *msg)
{
    char delims[] = ",";
    char *result = NULL;
    uint8_t index = 0;
    uint8_t ret = FALSE;

    result = strtok(buf, delims);
    while(result != NULL) {
        logi("index: %d, result: %s", index, result);
        if(index == 0) {
            memset(msg->id, 0x00, 16);
            snprintf(msg->id, 16, "%s", result + 2);
        } else if(index == 1) {
            memset(msg->topic, 0x00, 128);
            snprintf(msg->topic, 128, "%s", result);
        } else if(index == 2) {
            memset(msg->message, 0x00, 128);
            snprintf(msg->message, 128, "%s", result);
        }
        index ++;
        result = strtok(NULL, delims);
    }
    if(index == 4 && (buf[0] == '<') && (buf[1] == '<')) {
        ret = TRUE;
    }
    return ret;
}

void vTaskMqttProcess(void *pvParameters)
{
    static char id[16];
    static char buf[256];
    static char *buffer;
    int offset = 0;
    uint8_t recv;
    uint16_t index;
    uint8_t login_done = 0;
    uint32_t loop = 0;
    char message[10];
    uint16_t bytes;
    MqttMsg *msg;

    msg = &g_msg;
    memset(id, 0x00, 16);
    memset(buf, 0x00, 256);

    while(1) {
        //logi("%s: E", __func__);
        if(l206_is_connected()) {
            get_deviceid(id);
            if(!login_done) {
                thin_mqtt_publish(id, "LOGIN", "");
                login_done = 1;
            }
            recv = l206_recv(buf + offset, 256 - offset, 2000);
            offset += recv;
            if(buf[offset - 1] == '\n') {
                index = 0;
                while(buf[index] != '<') {
                    index ++;
                }
                buffer = buf + index;
                logi("recv: %s", buffer);
                //TODO: process the buffer
                if(thin_mqtt_parse(buffer, msg) == TRUE) {
                    if(xQueueSend(xMqttQueue,
                                (void *)&msg,
                                (TickType_t)10) != pdPASS) {
                        loge("%s: send message queue failed", __func__);
                    }
                }
                offset = 0;
                memset(buf, 0x00, 256);
            }
            os_delay_ms(50);
            if(loop % 10 == 0) {
                bytes = snprintf(message, 10, "%d", loop / 10);
                message[bytes] = '\0';
                if(thin_mqtt_publish(id, "PING", message) == FALSE) {
                    //reboot the device
                    software_reboot();
                }
            }
            if(loop ++ > 999) {
                loop = 0;
            }
        } else {
            logi("connecting...");
            os_delay(4);
        }
    }
}

int mqtt_client_read(char *buf, int bytes)
{
    return 0;
}

int mqtt_client_write(char *buf, int bytes)
{
    if(l206_send(buf, bytes, 5000) == TRUE) {
        return 0;
    } else {
        return -1;
    }
}

void thin_mqtt_init()
{
    logi("%s: E", __func__);

    logi("%s: X", __func__);
}

void thin_mqtt_release()
{

}

bool thin_mqtt_publish(const char *id, const char *topic, const char *message)
{
    int ret;
    char buf[128] = {0};
    int bytes = 0;
    int wbytes = 0;
    int i;
    uint8_t cs = 0x00;

    bytes = snprintf(buf, 128, ">>%s,%s,%s,",
            id, topic, message);
    for(i = 0; i< bytes; i++) {
        cs ^= buf[i];
    }
    bytes += snprintf(buf + bytes, 128, "%02x\n", cs);
    ret = mqtt_client_write(buf, bytes);
    if(ret < 0) {
        loge("%s: write %d bytes, actual write %d", __func__, bytes, wbytes);
        return FALSE;
    }

    return TRUE;
}

