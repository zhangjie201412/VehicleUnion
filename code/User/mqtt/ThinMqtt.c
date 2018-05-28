#include "ThinMqtt.h"
#include "l206.h"


void vTaskMqttProcess(void *pvParameters)
{
    static char id[16];
    memset(id, 0x00, 16);
    while(1) {
        logi("%s: E", __func__);
        if(l206_is_connected()) {
            get_deviceid(id);
            logi("device id: %s", id);
            thin_mqtt_publish(id, "ENG/DATA/RPM", "1200");
        } else {
            logi("connecting...");
        }
        os_delay(10);
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

    bytes = snprintf(buf, 128, ">>%s,IOT/%s,%s,",
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

