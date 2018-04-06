#include "ThinMqtt.h"
#include "l206.h"

static TaskHandle_t xHandleTaskMqttProcess = NULL;

static void vTaskMqttProcess(void *pvParameters);
static ThinMqttClient mMqttClient;

static void vTaskMqttProcess(void *pvParameters)
{
    while(1) {
        logi("%s: E", __func__);
        os_delay(10);
        if(l206_is_connected() == TRUE) {
            mMqttClient.mqtt_write("Hello\n", 6);
            logi("send hello!!");
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

    mMqttClient.mqtt_read = mqtt_client_read;
    mMqttClient.mqtt_write = mqtt_client_write;

    xTaskCreate(vTaskMqttProcess,
            "MqttProcess",
            2048,
            NULL,
            4,
            &xHandleTaskMqttProcess);
    logi("%s: X", __func__);
}

void thin_mqtt_release()
{

}

bool thin_mqtt_connect(ThinMqttClient *client)
{
    bool ret = TRUE;
    char buf[128] = {0};
    int bytes = 0;
    int wbytes = 0;
    int i;
    uint8_t cs = 0x00;

    bytes = snprintf(buf, 128, ">>%s,connect,", client->id);
    for(i = 0; i< bytes; i++) {
        cs ^= buf[i];
    }
    bytes += snprintf(buf + bytes, 128, "%02x\n", cs);
    wbytes = client->mqtt_write(buf, bytes);
    if(bytes != wbytes) {
        loge("%s: write %d bytes, actual write %d", __func__, bytes, wbytes);
        return FALSE;
    }
    //wait connack
    
    return ret;
}

bool thin_mqtt_disconnect(ThinMqttClient *client)
{
    bool ret = TRUE;
    char buf[128] = {0};
    int bytes = 0;
    int wbytes = 0;
    int i;
    uint8_t cs = 0x00;

    bytes = snprintf(buf, 128, ">>%s,disconnect,", client->id);
    for(i = 0; i< bytes; i++) {
        cs ^= buf[i];
    }
    bytes += snprintf(buf + bytes, 128, "%02x\n", cs);
    wbytes = client->mqtt_write(buf, bytes);
    if(bytes != wbytes) {
        loge("%s: write %d bytes, actual write %d", __func__, bytes, wbytes);
        return FALSE;
    }
    //wait connack

    return ret;
}

bool thin_mqtt_ping(ThinMqttClient *client)
{
    bool ret = TRUE;
    char buf[128] = {0};
    int bytes = 0;
    int wbytes = 0;
    int i;
    uint8_t cs = 0x00;

    bytes = snprintf(buf, 128, ">>%s,ping,", client->id);
    for(i = 0; i< bytes; i++) {
        cs ^= buf[i];
    }
    bytes += snprintf(buf + bytes, 128, "%02x\n", cs);
    wbytes = client->mqtt_write(buf, bytes);
    if(bytes != wbytes) {
        loge("%s: write %d bytes, actual write %d", __func__, bytes, wbytes);
        return FALSE;
    }
    //wait connack

    return ret;
}

bool thin_mqtt_publish(ThinMqttClient *client, const char *topic,
        const char *message)
{
    bool ret = TRUE;
    char buf[128] = {0};
    int bytes = 0;
    int wbytes = 0;
    int i;
    uint8_t cs = 0x00;

    bytes = snprintf(buf, 128, ">>%s,publish,%s,%s,",
            client->id, topic, message);
    for(i = 0; i< bytes; i++) {
        cs ^= buf[i];
    }
    bytes += snprintf(buf + bytes, 128, "%02x\n", cs);
    wbytes = client->mqtt_write(buf, bytes);
    if(bytes != wbytes) {
        loge("%s: write %d bytes, actual write %d", __func__, bytes, wbytes);
        return FALSE;
    }
    //wait connack

    return ret;
}

bool thin_mqtt_subscribe(ThinMqttClient *client, const char *topic)
{
    bool ret = TRUE;
    char buf[128] = {0};
    int bytes = 0;
    int wbytes = 0;
    int i;
    uint8_t cs = 0x00;

    bytes = snprintf(buf, 128, ">>%s,subscribe,%s",
            client->id, topic);
    for(i = 0; i< bytes; i++) {
        cs ^= buf[i];
    }
    bytes += snprintf(buf + bytes, 128, "%02x\n", cs);
    wbytes = client->mqtt_write(buf, bytes);
    if(bytes != wbytes) {
        loge("%s: write %d bytes, actual write %d", __func__, bytes, wbytes);
        return FALSE;
    }
    //wait connack

    return ret;
}
