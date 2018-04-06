#include "drivers.h"
#include "m25pxx.h"
#include "l206.h"

#include "flash_table.h"

static ConfigInfo s_config_info;

void flash_table_init(void)
{
    int size = sizeof(s_config_info);
    uint8_t *buf;
    int i;
    char deviceid[15];

    logi("%s: E", __func__);
    buf = malloc(size);
    memset(buf, 0x00, size);
    memset(&s_config_info, 0x00, size);
    M25PXX_ReadBuffer(buf, 0x00, size);
    for(i = 0; i < size; i++) {
        printf("%02x(%c) ", buf[i], buf[i]);
        if((i + 1) % 10 == 0) {
            printf("\r\n");
        }
    }
    printf("\r\n");
    memcpy(&s_config_info, buf, size);
    if(s_config_info.head[0] == '$' &&
            s_config_info.head[1] == '$') {
        logi("%s: config already done", __func__);
        logi("%s: deviceid - %s, boot times - %d", __func__,
                s_config_info.device_id, s_config_info.boot_times);
        s_config_info.boot_times ++;
        M25PXX_EraseSector(0x00);
        memcpy(buf, &s_config_info, size);
        M25PXX_WriteBuffer(buf, 0x00, size);
    } else {
        get_deviceid(deviceid);
        logi("%s: get device id %s", __func__, deviceid);
        s_config_info.head[0] = '$';
        s_config_info.head[1] = '$';
        memcpy(s_config_info.device_id, deviceid, 16);
        s_config_info.boot_times = 1;
        M25PXX_EraseSector(0x00);
        memcpy(buf, &s_config_info, size);
        M25PXX_WriteBuffer(buf, 0x00, size);
    }
    free(buf);
    logi("%s: X", __func__);
}
