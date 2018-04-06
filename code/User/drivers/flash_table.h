#ifndef __FLASH_TABLE_H__
#define __FLASH_TABLE_H__

typedef struct __ConfigInfo {
    uint8_t head[2];
    uint8_t device_id[16];
    uint32_t boot_times;
} ConfigInfo;

void flash_table_init(void);

#endif
