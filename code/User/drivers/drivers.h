#ifndef __DRIVERS_H__
#define __DRIVERS_H__

#include "includes.h"
#include "stm32f10x.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#define logi(fmt, ...)                                      \
    do {                                                    \
        uint32_t ticks = xTaskGetTickCount();               \
        printf("[%05d.%03d/I]: " fmt "\r\n",                \
                ticks / 1000, ticks % 1000,                 \
        ##__VA_ARGS__);                                     \
    } while(0)

#define loge(fmt, ...)                                      \
    do {                                                    \
        uint32_t ticks = xTaskGetTickCount();               \
        printf("[%05d.%03d/E]: " fmt "\r\n",                \
                ticks / 1000, ticks % 1000,                 \
        ##__VA_ARGS__);                                     \
    } while(0)

void drivers_init(void);
void os_delay_ms(uint16_t ms);
void os_delay(uint16_t s);
void os_delay_min(uint16_t min);

#endif
