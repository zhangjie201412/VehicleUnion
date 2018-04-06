#include "drivers.h"

void drivers_init(void)
{
    uint32_t flashId;

    M25PXX_Init();
    flashId = M25PXX_ReadID();
    logi("%s: read flash id %x", __func__, flashId);
}

void os_delay_ms(uint16_t ms)
{
    vTaskDelay(ms / portTICK_RATE_MS);
}

void os_delay(uint16_t s)
{
    vTaskDelay(s * 1000 / portTICK_RATE_MS);
}

void os_delay_min(uint16_t min)
{
    vTaskDelay(min * 60000 / portTICK_RATE_MS);
}
