#ifndef __LED_H__
#define __LED_H__

typedef enum {
    LED_MODE_ON,
    LED_MODE_OFF,
    LED_MODE_BLINK,
} LedMode;

/* 供外部调用的函数声明 */
void vTaskLedProcess(void *pvParameters);
void led_Init(void);
void led_On(void);
void led_Off(void);
void led_Toggle(void);
void led_Blink(void);

void led_SetMode(int mode);
void led_SetInterval(int ms);

#endif
