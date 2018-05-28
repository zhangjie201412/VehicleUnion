#include "led.h"
#include "bsp.h"
#include "drivers.h"

#define RCC_ALL_LED 	(RCC_APB2Periph_GPIOC)

#define GPIO_PORT_LED   GPIOC
#define GPIO_PIN_LED	GPIO_Pin_6

static int led_mode;
static int led_interval;

void led_SetMode(int mode)
{
    led_mode = mode;
}

void led_SetInterval(int ms)
{
    led_interval = ms;
}

void vTaskLedProcess(void *pvParameters)
{
    logi("%s: E", __func__);
    led_Init();
    while(1) {
        switch(led_mode) {
            case LED_MODE_ON:
                led_On();
                os_delay_ms(100);
                break;
            case LED_MODE_OFF:
                led_Off();
                os_delay_ms(100);
                break;
            case LED_MODE_BLINK:
                led_Toggle();
                os_delay_ms(led_interval);
                break;
            default:
                break;
        }
    }
}

void led_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;

    /* 打开GPIO时钟 */
    RCC_APB2PeriphClockCmd(RCC_ALL_LED, ENABLE);

    /*
       配置所有的LED指示灯GPIO为推挽输出模式
       由于将GPIO设置为输出时，GPIO输出寄存器的值缺省是0，因此会驱动LED点亮.
       这是我不希望的，因此在改变GPIO为输出前，先关闭LED指示灯
       */
    led_Off();

    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;	/* 推挽输出模式 */

    GPIO_InitStructure.GPIO_Pin = GPIO_PIN_LED;
    GPIO_Init(GPIO_PORT_LED, &GPIO_InitStructure);
    led_mode = LED_MODE_OFF;
    led_interval = 100;
}

void led_On(void)
{
    GPIO_PORT_LED->BRR = GPIO_PIN_LED;
}

void led_Off(void)
{
    GPIO_PORT_LED->BSRR = GPIO_PIN_LED;
}

void led_Toggle(void)
{
    GPIO_PORT_LED->ODR ^= GPIO_PIN_LED;
}

void led_Blink(void)
{
    led_On();
    os_delay_ms(100);
    led_Off();
}
