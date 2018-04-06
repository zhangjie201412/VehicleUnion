/*
*/
#include "includes.h"
#include "drivers.h"
#include "m25pxx.h"
#include "flash_table.h"
#include "ThinMqtt.h"

char *logo_text =
"+++++++++++++++++++++++++++++++++++++++++++\r\n"
"+                                         +\r\n"
"+            VehicleUnion.org             +\r\n"
"+             V2018.0226.01               +\r\n"
"+                                         +\r\n"
"+++++++++++++++++++++++++++++++++++++++++++\r\n"
"This is app!!\r\n";

/*
 **********************************************************************************************************
 函数声明
 **********************************************************************************************************
 */
static void vTaskStart(void *pvParameters);
static void vTaskFlashTest(void *pvParameters);

static void AppTaskCreate (void);

/*
 **********************************************************************************************************
 变量声明
 **********************************************************************************************************
 */
static TaskHandle_t xHandleTaskStart = NULL;
static TaskHandle_t xHandleTaskFlashTest = NULL;

/*
 *********************************************************************************************************
 *	函 数 名: main
 *	功能说明: 标准c程序入口。
 *	形    参：无
 *	返 回 值: 无
 *********************************************************************************************************
 */
int main(void)
{
    //SCB->VTOR = FLASH_BASE | 0x20000;
    /* 
       在启动调度前，为了防止初始化STM32外设时有中断服务程序执行，这里禁止全局中断(除了NMI和HardFault)。
       这样做的好处是：
       1. 防止执行的中断服务程序中有FreeRTOS的API函数。
       2. 保证系统正常启动，不受别的中断影响。
       3. 关于是否关闭全局中断，大家根据自己的实际情况设置即可。
       在移植文件port.c中的函数prvStartFirstTask中会重新开启全局中断。通过指令cpsie i开启，__set_PRIMASK(1)
       和cpsie i是等效的。
       */
    __set_PRIMASK(1);  

    /* 硬件初始化 */
    bsp_Init(); 
    printf("%s", logo_text);
    /* 创建任务 */
    AppTaskCreate();
    /* 启动调度，开始执行任务 */
    vTaskStartScheduler();

    /* 
       如果系统正常启动是不会运行到这里的，运行到这里极有可能是用于定时器任务或者空闲任务的
       heap空间不足造成创建失败，此要加大FreeRTOSConfig.h文件中定义的heap大小：
#define configTOTAL_HEAP_SIZE	      ( ( size_t ) ( 17 * 1024 ) )
*/
    while(1);
}

/*
 *********************************************************************************************************
 *	函 数 名: vTaskStart
 *	功能说明: 启动任务，也就是最高优先级任务，这里用作LED闪烁
 *	形    参: pvParameters 是在创建该任务时传递的形参
 *	返 回 值: 无
 *   优 先 级: 4  
 *********************************************************************************************************
 */
static void vTaskStart(void *pvParameters)
{
    char buf2[4];
    int total = 1024;

    drivers_init();
    l206_init();
    thin_mqtt_init();
    while(1)
    {
        os_delay_ms(2000);
    }
}

uint8_t Tx_Buffer[] = "HELLO!!";
uint8_t Rx_Buffer[7];
#define FLASH_SECTOR_TO_ERASE       0x100
static void vTaskFlashTest(void *pvParameters)
{
    uint32_t flashId;

    logi("%s: E", __func__);
    M25PXX_Init();

    flashId = M25PXX_ReadID();
    logi("%s: read flash id %x", __func__, flashId);
    M25PXX_EraseSector(FLASH_SECTOR_TO_ERASE);
    M25PXX_WriteBuffer(Tx_Buffer, 0x100, 7);
    M25PXX_ReadBuffer(Rx_Buffer, 0x100, 7);
    logi("read buffer: %s", Rx_Buffer);

    while(1)
    {
        os_delay_ms(2000);
    }
}

/*
 *********************************************************************************************************
 *	函 数 名: AppTaskCreate
 *	功能说明: 创建应用任务
 *	形    参：无
 *	返 回 值: 无
 *********************************************************************************************************
 */
static void AppTaskCreate (void)
{
    xTaskCreate( vTaskStart,     		/* 任务函数  */
            "vTaskStart",   		/* 任务名    */
            512,            		/* 任务栈大小，单位word，也就是4字节 */
            NULL,           		/* 任务参数  */
            4,              		/* 任务优先级*/
            &xHandleTaskStart );   /* 任务句柄  */
#if 0
    xTaskCreate( vTaskFlashTest,     		/* 任务函数  */
            "vTaskFlashTest",   		/* 任务名    */
            512,            		/* 任务栈大小，单位word，也就是4字节 */
            NULL,           		/* 任务参数  */
            3,              		/* 任务优先级*/
            &xHandleTaskFlashTest );   /* 任务句柄  */
#endif
}

/***************************** 安富莱电子 www.armfly.com (END OF FILE) *********************************/
