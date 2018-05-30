/*
*/
#include "includes.h"
#include "drivers.h"
#include "m25pxx.h"
#include "flash_table.h"
#include "l206.h"
#include "ThinMqtt.h"
#include "process.h"
#include "gps.h"

#include "kwp2000.h"
#include "can.h"

char *logo_text =
"+++++++++++++++++++++++++++++++++++++++++++\r\n"
"+                                         +\r\n"
"+            VehicleUnion.org             +\r\n"
"+             V2018.0226.01               +\r\n"
"+                                         +\r\n"
"+++++++++++++++++++++++++++++++++++++++++++\r\n"
;
/*
 **********************************************************************************************************
 ��������
 **********************************************************************************************************
 */
static void vTaskStart(void *pvParameters);
static void vTaskFlashTest(void *pvParameters);

static void AppTaskCreate (void);

/*
 **********************************************************************************************************
 ��������
 **********************************************************************************************************
 */
static TaskHandle_t xHandleTaskStart = NULL;
static TaskHandle_t xHandleTaskL206Process = NULL;
static TaskHandle_t xHandleTaskMqttProcess = NULL;
static TaskHandle_t xHandleTaskProcess = NULL;
static TaskHandle_t xHandleTaskGpsProcess = NULL;

#define TASK_L206_PRIO      1
#define TASK_MQTT_PRIO      1
#define TASK_PROCESS_PRIO   4
#define TASK_GPS_PRIO       1

#define TASK_L206_STACK         256
#define TASK_MQTT_STACK         1024
#define TASK_PROCESS_STACK      1024
#define TASK_GPS_STACK          512

static TaskHandle_t xHandleTaskFlashTest = NULL;

/*
 *********************************************************************************************************
 *	�� �� ��: main
 *	����˵��: ��׼c������ڡ�
 *	��    �Σ���
 *	�� �� ֵ: ��
 *********************************************************************************************************
 */
int main(void)
{
    //SCB->VTOR = FLASH_BASE | 0x20000;
    /* 
       ����������ǰ��Ϊ�˷�ֹ��ʼ��STM32����ʱ���жϷ������ִ�У������ֹȫ���ж�(����NMI��HardFault)��
       �������ĺô��ǣ�
       1. ��ִֹ�е��жϷ����������FreeRTOS��API������
       2. ��֤ϵͳ�������������ܱ���ж�Ӱ�졣
       3. �����Ƿ�ر�ȫ���жϣ���Ҹ����Լ���ʵ��������ü��ɡ�
       ����ֲ�ļ�port.c�еĺ���prvStartFirstTask�л����¿���ȫ���жϡ�ͨ��ָ��cpsie i������__set_PRIMASK(1)
       ��cpsie i�ǵ�Ч�ġ�
       */
    __set_PRIMASK(1);  

    /* Ӳ����ʼ�� */
    bsp_Init(); 
    printf("%s", logo_text);
    /* �������� */
    AppTaskCreate();
    /* �������ȣ���ʼִ������ */
    vTaskStartScheduler();

    /* 
       ���ϵͳ���������ǲ������е�����ģ����е����Ｋ�п��������ڶ�ʱ��������߿��������
       heap�ռ䲻����ɴ���ʧ�ܣ���Ҫ�Ӵ�FreeRTOSConfig.h�ļ��ж����heap��С��
#define configTOTAL_HEAP_SIZE	      ( ( size_t ) ( 17 * 1024 ) )
*/
    while(1);
}

/*
 *********************************************************************************************************
 *	�� �� ��: vTaskStart
 *	����˵��: ��������Ҳ����������ȼ�������������LED��˸
 *	��    ��: pvParameters ���ڴ���������ʱ���ݵ��β�
 *	�� �� ֵ: ��
 *   �� �� ��: 4  
 *********************************************************************************************************
 */
static void vTaskStart(void *pvParameters)
{
    drivers_init();
    l206_init();

    xTaskCreate(vTaskL206Process,
            "L206Process",
            2048,
            NULL,
            8,
            &xHandleTaskL206Process);
    xTaskCreate(vTaskMqttProcess,
            "MqttProcess",
            2048,
            NULL,
            6,
            &xHandleTaskMqttProcess);
    while(1)
    {
        os_delay(10);
        logi("...");
        //can_sample();
        //kwp2000_sample();
        //comClearRxFifo(COM1);
        //comSendBuf(COM1, "hello", 5);
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
 *	�� �� ��: AppTaskCreate
 *	����˵��: ����Ӧ������
 *	��    �Σ���
 *	�� �� ֵ: ��
 *********************************************************************************************************
 */
static void AppTaskCreate (void)
{
    drivers_init();
#if 1
    xTaskCreate(vTaskProcessTask,
            "ProcessTask",
            TASK_PROCESS_STACK,
            NULL,
            TASK_PROCESS_PRIO,
            &xHandleTaskProcess);
#endif
#if 1
    xTaskCreate(vTaskGpsProcess,
            "GpsProcess",
            TASK_GPS_STACK,
            NULL,
            TASK_GPS_PRIO,
            &xHandleTaskGpsProcess);
    xTaskCreate(vTaskL206Process,
            "L206Process",
            TASK_L206_STACK,
            NULL,
            TASK_L206_PRIO,
            &xHandleTaskL206Process);
    xTaskCreate(vTaskMqttProcess,
            "MqttProcess",
            TASK_MQTT_STACK,
            NULL,
            TASK_MQTT_PRIO,
            &xHandleTaskMqttProcess);
#endif

#if 0
    xTaskCreate(vTaskStart,     		/* ������  */
            "vTaskStart",   		/* ������    */
            512,            		/* ����ջ��С����λword��Ҳ����4�ֽ� */
            NULL,           		/* �������  */
            9,              		/* �������ȼ�*/
            &xHandleTaskStart );   /* ������  */
    xTaskCreate( vTaskFlashTest,     		/* ������  */
            "vTaskFlashTest",   		/* ������    */
            512,            		/* ����ջ��С����λword��Ҳ����4�ֽ� */
            NULL,           		/* �������  */
            3,              		/* �������ȼ�*/
            &xHandleTaskFlashTest );   /* ������  */
#endif
}

/***************************** ���������� www.armfly.com (END OF FILE) *********************************/
