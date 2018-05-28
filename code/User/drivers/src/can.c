#include "can.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_can.h"
#include "drivers.h"

#define RCC_ALL_CAN_SWITCH      RCC_APB2Periph_GPIOC

#define CAN_SWITCH_A_IO         GPIOC
#define CAN_SWITCH_A_PIN        GPIO_Pin_9
#define CAN_SWITCH_B_IO         GPIOC
#define CAN_SWITCH_B_PIN        GPIO_Pin_8

uint8_t canDebug = 0;
uint8_t extId = 0;

static void CAN_SetSwitch(uint8_t a, uint8_t b);

void can_sample(void)
{
    CanMessage_t test_msg = 
    {0x730, 8, 
        {0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07}
    };
    CAN_InitSwitch();
    CAN_SetSwitch(1, 0);

    CAN_UseStdId();
    CAN_Open();

    if(!CAN_SendMsg(&test_msg)) {
        loge("%s: failed to send test msg", __func__);
    }
}

void CAN_InitSwitch(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;

	/* 打开GPIO时钟 */
	RCC_APB2PeriphClockCmd(RCC_ALL_CAN_SWITCH, ENABLE);

	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;	/* 推挽输出模式 */
	
	GPIO_InitStructure.GPIO_Pin = CAN_SWITCH_A_PIN;
	GPIO_Init(CAN_SWITCH_A_IO, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = CAN_SWITCH_B_PIN;
	GPIO_Init(CAN_SWITCH_B_IO, &GPIO_InitStructure);
}

void CAN_SwitchMode(uint8_t mode)
{
    switch(mode) {
        case CAN_PIN_1_9:
            break;
            CAN_SetSwitch(1, 0);
        case CAN_PIN_3_11:
            CAN_SetSwitch(0, 0);
            break;
        case CAN_PIN_6_14:
        default:
            CAN_SetSwitch(1, 1);
            break;
    }
}

void CAN_SetSwitch(uint8_t a, uint8_t b)
{
    //set switch A to high
    //set switch B to low

    if(a) {
        GPIO_SetBits(CAN_SWITCH_A_IO, CAN_SWITCH_A_PIN);
    } else {
        GPIO_ResetBits(CAN_SWITCH_A_IO, CAN_SWITCH_A_PIN);
    }

    if(b) {
        GPIO_SetBits(CAN_SWITCH_B_IO, CAN_SWITCH_B_PIN);
    } else {
        GPIO_ResetBits(CAN_SWITCH_B_IO, CAN_SWITCH_B_PIN);
    }
}

void CAN_UseStdId(void)
{
    extId = 0;
}

void CAN_UseExtId(void)
{
    extId = 1;
}

#ifdef DEBUG_CAN_FRAME
void DebugCanFrame(uint8_t tx, uint8_t extFrame, CanMessage_t *msg)
{

}
#endif

void CAN_Open(void)
{
    CAN_InitTypeDef        CAN_InitStructure;
    CAN_FilterInitTypeDef  CAN_FilterInitStructure;
    GPIO_InitTypeDef GPIO_InitStructure;

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_AFIO, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN1, ENABLE);

    // Configure CAN pin: RX
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    // Configure CAN pin: TX
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    /* CAN register init */
    CAN_DeInit(CAN1);
    CAN_StructInit(&CAN_InitStructure);

    /* CAN cell init */
    CAN_InitStructure.CAN_TTCM=DISABLE;
    CAN_InitStructure.CAN_ABOM=DISABLE;
    CAN_InitStructure.CAN_AWUM=DISABLE;
    //CAN_InitStructure.CAN_NART=DISABLE;
    CAN_InitStructure.CAN_NART=ENABLE; //no automatic retransmission
    CAN_InitStructure.CAN_RFLM=DISABLE;
    CAN_InitStructure.CAN_TXFP=DISABLE;

    CAN_InitStructure.CAN_Mode=CAN_Mode_Normal;

    //CAN block is clocked with Low Speed APB1 clock (PCLK1) = 36MHz
    CAN_InitStructure.CAN_SJW=CAN_SJW_1tq;
    CAN_InitStructure.CAN_BS1=CAN_BS1_11tq; //CAN_BS1_8tq @ 24mhz
    CAN_InitStructure.CAN_BS2=CAN_BS2_6tq; //CAN_BS2_7tq @24mhz
    CAN_InitStructure.CAN_Prescaler=4; //=3  500kbit@24mhz

    CAN_Init(CAN1, &CAN_InitStructure);

#if 0
    /* CAN filter init */
    CAN_FilterInitStructure.CAN_FilterNumber=0;
    CAN_FilterInitStructure.CAN_FilterMode=CAN_FilterMode_IdMask;
    CAN_FilterInitStructure.CAN_FilterScale=CAN_FilterScale_32bit;
    CAN_FilterInitStructure.CAN_FilterIdHigh=0x0000;
    CAN_FilterInitStructure.CAN_FilterIdLow=0x0000;
    CAN_FilterInitStructure.CAN_FilterMaskIdHigh=0x0000;
    CAN_FilterInitStructure.CAN_FilterMaskIdLow=0x0000;
    CAN_FilterInitStructure.CAN_FilterFIFOAssignment=0;
    CAN_FilterInitStructure.CAN_FilterActivation=ENABLE;
    CAN_FilterInit(&CAN_FilterInitStructure);
#endif
}

void CAN_Close(void)
{
    CAN_DeInit(CAN1);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN1, DISABLE);
}

uint8_t CAN_ReceiveMsg(CanMessage_t *msg)
{
    CanRxMsg RxMessage;

    CAN_Receive(CAN1, CAN_FIFO0, &RxMessage);
    if (CAN_ID_STD == RxMessage.IDE)
    {
        msg->id = RxMessage.StdId;
        msg->len = RxMessage.DLC;
        memmove(msg->payload, RxMessage.Data, 8);
#ifdef DEBUG_LOG_CAN_FRAME
        DebugCanFrame(0, 0, msg);
#endif
        return 0;
    }
    else //CAN_ID_EXT
    {
        msg->id = RxMessage.ExtId;
        msg->len = RxMessage.DLC;
        memmove(msg->payload, RxMessage.Data, 8);
#ifdef DEBUG_LOG_CAN_FRAME
        DebugCanFrame(0, 1, msg);
#endif
        return 1;
    }
}

uint8_t CAN_SendMsg(CanMessage_t *msg)
{
    CanTxMsg TxMessage;
    uint8_t TransmitMailbox, i;

    if (0 == extId)
    {
        TxMessage.StdId = msg->id;
        TxMessage.ExtId = 0;
        TxMessage.IDE = CAN_ID_STD;
    }
    else
    {
        TxMessage.StdId = 0;
        TxMessage.ExtId = msg->id;
        TxMessage.IDE = CAN_ID_EXT;
    }

    TxMessage.RTR = CAN_RTR_DATA;
    TxMessage.DLC = msg->len;

    memmove(TxMessage.Data, msg->payload, 8);

    TransmitMailbox=CAN_Transmit(CAN1, &TxMessage);
    if (CAN_NO_MB == TransmitMailbox) {
        return 0;
    }

    i = 0;
    while((CAN_TransmitStatus(CAN1, TransmitMailbox) != CANTXOK))
    {
        i++;
        os_delay_ms(10);
        if(i >= 0xff) {
            loge("%s: timeout", __func__);
            break;
        }
    }

    if (i != 0xff)
    {
#ifdef DEBUG_LOG_CAN_FRAME
        DebugCanFrame(1, extId, msg);
#endif
        return 255;
    }
    else
    {
        return 0; //can transmit fails
    }
}
void CAN_SetFilter0(uint32_t id)
{
    CAN_FilterInitTypeDef  CAN_FilterInitStructure;
    /* CAN filter init */
    CAN_FilterInitStructure.CAN_FilterNumber=0;
    CAN_FilterInitStructure.CAN_FilterMode=CAN_FilterMode_IdList; //CAN_FilterMode_IdMask;
    CAN_FilterInitStructure.CAN_FilterScale=CAN_FilterScale_32bit;

    CAN_FilterInitStructure.CAN_FilterIdHigh=(id & 0x07ff) << 5L; 
    CAN_FilterInitStructure.CAN_FilterIdLow=0x0000; //IDE=0

    CAN_FilterInitStructure.CAN_FilterMaskIdHigh=(((id+1) & 0x3E000) >> 13L); 
    CAN_FilterInitStructure.CAN_FilterMaskIdLow= (((id+1) & 0x01fff) << 3L) | 0x4; //IDE=1
    CAN_FilterInitStructure.CAN_FilterFIFOAssignment=0;
    CAN_FilterInitStructure.CAN_FilterActivation=ENABLE;
    CAN_FilterInit(&CAN_FilterInitStructure);
}


void CAN_ResetFilter0(void)
{
    CAN_FilterInitTypeDef  CAN_FilterInitStructure;
    /* CAN filter init */
    CAN_FilterInitStructure.CAN_FilterNumber=0;
    CAN_FilterInitStructure.CAN_FilterMode=CAN_FilterMode_IdMask;
    CAN_FilterInitStructure.CAN_FilterScale=CAN_FilterScale_32bit;

    CAN_FilterInitStructure.CAN_FilterIdHigh=0x0000;
    CAN_FilterInitStructure.CAN_FilterIdLow=0x0000; 

    CAN_FilterInitStructure.CAN_FilterMaskIdHigh=0x0000;
    CAN_FilterInitStructure.CAN_FilterMaskIdLow= 0x0000;
    CAN_FilterInitStructure.CAN_FilterFIFOAssignment=0;
    CAN_FilterInitStructure.CAN_FilterActivation=ENABLE;
    CAN_FilterInit(&CAN_FilterInitStructure);
}


void CAN_FlushReceiveFifo()
{
    CanRxMsg RxMessage;

    while (CAN_MessagePending(CAN1, CAN_FIFO0))
    {
        CAN_Receive(CAN1, CAN_FIFO0, &RxMessage);
    }
}

