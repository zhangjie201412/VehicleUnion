#ifndef __CAN_H__
#define __CAN_H__

#include "stm32f10x.h"


typedef enum {
    CAN_PIN_1_9,
    CAN_PIN_3_11,
    CAN_PIN_6_14,
};

typedef struct {
    uint16_t id;
    uint8_t  len;
    uint8_t  payload[8];
} CanMessage_t;

void CAN_InitSwitch(void);
void CAN_SwitchMode(uint8_t mode);
void CAN_Open(void);
void CAN_Close(void);
uint8_t CAN_ReceiveMsg(CanMessage_t *msg);
uint8_t CAN_SendMsg(CanMessage_t *msg);
void CAN_SetFilter0(uint32_t id);
void CAN_ResetFilter0(void);
void CAN_FlushReceiveFifo(void);
void CAN_UseStdId(void);
void CAN_UseExtId(void);

void can_sample(void);

#endif
