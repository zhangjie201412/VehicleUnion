#ifndef __KWP2000_H__
#define __KWP2000_H__

#define KWP2000_BAUD                10400
#define KWP2000_TX_BUF_SIZE         128
#define KWP2000_RX_BUF_SIZE         1024

typedef struct {
    USART_TypeDef *uart;
    uint8_t *pTxBuf;
    uint8_t *pRxBuf;
    uint16_t usTxBufSize;
    uint16_t usRxBufSize;
    __IO uint16_t usTxWrite;
    __IO uint16_t usTxRead;
    __IO uint16_t usTxCount;
    __IO uint16_t usRxWrite;
    __IO uint16_t usRxRead;
    __IO uint16_t usRxCount;
} USART_T;

void kwp2000_InitGpio(void);
void kwp2000_InitUart(void);
void kwp2000_SendBuf(uint8_t *buf, uint16_t len);
void kwp2000_SendChar(uint8_t byte);
uint8_t kwp2000_GetChar(uint8_t *_pByte);
void kwp2000_ClearRxFifo(void);
void kwp2000_ClearTxFifo(void);

void kwp2000_sample(void);

#endif
