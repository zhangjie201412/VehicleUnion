#include "bsp.h"
#include "kwp2000.h"
#include "drivers.h"

static USART_T g_Usart;
static uint8_t g_TxBuf[KWP2000_TX_BUF_SIZE];
static uint8_t g_RxBuf[KWP2000_RX_BUF_SIZE];

static void kwp2000_UartVarInit(void);
static void kwp2000_InitHardUart(void);
static void kwp2000_UartSend(USART_T *_pUart, uint8_t *_ucaBuf, uint16_t _usLen);
static uint8_t kwp2000_UartGetChar(USART_T *_pUart, uint8_t *_pByte);
static void kwp2000_UartIRQ(USART_T *_pUart);
static void kwp2000_ConfigUartNVIC(void);

static void kwp2000_active(void);

void kwp2000_sample(void)
{
    uint8_t send_cmd[5] = {0x81, 0x2a, 0xf0, 0x81, 0x1c};
    uint8_t ret;
    uint8_t u_data;

    logi("%s: E", __func__);
    kwp2000_InitGpio();
    kwp2000_active();
    kwp2000_InitUart();
    kwp2000_ClearRxFifo();
    kwp2000_ClearTxFifo();
    kwp2000_SendBuf(send_cmd, 5); 

    os_delay(1);
    while(TRUE) {
        ret = kwp2000_GetChar(&u_data);
        if(ret == 0) {
            break;
        } else {
            logi("%02x", u_data);
        }
    }
    logi("%s: X", __func__);
}

void kwp2000_InitGpio(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_AFIO, ENABLE);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

    GPIO_WriteBit(GPIOA, GPIO_Pin_9, Bit_SET);
}

void kwp2000_InitUart(void)
{
	kwp2000_UartVarInit();		/* 必须先初始化全局变量,再配置硬件 */
	kwp2000_InitHardUart();		/* 配置串口的硬件参数(波特率等) */
	kwp2000_ConfigUartNVIC();	/* 配置串口中断 */
}

void kwp2000_SendBuf(uint8_t *_ucaBuf, uint16_t _usLen)
{
	USART_T *pUart;

	pUart = &g_Usart;
	if (pUart == 0)
	{
		return;
	}

	kwp2000_UartSend(pUart, _ucaBuf, _usLen);
}

void kwp2000_SendChar(uint8_t _ucByte)
{
	kwp2000_SendBuf(&_ucByte, 1);
}

uint8_t kwp2000_GetChar(uint8_t *_pByte)
{
	USART_T *pUart;

	pUart = &g_Usart;
	if (pUart == 0)
	{
		return 0;
	}

	return kwp2000_UartGetChar(pUart, _pByte);
}

void kwp2000_ClearTxFifo(void)
{
	USART_T *pUart;

	pUart = &g_Usart;
	if (pUart == 0)
	{
		return;
	}

	pUart->usTxWrite = 0;
	pUart->usTxRead = 0;
	pUart->usTxCount = 0;
}

void kwp2000_ClearRxFifo(void)
{
	USART_T *pUart;

	pUart = &g_Usart;
	if (pUart == 0)
	{
		return;
	}

	pUart->usRxWrite = 0;
	pUart->usRxRead = 0;
	pUart->usRxCount = 0;
}

static void kwp2000_active(void)
{
    GPIO_WriteBit(GPIOA, GPIO_Pin_9, Bit_SET);
    os_delay(1);
    GPIO_WriteBit(GPIOA, GPIO_Pin_9, Bit_RESET);
    os_delay_ms(25);
    GPIO_WriteBit(GPIOA, GPIO_Pin_9, Bit_SET);
    os_delay_ms(25);
}

static void kwp2000_UartVarInit(void)
{
	g_Usart.uart = USART1;						/* STM32 串口设备 */
	g_Usart.pTxBuf = g_TxBuf;					/* 发送缓冲区指针 */
	g_Usart.pRxBuf = g_RxBuf;					/* 接收缓冲区指针 */
	g_Usart.usTxBufSize = KWP2000_TX_BUF_SIZE;	/* 发送缓冲区大小 */
	g_Usart.usRxBufSize = KWP2000_RX_BUF_SIZE;	/* 接收缓冲区大小 */
	g_Usart.usTxWrite = 0;						/* 发送FIFO写索引 */
	g_Usart.usTxRead = 0;						/* 发送FIFO读索引 */
	g_Usart.usRxWrite = 0;						/* 接收FIFO写索引 */
	g_Usart.usRxRead = 0;						/* 接收FIFO读索引 */
	g_Usart.usRxCount = 0;						/* 接收到的新数据个数 */
	g_Usart.usTxCount = 0;						/* 待发送的数据个数 */
}

static void kwp2000_InitHardUart(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;

	/* 第1步：打开GPIO和USART部件的时钟 */
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_AFIO, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);

	/* 第2步：将USART Tx的GPIO配置为推挽复用模式 */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	/* 第3步：将USART Rx的GPIO配置为浮空输入模式
		由于CPU复位后，GPIO缺省都是浮空输入模式，因此下面这个步骤不是必须的
		但是，我还是建议加上便于阅读，并且防止其它地方修改了这个口线的设置参数
	*/
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	
	/* 第4步： 配置串口硬件参数 */
	USART_InitStructure.USART_BaudRate = KWP2000_BAUD;	/* 波特率 */
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No ;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
	USART_Init(USART1, &USART_InitStructure);

	USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);	/* 使能接收中断 */
	/*
		USART_ITConfig(USART1, USART_IT_TXE, ENABLE);
		注意: 不要在此处打开发送中断
		发送中断使能在SendUart()函数打开
	*/
	USART_Cmd(USART1, ENABLE);		/* 使能串口 */

	/* CPU的小缺陷：串口配置好，如果直接Send，则第1个字节发送不出去
		如下语句解决第1个字节无法正确发送出去的问题 */
	USART_ClearFlag(USART1, USART_FLAG_TC);     /* 清发送完成标志，Transmission Complete flag */
}

static void kwp2000_ConfigUartNVIC(void)
{
	NVIC_InitTypeDef NVIC_InitStructure;

	/* Configure the NVIC Preemption Priority Bits */
	/*	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_0);  --- 在 bsp.c 中 bsp_Init() 中配置中断优先级组 */

	/* 使能串口1中断 */
	NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
}

static void kwp2000_UartSend(USART_T *_pUart, uint8_t *_ucaBuf, uint16_t _usLen)
{
	uint16_t i;

	for (i = 0; i < _usLen; i++)
	{
		/* 当 _pUart->usTxBufSize == 1 时, 下面的函数会死掉(待完善) */
		while (1)
		{
			__IO uint16_t usCount;

			DISABLE_INT();
			usCount = _pUart->usTxCount;
			ENABLE_INT();

			if (usCount < _pUart->usTxBufSize)
			{
				break;
			}
		}

		/* 将新数据填入发送缓冲区 */
		_pUart->pTxBuf[_pUart->usTxWrite] = _ucaBuf[i];

		DISABLE_INT();
		if (++_pUart->usTxWrite >= _pUart->usTxBufSize)
		{
			_pUart->usTxWrite = 0;
		}
		_pUart->usTxCount++;
		ENABLE_INT();
	}

	USART_ITConfig(_pUart->uart, USART_IT_TXE, ENABLE);
}

static uint8_t kwp2000_UartGetChar(USART_T *_pUart, uint8_t *_pByte)
{
	uint16_t usCount;

	/* usRxWrite 变量在中断函数中被改写，主程序读取该变量时，必须进行临界区保护 */
	DISABLE_INT();
	usCount = _pUart->usRxCount;
	ENABLE_INT();

	/* 如果读和写索引相同，则返回0 */
	//if (_pUart->usRxRead == usRxWrite)
	if (usCount == 0)	/* 已经没有数据 */
	{
		return 0;
	}
	else
	{
		*_pByte = _pUart->pRxBuf[_pUart->usRxRead];		/* 从串口接收FIFO取1个数据 */

		/* 改写FIFO读索引 */
		DISABLE_INT();
		if (++_pUart->usRxRead >= _pUart->usRxBufSize)
		{
			_pUart->usRxRead = 0;
		}
		_pUart->usRxCount--;
		ENABLE_INT();
		return 1;
	}
}

static void kwp2000_UartIRQ(USART_T *_pUart)
{
	/* 处理接收中断  */
	if (USART_GetITStatus(_pUart->uart, USART_IT_RXNE) != RESET)
	{
		/* 从串口接收数据寄存器读取数据存放到接收FIFO */
		uint8_t ch;

		ch = USART_ReceiveData(_pUart->uart);
		_pUart->pRxBuf[_pUart->usRxWrite] = ch;
		if (++_pUart->usRxWrite >= _pUart->usRxBufSize)
		{
			_pUart->usRxWrite = 0;
		}
		if (_pUart->usRxCount < _pUart->usRxBufSize)
		{
			_pUart->usRxCount++;
		}

	}

	/* 处理发送缓冲区空中断 */
	if (USART_GetITStatus(_pUart->uart, USART_IT_TXE) != RESET)
	{
		//if (_pUart->usTxRead == _pUart->usTxWrite)
		if (_pUart->usTxCount == 0)
		{
			/* 发送缓冲区的数据已取完时， 禁止发送缓冲区空中断 （注意：此时最后1个数据还未真正发送完毕）*/
			USART_ITConfig(_pUart->uart, USART_IT_TXE, DISABLE);

			/* 使能数据发送完毕中断 */
			USART_ITConfig(_pUart->uart, USART_IT_TC, ENABLE);
		}
		else
		{
			/* 从发送FIFO取1个字节写入串口发送数据寄存器 */
			USART_SendData(_pUart->uart, _pUart->pTxBuf[_pUart->usTxRead]);
			if (++_pUart->usTxRead >= _pUart->usTxBufSize)
			{
				_pUart->usTxRead = 0;
			}
			_pUart->usTxCount--;
		}

	}
	/* 数据bit位全部发送完毕的中断 */
	else if (USART_GetITStatus(_pUart->uart, USART_IT_TC) != RESET)
	{
		//if (_pUart->usTxRead == _pUart->usTxWrite)
		if (_pUart->usTxCount == 0)
		{
			/* 如果发送FIFO的数据全部发送完毕，禁止数据发送完毕中断 */
			USART_ITConfig(_pUart->uart, USART_IT_TC, DISABLE);
		}
		else
		{
			/* 正常情况下，不会进入此分支 */

			/* 如果发送FIFO的数据还未完毕，则从发送FIFO取1个数据写入发送数据寄存器 */
			USART_SendData(_pUart->uart, _pUart->pTxBuf[_pUart->usTxRead]);
			if (++_pUart->usTxRead >= _pUart->usTxBufSize)
			{
				_pUart->usTxRead = 0;
			}
			_pUart->usTxCount--;
		}
	}
}

void USART1_IRQHandler(void)
{
	kwp2000_UartIRQ(&g_Usart);
}
