/*!
*  \file hmi_user_uart.h
*  \brief 串口屏通信底层接口（HAL 版，USART1 = RXD0/TXD0）
*  \version 1.0
*  \date 2026-10-22
*  \copyright 广州大彩光电科技有限公司（协议部分）
*/
#ifndef _USER_UART__
#define _USER_UART__

#include "main.h"

#define uchar    unsigned char
#define uint8    unsigned char
#define uint16   unsigned short int
#define uint32   unsigned long
#define int16    short int
#define int32    long

/****************************************************************************
* 名    称： UartInit()
* 功    能： 串口初始化（并启动中断接收）
* 入口参数： BaudRate 波特率
****************************************************************************/
void UartInit(uint32 BaudRate);

/*****************************************************************
* 名    称： SendChar()
* 功    能： 发送1个字节
* 入口参数： t  发送的字节
*****************************************************************/
void SendChar(uchar t);

#endif
