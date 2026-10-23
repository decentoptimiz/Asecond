/************************************版权申明********************************************
**                             广州大彩光电科技有限公司
**                             http://www.gz-dc.com
**-----------------------------------文件信息--------------------------------------------
** 文件名称:   hmi_user_uart.c
** 文件说明:   串口屏通信底层驱动（HAL 版，USART1 = PA9/TX, PA10/RX）
** 移植说明:   仅需实现 UartInit / SendChar 两个底层函数，协议层 hmi_driver.c 无需改动
--------------------------------------------------------------------------------------*/
#include "hmi_user_uart.h"
#include "usart.h"
#include "cmd_queue.h"

static uint8_t hmi_rx_byte = 0;

/*!
*   \brief   串口初始化（并启动中断接收）
*   \param   BaudRate 波特率
*/
void UartInit(uint32 BaudRate)
{
    /* USART1 已在 MX_USART1_UART_Init 中初始化（PA9=TX, PA10=RX, 8N1）*/
    if(huart1.Init.BaudRate != BaudRate)
    {
        huart1.Init.BaudRate = BaudRate;
        HAL_UART_DeInit(&huart1);
        if(HAL_UART_Init(&huart1) != HAL_OK)
        {
            Error_Handler();
        }
    }

    /* 启动单字节中断接收，数据在 HAL_UART_RxCpltCallback 中进入命令队列 */
    HAL_UART_Receive_IT(&huart1, &hmi_rx_byte, 1);
}

/*!
*   \brief  发送1个字节（阻塞）
*   \param  t 发送的字节
*/
void SendChar(uchar t)
{
    uint8_t b = (uint8_t)t;
    HAL_UART_Transmit(&huart1, &b, 1, 100);
}

/*!
*   \brief  USART1 接收中断回调
*   \detail 每收到1字节，送入指令队列并重新启动接收
*/
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if(huart->Instance == USART1)
    {
        queue_push(hmi_rx_byte);
        HAL_UART_Receive_IT(&huart1, &hmi_rx_byte, 1);
    }
}
