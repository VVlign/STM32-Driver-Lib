/**
 ******************************************************************************
 * @file    stm32xxxx_it.c
 * @brief   中断处理函数
 * @note    需要根据MCU修改文件名（如stm32f1xx_it.c）
 ******************************************************************************
 */

#include "main.h"
#include "rs485_driver.h"

/* 外部变量声明 --------------------------------------------------------------*/
extern UART_HandleTypeDef huart1;
extern RS485_Handle_t rs485_device;

/* 中断处理函数 --------------------------------------------------------------*/

/**
 * @brief  SysTick中断处理
 */
void SysTick_Handler(void)
{
    HAL_IncTick();
}

/**
 * @brief  UART1中断处理
 */
void USART1_IRQHandler(void)
{
    HAL_UART_IRQHandler(&huart1);
}

/* HAL库回调函数 -------------------------------------------------------------*/

/**
 * @brief  UART发送完成回调
 * @note   在这里调用RS485驱动的回调
 */
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART1) {
        // 调用RS485驱动的发送完成回调
        RS485_TxCpltCallback(&rs485_device);
    }
}

/**
 * @brief  UART接收完成回调
 * @note   在这里调用RS485驱动的回调
 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART1) {
        // 调用RS485驱动的接收完成回调
        RS485_RxCpltCallback(&rs485_device);
    }
}