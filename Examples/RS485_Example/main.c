/**
 ******************************************************************************
 * @file    main.c
 * @brief   RS485驱动测试例程
 * @note    本例程演示如何使用RS485驱动读取Modbus设备
 ******************************************************************************
 */

/* 头文件包含 ----------------------------------------------------------------*/
#include "main.h"
#include "rs485_driver.h"
#include <stdio.h>
#include <string.h>

/* 私有宏定义 ----------------------------------------------------------------*/
#define LED_PIN         GPIO_PIN_13     // 状态指示灯（可根据实际板子修改）
#define LED_PORT        GPIOC

/* 私有变量 ------------------------------------------------------------------*/
UART_HandleTypeDef huart1;              // UART句柄
RS485_Handle_t rs485_device;            // RS485设备句柄

uint8_t test_counter = 0;               // 测试计数器
uint32_t last_test_time = 0;           // 上次测试时间

/* 私有函数声明 --------------------------------------------------------------*/
static void SystemClock_Config(void);
static void GPIO_Init(void);
static void UART_Init(void);
static void LED_Toggle(void);
static void RS485_Test_Task(void);

/* Modbus请求帧示例 */
static void Modbus_ReadHoldingRegs(uint8_t slave_addr, uint16_t reg_addr, uint16_t reg_count);
static uint16_t CRC16_Modbus(uint8_t *data, uint16_t len);

/* 回调函数 */
static void RS485_TxComplete_Callback(void);
static void RS485_RxComplete_Callback(uint8_t *data, uint16_t len);
static void RS485_Error_Callback(RS485_Error_t error);

/**
 * @brief  主函数
 */
int main(void)
{
    /* HAL库初始化 */
    HAL_Init();
    
    /* 配置系统时钟 */
    SystemClock_Config();
    
    /* 初始化GPIO */
    GPIO_Init();
    
    /* 初始化UART */
    UART_Init();
    
    /* 初始化RS485驱动 */
    RS485_Error_t ret = RS485_Init(&rs485_device, 
                                   &huart1,              // UART1
                                   GPIOA,                // DE引脚在PA12
                                   GPIO_PIN_12);
    
    if (ret != RS485_OK) {
        // 初始化失败，LED快闪
        while(1) {
            LED_Toggle();
            HAL_Delay(100);
        }
    }
    
    /* 注册回调函数 */
    rs485_device.tx_complete_callback = RS485_TxComplete_Callback;
    rs485_device.rx_complete_callback = RS485_RxComplete_Callback;
    rs485_device.error_callback = RS485_Error_Callback;
    
    /* 主循环 */
    while (1)
    {
        /* 处理RS485状态机 */
        RS485_Process(&rs485_device);
        
        /* 定时测试任务（每2秒发送一次） */
        if (HAL_GetTick() - last_test_time >= 2000) {
            last_test_time = HAL_GetTick();
            RS485_Test_Task();
        }
        
        /* 其他任务可以在这里添加 */
        // ...
    }
}

/**
 * @brief  RS485测试任务
 */
static void RS485_Test_Task(void)
{
    RS485_State_t state = RS485_GetState(&rs485_device);
    
    // 只有在空闲状态才能发送新请求
    if (state == RS485_STATE_IDLE) {
        test_counter++;
        
        // 发送Modbus读保持寄存器命令
        // 从机地址=0x01, 寄存器地址=0x0000, 读取数量=2个寄存器
        Modbus_ReadHoldingRegs(0x01, 0x0000, 2);
        
        LED_Toggle();  // 发送时LED翻转
        
    } else if (state == RS485_STATE_RX_DONE) {
        // 接收完成，可以处理数据
        // 注意：实际应用中应该在回调函数中处理
        
        // 读取完成后，重置状态（重要！）
        rs485_device.state = RS485_STATE_IDLE;
        
    } else if (state == RS485_STATE_TIMEOUT) {
        // 超时了，重置状态重试
        rs485_device.state = RS485_STATE_IDLE;
    }
}

/**
 * @brief  发送Modbus读保持寄存器命令
 * @param  slave_addr: 从机地址
 * @param  reg_addr: 寄存器起始地址
 * @param  reg_count: 读取寄存器数量
 */
static void Modbus_ReadHoldingRegs(uint8_t slave_addr, uint16_t reg_addr, uint16_t reg_count)
{
    uint8_t modbus_frame[8];
    uint16_t crc;
    
    // 构造Modbus RTU帧
    modbus_frame[0] = slave_addr;           // 从机地址
    modbus_frame[1] = 0x03;                 // 功能码：读保持寄存器
    modbus_frame[2] = (reg_addr >> 8);      // 寄存器地址高字节
    modbus_frame[3] = (reg_addr & 0xFF);    // 寄存器地址低字节
    modbus_frame[4] = (reg_count >> 8);     // 数量高字节
    modbus_frame[5] = (reg_count & 0xFF);   // 数量低字节
    
    // 计算CRC
    crc = CRC16_Modbus(modbus_frame, 6);
    modbus_frame[6] = (crc & 0xFF);         // CRC低字节
    modbus_frame[7] = (crc >> 8);           // CRC高字节
    
    // 发送帧，期望接收9字节响应（地址1+功能码1+字节数1+数据4+CRC2）
    RS485_TransmitReceive(&rs485_device, modbus_frame, 8, 9, 1000);
}

/**
 * @brief  计算Modbus CRC16校验
 * @param  data: 数据指针
 * @param  len: 数据长度
 * @retval CRC16值
 */
static uint16_t CRC16_Modbus(uint8_t *data, uint16_t len)
{
    uint16_t crc = 0xFFFF;
    
    for (uint16_t i = 0; i < len; i++) {
        crc ^= data[i];
        for (uint8_t j = 0; j < 8; j++) {
            if (crc & 0x0001) {
                crc = (crc >> 1) ^ 0xA001;
            } else {
                crc = crc >> 1;
            }
        }
    }
    
    return crc;
}

/* 回调函数实现 --------------------------------------------------------------*/

/**
 * @brief  发送完成回调
 */
static void RS485_TxComplete_Callback(void)
{
    // 可以在这里添加调试信息
    // printf("TX Complete\r\n");
}

/**
 * @brief  接收完成回调
 * @param  data: 接收到的数据
 * @param  len: 数据长度
 */
static void RS485_RxComplete_Callback(uint8_t *data, uint16_t len)
{
    // 可以在这里处理接收到的数据
    // printf("RX Complete: ");
    // for(uint16_t i = 0; i < len; i++) {
    //     printf("%02X ", data[i]);
    // }
    // printf("\r\n");
    
    LED_Toggle();  // 接收成功LED翻转
}

/**
 * @brief  错误回调
 * @param  error: 错误代码
 */
static void RS485_Error_Callback(RS485_Error_t error)
{
    // 可以在这里处理错误
    switch(error) {
        case RS485_ERR_TIMEOUT:
            // printf("Error: Timeout\r\n");
            break;
        case RS485_ERR_CRC:
            // printf("Error: CRC\r\n");
            break;
        default:
            // printf("Error: %d\r\n", error);
            break;
    }
}

/* 硬件初始化函数 ------------------------------------------------------------*/

/**
 * @brief  GPIO初始化
 */
static void GPIO_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    
    /* 使能时钟 */
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();
    
    /* 配置LED引脚 */
    GPIO_InitStruct.Pin = LED_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(LED_PORT, &GPIO_InitStruct);
    
    /* 配置RS485 DE引脚 */
    GPIO_InitStruct.Pin = GPIO_PIN_12;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
    
    /* DE引脚初始化为低电平 */
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_12, GPIO_PIN_RESET);
}

/**
 * @brief  UART初始化
 */
static void UART_Init(void)
{
    /* UART1配置 */
    huart1.Instance = USART1;
    huart1.Init.BaudRate = 9600;            // 波特率9600（Modbus常用）
    huart1.Init.WordLength = UART_WORDLENGTH_8B;
    huart1.Init.StopBits = UART_STOPBITS_1;
    huart1.Init.Parity = UART_PARITY_NONE;
    huart1.Init.Mode = UART_MODE_TX_RX;
    huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    huart1.Init.OverSampling = UART_OVERSAMPLING_16;
    
    if (HAL_UART_Init(&huart1) != HAL_OK) {
        Error_Handler();
    }
}

/**
 * @brief  LED翻转
 */
static void LED_Toggle(void)
{
    HAL_GPIO_TogglePin(LED_PORT, LED_PIN);
}

/**
 * @brief  系统时钟配置（根据你的MCU调整）
 */
static void SystemClock_Config(void)
{
    // 这里需要根据你的具体MCU配置
    // 可以用STM32CubeMX生成
}

/**
 * @brief  错误处理
 */
void Error_Handler(void)
{
    while(1) {
        LED_Toggle();
        HAL_Delay(200);
    }
}