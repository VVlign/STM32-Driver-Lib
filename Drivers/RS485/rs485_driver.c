/**
 ******************************************************************************
 * @file    rs485_driver.c
 * @brief   STM32通用RS485通信驱动实现
 * @author  VVlign
 * @date    2025-01-29
 ******************************************************************************
 */

/* 头文件包含 ----------------------------------------------------------------*/
#include "rs485_driver.h"
#include <string.h>  // memcpy, memset

/* 
 * 注意：这里需要根据你的MCU包含对应的HAL头文件
 * 由于我们要做通用驱动，这里用条件编译
 */
#if defined(STM32F103xB) || defined(STM32F103xE)
    #include "stm32f1xx_hal.h"
#elif defined(STM32L431xx)
    #include "stm32l4xx_hal.h"
#elif defined(STM32G030xx)
    #include "stm32g0xx_hal.h"
#else
    #warning "Unknown MCU, please add HAL header manually"
#endif

/* 私有宏定义 ----------------------------------------------------------------*/
#define RS485_DE_HIGH(handle)   HAL_GPIO_WritePin((GPIO_TypeDef*)(handle)->de_gpio_port, \
                                                   (handle)->de_gpio_pin, GPIO_PIN_SET)
#define RS485_DE_LOW(handle)    HAL_GPIO_WritePin((GPIO_TypeDef*)(handle)->de_gpio_port, \
                                                   (handle)->de_gpio_pin, GPIO_PIN_RESET)

/* 私有函数声明 --------------------------------------------------------------*/
static uint32_t RS485_GetTick(void);
static bool RS485_IsTimeout(uint32_t start_time, uint32_t timeout_ms);

RS485_Handle_t rs485_device; 

/* 私有函数实现 --------------------------------------------------------------*/

/**
 * @brief  获取系统时钟（毫秒）
 * @retval 当前时间戳
 */
static uint32_t RS485_GetTick(void)
{
    return HAL_GetTick();
}

/**
 * @brief  检查是否超时
 * @param  start_time: 起始时间
 * @param  timeout_ms: 超时时间
 * @retval true=超时, false=未超时
 */
static bool RS485_IsTimeout(uint32_t start_time, uint32_t timeout_ms)
{
    uint32_t current = RS485_GetTick();
    
    // 处理时钟溢出的情况（HAL_GetTick每49.7天溢出一次）
    if (current >= start_time) {
        return (current - start_time) >= timeout_ms;
    } else {
        // 溢出后的计算
        return ((0xFFFFFFFF - start_time) + current) >= timeout_ms;
    }
}

/* 导出函数实现 --------------------------------------------------------------*/

/**
 * @brief  初始化RS485驱动
 * @param  handle: RS485句柄指针
 * @param  uart_handle: UART句柄（如 &huart1）
 * @param  de_port: DE引脚端口（如 GPIOA）
 * @param  de_pin: DE引脚编号（如 GPIO_PIN_12）
 * @retval RS485_Error_t: 错误代码
 */
RS485_Error_t RS485_Init(RS485_Handle_t *handle, 
                         void *uart_handle,
                         void *de_port, 
                         uint16_t de_pin,
                         RS485_Mode_t mode)  // ← 新增参数)
{
    // 参数检查
    if (handle == NULL || uart_handle == NULL || de_port == NULL) {
        return RS485_ERR_INVALID_PARAM;
    }
    
    // 清零整个结构体
    memset(handle, 0, sizeof(RS485_Handle_t));
    
    // 保存硬件接口
    handle->uart_handle = uart_handle;
    handle->de_gpio_port = de_port;
    handle->de_gpio_pin = de_pin;
    handle->mode = mode;
    
    // 初始化状态
    handle->state = RS485_STATE_IDLE;
    handle->timeout_ms = RS485_DEFAULT_TIMEOUT;      // 接收超时1000ms
    
    // ========== 新增：发送超时设置 ========== 
    handle->tx_timeout_ms = 500;  // 发送超时500ms（根据波特率调整）
    
    /*****如果USART波特率可配置****
    handle->baudrate = 9600;

    // 动态计算
    uint32_t bit_time_us = 1000000 / handle->baudrate;  // 每位的微秒数
    uint32_t byte_time_ms = (bit_time_us * 10) / 1000;  // 每字节毫秒数
    handle->tx_timeout_ms = tx_len * byte_time_ms * 2;  // 2倍安全系数               ***************/
        
    // DE引脚初始化为低电平（接收模式）
    RS485_DE_LOW(handle);
    
    return RS485_OK;
}

/**
 * @brief  发送数据并等待响应
 * @param  handle: RS485句柄指针
 * @param  tx_data: 要发送的数据
 * @param  tx_len: 发送数据长度
 * @param  rx_expected: 期望接收长度（0表示不接收）
 * @param  timeout_ms: 超时时间(毫秒)
 * @retval RS485_Error_t: 错误代码
 */
RS485_Error_t RS485_TransmitReceive(RS485_Handle_t *handle,
                                    const uint8_t *tx_data,
                                    uint16_t tx_len,
                                    uint16_t rx_expected,
                                    uint32_t timeout_ms)
{
    // 参数检查
    if (handle == NULL || tx_data == NULL || tx_len == 0) {
        // ========== 记录错误 ========== 
        if (handle != NULL) {
            handle->last_error = RS485_ERR_INVALID_PARAM;
        }
        return RS485_ERR_INVALID_PARAM;
    }
    
    if (tx_len > RS485_TX_BUFFER_SIZE) {
        handle->last_error = RS485_ERR_INVALID_PARAM;
        return RS485_ERR_INVALID_PARAM;
    }
    
    if (rx_expected > RS485_RX_BUFFER_SIZE) {
        handle->last_error = RS485_ERR_INVALID_PARAM;
        return RS485_ERR_INVALID_PARAM;
    }
    
    // 检查设备是否空闲
    if (handle->state != RS485_STATE_IDLE) {
        handle->last_error = RS485_ERR_BUSY;
        return RS485_ERR_BUSY;
    }
    
    // 复制数据到内部缓冲区
    memcpy(handle->tx_buffer, tx_data, tx_len);
    handle->tx_size = tx_len;
    handle->rx_expected = rx_expected;
    handle->rx_size = 0;
    handle->timeout_ms = timeout_ms;
    
    // ========== 动态计算发送超时 ========== 
    // 假设波特率9600，每字节约1ms10位）
    // 实际应该根据UART配置动态计算
    uint32_t tx_time_estimate = (tx_len * 10 * 1000) / 9600;
    handle->tx_timeout_ms = tx_time_estimate * 2 + 50;
    
    handle->last_tx_time = RS485_GetTick();
    
    // 切换到发送模式
    RS485_DE_HIGH(handle);
    for(volatile uint32_t i = 0; i < 100; i++);
    handle->state = RS485_STATE_TX;
    
    UART_HandleTypeDef *huart = (UART_HandleTypeDef *)handle->uart_handle;
    HAL_StatusTypeDef hal_status;
    
    // ========== 根据模式选择发送方式 ========== 
    if (handle->mode == RS485_MODE_DMA) {
        // DMA方式发送
        hal_status = HAL_UART_Transmit_DMA(huart, handle->tx_buffer, handle->tx_size);
    } else {
        // 中断方式发送（默认）
        hal_status = HAL_UART_Transmit_IT(huart, handle->tx_buffer, handle->tx_size);
    }
    
    if (hal_status != HAL_OK) {
        // ========== 发送启动失败，记录错误 ========== 
        RS485_DE_LOW(handle);
        handle->state = RS485_STATE_IDLE;
        handle->last_error = RS485_ERR_HW_FAULT;
        handle->tx_error_count++;  // 统计
        return RS485_ERR_HW_FAULT;
    }
    
    // ========== 成功启动发送 ========== 
    handle->last_error = RS485_OK;
    return RS485_OK;
}

/**
 * @brief  处理RS485状态机（在主循环中调用）
 * @param  handle: RS485句柄指针
 * @retval RS485_Error_t: 错误代码
 */
RS485_Error_t RS485_Process(RS485_Handle_t *handle)
{
    if (handle == NULL) {
        return RS485_ERR_INVALID_PARAM;
    }
    
    UART_HandleTypeDef *huart = (UART_HandleTypeDef *)handle->uart_handle;
    
    switch (handle->state) {
        case RS485_STATE_IDLE:
        case RS485_STATE_RX_DONE:
            // 这些状态不需要处理
            break;
        
        // ========== 发送状态：检测超时 ========== 
        case RS485_STATE_TX:
            if (RS485_IsTimeout(handle->last_tx_time, handle->tx_timeout_ms)) {
                // 发送超时
                HAL_UART_AbortTransmit_IT(huart);
                RS485_DE_LOW(handle);
                
                // 记录错误
                handle->last_error = RS485_ERR_TIMEOUT;
                handle->tx_error_count++;
                handle->timeout_count++;
                
                // 判断是否重试
                if (handle->retry_count < handle->max_retries) {
                    handle->retry_count++;
                    handle->state = RS485_STATE_IDLE;  // 恢复到空闲，允许重试
                } else {
                    // 超过重试次数，进入错误状态
                    handle->state = RS485_STATE_ERROR;
                }
                
                // 调用用户错误回调
                if (handle->error_callback != NULL) {
                    handle->error_callback(RS485_ERR_TIMEOUT);
                }
            }
            break;
            
        // ========== 等待接收状态：检测超时 ========== 
        case RS485_STATE_WAIT_RX:
            if (RS485_IsTimeout(handle->last_tx_time, handle->timeout_ms)) {
                // 接收超时
                HAL_UART_AbortReceive_IT(huart);
                
                // 记录错误
                handle->last_error = RS485_ERR_TIMEOUT;
                handle->rx_error_count++;
                handle->timeout_count++;
                
                // 判断是否重试
                if (handle->retry_count < handle->max_retries) {
                    handle->retry_count++;
                    handle->state = RS485_STATE_IDLE;
                } else {
                    handle->state = RS485_STATE_TIMEOUT;
                }
                
                // 调用用户错误回调
                if (handle->error_callback != NULL) {
                    handle->error_callback(RS485_ERR_TIMEOUT);
                }
            }
            break;
        
        // ========== 超时状态：自动恢复 ========== 
        case RS485_STATE_TIMEOUT:
            // 延迟一段时间后自动恢复
            // 可以在这里检查是否满足恢复条件
            handle->state = RS485_STATE_IDLE;
            handle->retry_count = 0;  // 重置重试计数
            break;
        
        // ========== 错误状态：需要用户手动恢复 ========== 
        case RS485_STATE_ERROR:
            // 错误状态不会自动恢复
            // 用户需要调用 RS485_ClearError() 或 RS485_ForceReset()
//            RS485_ClearError(&rs485_device);
//            RS485_ForceReset(&rs485_device);
            break;
            
        default:
            break;
    }
    
    return RS485_OK;
}

/**
 * @brief  UART发送完成回调（在HAL_UART_TxCpltCallback中调用）
 * @param  handle: RS485句柄指针
 * @retval 无
 */
void RS485_TxCpltCallback(RS485_Handle_t *handle)
{
    if (handle == NULL || handle->state != RS485_STATE_TX) {
        return;
    }
    
    // 发送完成，切换到接收模式
    RS485_DE_LOW(handle);
    
    // 调用用户的发送完成回调
    if (handle->tx_complete_callback != NULL) {
        handle->tx_complete_callback();
    }
    
    // 判断是否需要接收响应
    if (handle->rx_expected > 0) {
        UART_HandleTypeDef *huart = (UART_HandleTypeDef *)handle->uart_handle;
        HAL_StatusTypeDef hal_status;
        
        // ========== 根据模式选择接收方式 ========== 
        if (handle->mode == RS485_MODE_DMA) {
            // DMA方式：使用空闲中断接收（不定长）
            hal_status = HAL_UARTEx_ReceiveToIdle_DMA(huart, 
                                                      handle->rx_buffer, 
                                                      RS485_RX_BUFFER_SIZE);
            // 禁用DMA半传输完成中断（可选）
            __HAL_DMA_DISABLE_IT(huart->hdmarx, DMA_IT_HT);
        } else {
            // 中断方式：定长接收
            hal_status = HAL_UART_Receive_IT(huart, 
                                             handle->rx_buffer, 
                                             handle->rx_expected);
        }
        
        if (hal_status == HAL_OK) {
            handle->state = RS485_STATE_WAIT_RX;
            handle->last_tx_time = RS485_GetTick();  // 重置超时计时
            handle->last_error = RS485_OK;
        } else {
            // ========== 启动接收失败 ========== 
            handle->state = RS485_STATE_ERROR;
            handle->last_error = RS485_ERR_HW_FAULT;
            handle->rx_error_count++;
            
            if (handle->error_callback != NULL) {
                handle->error_callback(RS485_ERR_HW_FAULT);
            }
        }
    } else {
        // 不需要接收，直接完成
        handle->state = RS485_STATE_IDLE;
        handle->retry_count = 0;  // 成功后重置重试计数
    }
}

/**
 * @brief  UART接收完成回调（在HAL_UART_RxCpltCallback中调用）
 * @param  handle: RS485句柄指针
 * @retval 无
 */
void RS485_RxCpltCallback(RS485_Handle_t *handle)
{
    if (handle == NULL || handle->state != RS485_STATE_WAIT_RX) {
        return;
    }
    
    // 接收完成
    handle->rx_size = handle->rx_expected;
    handle->state = RS485_STATE_RX_DONE;
    handle->last_error = RS485_OK;
    handle->retry_count = 0;  // 成功后重置重试计数
    
    // 调用用户的接收完成回调
    if (handle->rx_complete_callback != NULL) {
        handle->rx_complete_callback(handle->rx_buffer, handle->rx_size);
    }
}

/**
 * @brief  UART接收事件回调（DMA空闲中断）
 */
void RS485_RxEventCallback(RS485_Handle_t *handle, uint16_t Size)
{
    if (handle == NULL || handle->state != RS485_STATE_WAIT_RX) {
        return;
    }
    
    // DMA接收完成（触发了空闲中断）
    handle->rx_size = Size;  // 实际接收的字节数
    handle->state = RS485_STATE_RX_DONE;
    handle->last_error = RS485_OK;
    handle->retry_count = 0;
    
    // 调用用户的接收完成回调
    if (handle->rx_complete_callback != NULL) {
        handle->rx_complete_callback(handle->rx_buffer, handle->rx_size);
    }
}

/**
 * @brief  获取当前状态
 * @param  handle: RS485句柄指针
 * @retval RS485_State_t: 当前状态
 */
RS485_State_t RS485_GetState(const RS485_Handle_t *handle)
{
    if (handle == NULL) {
        return RS485_STATE_ERROR;
    }
    return handle->state;
}

/**
 * @brief  中止当前传输
 * @param  handle: RS485句柄指针
 * @retval 无
 */
void RS485_AbortTransfer(RS485_Handle_t *handle)
{
    if (handle == NULL) {
        return;
    }
    
    UART_HandleTypeDef *huart = (UART_HandleTypeDef *)handle->uart_handle;
    
    // 中止UART传输
    HAL_UART_Abort_IT(huart);
    
    // 恢复DE引脚
    RS485_DE_LOW(handle);
    
    // 重置状态
    handle->state = RS485_STATE_IDLE;
    handle->rx_size = 0;
    handle->tx_size = 0;
}

/* 错误处理函数实现 ---------------------------------------------------------*/

/**
 * @brief  获取最后一次错误码
 */
RS485_Error_t RS485_GetLastError(const RS485_Handle_t *handle)
{
    if (handle == NULL) {
        return RS485_ERR_INVALID_PARAM;
    }
    return handle->last_error;
}

/**
 * @brief  清除错误状态
 */
void RS485_ClearError(RS485_Handle_t *handle)
{
    if (handle == NULL) {
        return;
    }
    
    handle->last_error = RS485_OK;
    handle->retry_count = 0;
    
    // 如果处于错误状态，恢复到空闲状态
    if (handle->state == RS485_STATE_ERROR || 
        handle->state == RS485_STATE_TIMEOUT) {
        handle->state = RS485_STATE_IDLE;
    }
}

/**
 * @brief  强制复位驱动
 */
void RS485_ForceReset(RS485_Handle_t *handle)
{
    if (handle == NULL) {
        return;
    }
    
    UART_HandleTypeDef *huart = (UART_HandleTypeDef *)handle->uart_handle;
    
    // 1. 中止所有UART传输
    HAL_UART_Abort_IT(huart);
    
    // 2. 恢复DE引脚到接收状态
    RS485_DE_LOW(handle);
    
    // 3. 清空缓冲区
    memset(handle->tx_buffer, 0, RS485_TX_BUFFER_SIZE);
    memset(handle->rx_buffer, 0, RS485_RX_BUFFER_SIZE);
    handle->tx_size = 0;
    handle->rx_size = 0;
    
    // 4. 重置状态
    handle->state = RS485_STATE_IDLE;
    handle->retry_count = 0;
    
    // 5. 清除错误（但保留统计信息，便于调试）
    handle->last_error = RS485_OK;
}

/**
 * @brief  获取错误统计信息
 */
void RS485_GetErrorStats(const RS485_Handle_t *handle, 
                        uint16_t *tx_errors,
                        uint16_t *rx_errors,
                        uint16_t *timeouts)
{
    if (handle == NULL) {
        return;
    }
    
    if (tx_errors != NULL) {
        *tx_errors = handle->tx_error_count;
    }
    
    if (rx_errors != NULL) {
        *rx_errors = handle->rx_error_count;
    }
    
    if (timeouts != NULL) {
        *timeouts = handle->timeout_count;
    }
}

/**
 * @brief  获取错误描述字符串
 */
const char* RS485_GetErrorString(RS485_Error_t error)
{
    switch(error) {
        case RS485_OK:
            return "No error";
        case RS485_ERR_BUSY:
            return "Device busy";
        case RS485_ERR_TIMEOUT:
            return "Timeout";
        case RS485_ERR_CRC:
            return "CRC error";
        case RS485_ERR_INVALID_PARAM:
            return "Invalid parameter";
        case RS485_ERR_HW_FAULT:
            return "Hardware fault";
        default:
            return "Unknown error";
    }
}