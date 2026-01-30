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
                         uint16_t de_pin)
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
    
    // 初始化状态
    handle->state = RS485_STATE_IDLE;
    handle->timeout_ms = RS485_DEFAULT_TIMEOUT;
    
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
        return RS485_ERR_INVALID_PARAM;
    }
    
    if (tx_len > RS485_TX_BUFFER_SIZE) {
        return RS485_ERR_INVALID_PARAM;
    }
    
    if (rx_expected > RS485_RX_BUFFER_SIZE) {
        return RS485_ERR_INVALID_PARAM;
    }
    
    // 检查设备是否空闲
    if (handle->state != RS485_STATE_IDLE) {
        return RS485_ERR_BUSY;
    }
    
    // 复制数据到内部缓冲区（重要！防止用户数据被销毁）
    memcpy(handle->tx_buffer, tx_data, tx_len);
    handle->tx_size = tx_len;
    handle->rx_expected = rx_expected;
    handle->rx_size = 0;
    handle->timeout_ms = timeout_ms;
    handle->last_tx_time = RS485_GetTick();
    
    // 切换到发送模式
    RS485_DE_HIGH(handle);
    handle->state = RS485_STATE_TX;
    
    // 启动UART中断发送
    UART_HandleTypeDef *huart = (UART_HandleTypeDef *)handle->uart_handle;
    HAL_StatusTypeDef hal_status = HAL_UART_Transmit_IT(huart, handle->tx_buffer, handle->tx_size);
    
    if (hal_status != HAL_OK) {
        // 发送失败，恢复状态
        RS485_DE_LOW(handle);
        handle->state = RS485_STATE_IDLE;
        return RS485_ERR_HW_FAULT;
    }
    
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
    
    // 根据当前状态处理
    switch (handle->state) {
        case RS485_STATE_IDLE:
        case RS485_STATE_TX:
        case RS485_STATE_RX_DONE:
        case RS485_STATE_ERROR:
            // 这些状态不需要处理
            break;
            
        case RS485_STATE_WAIT_RX:
            // 检查是否超时
            if (RS485_IsTimeout(handle->last_tx_time, handle->timeout_ms)) {
                // 超时，中止接收
                UART_HandleTypeDef *huart = (UART_HandleTypeDef *)handle->uart_handle;
                HAL_UART_AbortReceive_IT(huart);
                
                handle->state = RS485_STATE_TIMEOUT;
                
                // 调用错误回调
                if (handle->error_callback != NULL) {
                    handle->error_callback(RS485_ERR_TIMEOUT);
                }
            }
            break;
            
        case RS485_STATE_TIMEOUT:
            // 超时后自动恢复到空闲状态（可选，根据需求调整）
            handle->state = RS485_STATE_IDLE;
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
        // 启动接收
        UART_HandleTypeDef *huart = (UART_HandleTypeDef *)handle->uart_handle;
        HAL_StatusTypeDef hal_status = HAL_UART_Receive_IT(huart, 
                                                            handle->rx_buffer, 
                                                            handle->rx_expected);
        
        if (hal_status == HAL_OK) {
            handle->state = RS485_STATE_WAIT_RX;
            handle->last_tx_time = RS485_GetTick();  // 重置超时计时
        } else {
            // 启动接收失败
            handle->state = RS485_STATE_ERROR;
            if (handle->error_callback != NULL) {
                handle->error_callback(RS485_ERR_HW_FAULT);
            }
        }
    } else {
        // 不需要接收，直接完成
        handle->state = RS485_STATE_IDLE;
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