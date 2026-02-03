/**
 ******************************************************************************
 * @file    rs485_driver.h
 * @brief   STM32通用RS485通信驱动
 * @author  VVlign
 * @date    2025-01-29
 * @version 1.0
 ******************************************************************************
 * @attention
 *
 * 本驱动提供硬件无关的RS485接口，支持半双工通信，自动DE/RE控制。
 *
 * 主要特性：
 * - 硬件抽象（兼容HAL/LL库）
 * - 状态机管理收发流程
 * - 超时检测
 * - 错误处理
 * - 回调函数支持
 *
 ******************************************************************************
 */

#ifndef RS485_DRIVER_H
#define RS485_DRIVER_H

#ifdef __cplusplus
extern "C" {
#endif

/* 头文件包含 ----------------------------------------------------------------*/
#include <stdint.h>
#include <stdbool.h>

/* 配置参数 ------------------------------------------------------------------*/
#define RS485_MAX_DEVICES       4       /* 最大支持设备数量 */
#define RS485_TX_BUFFER_SIZE    256     /* 发送缓冲区大小 */
#define RS485_RX_BUFFER_SIZE    256     /* 接收缓冲区大小 */
#define RS485_DEFAULT_TIMEOUT   1000    /* 默认超时时间(毫秒) */

/* 导出类型 ------------------------------------------------------------------*/

/**
 * @brief RS485通信状态枚举
 */
typedef enum {
    RS485_STATE_IDLE = 0,       /* 空闲状态 */
    RS485_STATE_TX,             /* 正在发送 */
    RS485_STATE_WAIT_RX,        /* 等待接收 */
    RS485_STATE_RX_DONE,        /* 接收完成 */
    RS485_STATE_TIMEOUT,        /* 通信超时 */
    RS485_STATE_ERROR           /* 通信错误 */
} RS485_State_t;

/**
 * @brief RS485错误代码枚举
 */
typedef enum {
    RS485_OK = 0,               /* 无错误 */
    RS485_ERR_BUSY,             /* 设备忙 */
    RS485_ERR_TIMEOUT,          /* 超时 */
    RS485_ERR_CRC,              /* CRC校验错误 */
    RS485_ERR_INVALID_PARAM,    /* 无效参数 */
    RS485_ERR_HW_FAULT          /* 硬件故障 */
} RS485_Error_t;


/**
 * @brief RS485传输模式
 */
typedef enum {
    RS485_MODE_IT = 0,      /* 中断模式（默认） */
    RS485_MODE_DMA          /* DMA模式 */
} RS485_Mode_t;

/**
 * @brief RS485设备句柄结构体
 */
typedef struct {
    /* 硬件接口（由用户提供） */
    void *uart_handle;                          /* UART句柄（如 &huart1） */
    void *de_gpio_port;                         /* DE引脚端口（如 GPIOA） */
    uint16_t de_gpio_pin;                       /* DE引脚编号（如 GPIO_PIN_12） */
    
    /* 通信缓冲区 */
    uint8_t tx_buffer[RS485_TX_BUFFER_SIZE];    /* 发送缓冲区 */
    uint8_t rx_buffer[RS485_RX_BUFFER_SIZE];    /* 接收缓冲区 */
    uint16_t tx_size;                           /* 发送数据长度 */
    uint16_t rx_size;                           /* 已接收数据长度 */
    uint16_t rx_expected;                       /* 期望接收长度 */
    
    // ========== 新增：传输模式 ========== 
    RS485_Mode_t mode;                      /* 传输模式（IT或DMA） */
    
    /* 状态管理 */
    RS485_State_t state;                        /* 当前状态 */
    uint32_t last_tx_time;                      /* 上次发送时间戳 */
    uint32_t timeout_ms;                        /* 超时时间(毫秒) */
    
    // ========== 新增：错误管理 ========== 
    RS485_Error_t last_error;                   /* 最后一次错误码 */
    uint16_t tx_error_count;                    /* 发送错误次数（累计） */
    uint16_t rx_error_count;                    /* 接收错误次数（累计） */
    uint16_t timeout_count;                     /* 超时次数（累计） */
    uint8_t retry_count;                        /* 当前操作的重试次数 */
    uint8_t max_retries;                        /* 最大重试次数（默认3） */
    
    // ========== 新增：发送阶段专用超时 ========== 
    uint32_t tx_timeout_ms;                     // 发送超时（默认100ms）
    
    /* 回调函数（可选） */
    void (*tx_complete_callback)(void);                         /* 发送完成回调 */
    void (*rx_complete_callback)(uint8_t *data, uint16_t len);  /* 接收完成回调 */
    void (*error_callback)(RS485_Error_t error);                /* 错误回调 */
} RS485_Handle_t;

extern RS485_Handle_t rs485_device;

/* 导出函数 ------------------------------------------------------------------*/

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
                         RS485_Mode_t mode);

/**
 * @brief  发送数据并等待响应
 * @param  handle: RS485句柄指针
 * @param  tx_data: 要发送的数据
 * @param  tx_len: 发送数据长度
 * @param  rx_expected: 期望接收长度（如果不需要接收则填0）
 * @param  timeout_ms: 超时时间(毫秒)
 * @retval RS485_Error_t: 错误代码
 */
RS485_Error_t RS485_TransmitReceive(RS485_Handle_t *handle,
                                    const uint8_t *tx_data,
                                    uint16_t tx_len,
                                    uint16_t rx_expected,
                                    uint32_t timeout_ms);

/**
 * @brief  处理RS485状态机（在主循环或定时器中调用）
 * @param  handle: RS485句柄指针
 * @retval RS485_Error_t: 错误代码
 */
RS485_Error_t RS485_Process(RS485_Handle_t *handle);

/**
 * @brief  获取当前状态
 * @param  handle: RS485句柄指针
 * @retval RS485_State_t: 当前状态
 */
RS485_State_t RS485_GetState(const RS485_Handle_t *handle);

/**
 * @brief  中止当前传输
 * @param  handle: RS485句柄指针
 * @retval 无
 */
void RS485_AbortTransfer(RS485_Handle_t *handle);

/**
 * @brief  UART发送完成回调（在HAL_UART_TxCpltCallback中调用）
 * @param  handle: RS485句柄指针
 * @retval 无
 */
void RS485_TxCpltCallback(RS485_Handle_t *handle);

/**
 * @brief  UART接收完成回调（在HAL_UART_RxCpltCallback中调用）
 * @param  handle: RS485句柄指针
 * @retval 无
 */
void RS485_RxCpltCallback(RS485_Handle_t *handle);

/* 错误处理API ---------------------------------------------------------------*/

/**
 * @brief  获取最后一次错误码
 * @param  handle: RS485句柄指针
 * @retval RS485_Error_t: 错误码
 */
RS485_Error_t RS485_GetLastError(const RS485_Handle_t *handle);

/**
 * @brief  清除错误状态
 * @param  handle: RS485句柄指针
 * @retval 无
 */
void RS485_ClearError(RS485_Handle_t *handle);

/**
 * @brief  强制复位驱动（用于从严重错误中恢复）
 * @param  handle: RS485句柄指针
 * @retval 无
 */
void RS485_ForceReset(RS485_Handle_t *handle);

/**
 * @brief  获取错误统计信息
 * @param  handle: RS485句柄指针
 * @param  tx_errors: 发送错误次数（输出参数）
 * @param  rx_errors: 接收错误次数（输出参数）
 * @param  timeouts: 超时次数（输出参数）
 * @retval 无
 */
void RS485_GetErrorStats(const RS485_Handle_t *handle, 
                        uint16_t *tx_errors,
                        uint16_t *rx_errors,
                        uint16_t *timeouts);

/**
 * @brief  获取错误描述字符串（用于调试）
 * @param  error: 错误码
 * @retval 错误描述字符串
 */
const char* RS485_GetErrorString(RS485_Error_t error);

/**
 * @brief  UART空闲中断回调（用于DMA模式，在HAL_UARTEx_RxEventCallback中调用）
 * @param  handle: RS485句柄指针
 * @param  Size: 实际接收的字节数
 * @retval 无
 */
void RS485_RxEventCallback(RS485_Handle_t *handle, uint16_t Size);

#ifdef __cplusplus
}
#endif

#endif /* RS485_DRIVER_H */