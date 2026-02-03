/**
 ******************************************************************************
 * @file    modbus_rtu.h
 * @brief   Modbus RTU协议驱动（基于RS485）
 * @author  VVlign
 * @date    2025-01-30
 * @version 1.0
 ******************************************************************************
 * @attention
 *
 * 设计特点：
 * 1. 每次调用必须指定从机地址（防止误操作）
 * 2. 支持设置默认地址（可选，方便单从机场景）
 * 3. 基于RS485驱动，支持IT和DMA两种模式
 * 4. 完整的错误处理和CRC校验
 *
 ******************************************************************************
 */

#ifndef MODBUS_RTU_H
#define MODBUS_RTU_H

#ifdef __cplusplus
extern "C" {
#endif

/* 头文件包含 ----------------------------------------------------------------*/
#include <stdint.h>
#include <stdbool.h>
#include "rs485_driver.h"

/* Modbus功能码定义 ---------------------------------------------------------*/
#define MODBUS_FC_READ_COILS              0x01  /* 读线圈 */
#define MODBUS_FC_READ_DISCRETE_INPUTS    0x02  /* 读离散输入 */
#define MODBUS_FC_READ_HOLDING_REGS       0x03  /* 读保持寄存器 */
#define MODBUS_FC_READ_INPUT_REGS         0x04  /* 读输入寄存器 */
#define MODBUS_FC_WRITE_SINGLE_COIL       0x05  /* 写单个线圈 */
#define MODBUS_FC_WRITE_SINGLE_REG        0x06  /* 写单个寄存器 */
#define MODBUS_FC_WRITE_MULTIPLE_COILS    0x0F  /* 写多个线圈 */
#define MODBUS_FC_WRITE_MULTIPLE_REGS     0x10  /* 写多个寄存器 */

/* Modbus异常码定义 ---------------------------------------------------------*/
#define MODBUS_EXCEPTION_ILLEGAL_FUNCTION       0x01  /* 非法功能 */
#define MODBUS_EXCEPTION_ILLEGAL_DATA_ADDRESS   0x02  /* 非法数据地址 */
#define MODBUS_EXCEPTION_ILLEGAL_DATA_VALUE     0x03  /* 非法数据值 */
#define MODBUS_EXCEPTION_SLAVE_DEVICE_FAILURE   0x04  /* 从站设备故障 */
#define MODBUS_EXCEPTION_ACKNOWLEDGE            0x05  /* 确认 */
#define MODBUS_EXCEPTION_SLAVE_DEVICE_BUSY      0x06  /* 从站设备忙 */

/* 配置参数 -----------------------------------------------------------------*/
#define MODBUS_MAX_FRAME_SIZE   256     /* 最大帧长度 */
#define MODBUS_DEFAULT_TIMEOUT  1000    /* 默认超时(ms) */
#define MODBUS_MIN_SLAVE_ADDR   1       /* 最小从机地址 */
#define MODBUS_MAX_SLAVE_ADDR   247     /* 最大从机地址 */

/* 导出类型 -----------------------------------------------------------------*/

/**
 * @brief Modbus错误类型
 */
typedef enum {
    MODBUS_OK = 0,              /* 无错误 */
    MODBUS_ERR_TIMEOUT,         /* 通信超时 */
    MODBUS_ERR_CRC,             /* CRC校验错误 */
    MODBUS_ERR_EXCEPTION,       /* 从机返回异常 */
    MODBUS_ERR_INVALID_PARAM,   /* 无效参数 */
    MODBUS_ERR_FRAME_ERROR,     /* 帧格式错误 */
    MODBUS_ERR_RS485            /* RS485通信错误 */
} Modbus_Error_t;

/**
 * @brief Modbus主机句柄
 */
typedef struct {
    RS485_Handle_t *rs485;              /* RS485驱动句柄 */
    uint8_t default_slave_addr;         /* 默认从机地址（0表示无默认） */
    uint32_t default_timeout_ms;        /* 默认超时时间 */
    
    /* 错误管理 */
    Modbus_Error_t last_error;          /* 最后一次错误 */
    uint8_t last_exception_code;        /* 最后一次异常码 */
    uint16_t success_count;             /* 成功次数统计 */
    uint16_t error_count;               /* 错误次数统计 */
} Modbus_Handle_t;

/* 导出函数 -----------------------------------------------------------------*/

/**
 * @brief  初始化Modbus主机
 * @param  handle: Modbus句柄
 * @param  rs485: RS485驱动句柄
 * @param  default_slave: 默认从机地址（0表示无默认地址，每次必须指定）
 * @retval Modbus_Error_t: 错误码
 */
Modbus_Error_t Modbus_Init(Modbus_Handle_t *handle,
                          RS485_Handle_t *rs485,
                          uint8_t default_slave);

/**
 * @brief  读保持寄存器（功能码0x03）
 * @param  handle: Modbus句柄
 * @param  slave_addr: 从机地址（1-247，必须指定）
 * @param  reg_addr: 寄存器起始地址
 * @param  reg_count: 寄存器数量
 * @param  data: 接收数据缓冲区（uint16_t数组）
 * @retval Modbus_Error_t: 错误码
 */
Modbus_Error_t Modbus_ReadHoldingRegs(Modbus_Handle_t *handle,
                                     uint8_t slave_addr,
                                     uint16_t reg_addr,
                                     uint16_t reg_count,
                                     uint16_t *data);

/**
 * @brief  读输入寄存器（功能码0x04）
 * @param  handle: Modbus句柄
 * @param  slave_addr: 从机地址（1-247）
 * @param  reg_addr: 寄存器起始地址
 * @param  reg_count: 寄存器数量
 * @param  data: 接收数据缓冲区
 * @retval Modbus_Error_t: 错误码
 */
Modbus_Error_t Modbus_ReadInputRegs(Modbus_Handle_t *handle,
                                   uint8_t slave_addr,
                                   uint16_t reg_addr,
                                   uint16_t reg_count,
                                   uint16_t *data);

/**
 * @brief  写单个寄存器（功能码0x06）
 * @param  handle: Modbus句柄
 * @param  slave_addr: 从机地址（1-247）
 * @param  reg_addr: 寄存器地址
 * @param  value: 写入值
 * @retval Modbus_Error_t: 错误码
 */
Modbus_Error_t Modbus_WriteSingleReg(Modbus_Handle_t *handle,
                                    uint8_t slave_addr,
                                    uint16_t reg_addr,
                                    uint16_t value);

/**
 * @brief  写多个寄存器（功能码0x10）
 * @param  handle: Modbus句柄
 * @param  slave_addr: 从机地址（1-247）
 * @param  reg_addr: 寄存器起始地址
 * @param  reg_count: 寄存器数量
 * @param  data: 写入数据
 * @retval Modbus_Error_t: 错误码
 */
Modbus_Error_t Modbus_WriteMultipleRegs(Modbus_Handle_t *handle,
                                       uint8_t slave_addr,
                                       uint16_t reg_addr,
                                       uint16_t reg_count,
                                       const uint16_t *data);

/**
 * @brief  读线圈（功能码0x01）
 * @param  handle: Modbus句柄
 * @param  slave_addr: 从机地址（1-247）
 * @param  coil_addr: 线圈起始地址
 * @param  coil_count: 线圈数量
 * @param  data: 接收数据缓冲区（uint8_t数组，每bit代表一个线圈）
 * @retval Modbus_Error_t: 错误码
 */
Modbus_Error_t Modbus_ReadCoils(Modbus_Handle_t *handle,
                               uint8_t slave_addr,
                               uint16_t coil_addr,
                               uint16_t coil_count,
                               uint8_t *data);

/**
 * @brief  写单个线圈（功能码0x05）
 * @param  handle: Modbus句柄
 * @param  slave_addr: 从机地址（1-247）
 * @param  coil_addr: 线圈地址
 * @param  value: 写入值（0或1）
 * @retval Modbus_Error_t: 错误码
 */
Modbus_Error_t Modbus_WriteSingleCoil(Modbus_Handle_t *handle,
                                     uint8_t slave_addr,
                                     uint16_t coil_addr,
                                     bool value);

/* 辅助函数 -----------------------------------------------------------------*/

/**
 * @brief  设置默认从机地址
 * @param  handle: Modbus句柄
 * @param  slave_addr: 从机地址（0表示取消默认地址）
 * @retval 无
 */
void Modbus_SetDefaultAddr(Modbus_Handle_t *handle, uint8_t slave_addr);

/**
 * @brief  获取默认从机地址
 * @param  handle: Modbus句柄
 * @retval 默认从机地址（0表示无默认地址）
 */
uint8_t Modbus_GetDefaultAddr(const Modbus_Handle_t *handle);

/**
 * @brief  设置默认超时时间
 * @param  handle: Modbus句柄
 * @param  timeout_ms: 超时时间(毫秒)
 * @retval 无
 */
void Modbus_SetDefaultTimeout(Modbus_Handle_t *handle, uint32_t timeout_ms);

/**
 * @brief  获取最后错误码
 * @param  handle: Modbus句柄
 * @retval Modbus_Error_t: 错误码
 */
Modbus_Error_t Modbus_GetLastError(const Modbus_Handle_t *handle);

/**
 * @brief  获取最后异常码
 * @param  handle: Modbus句柄
 * @retval 异常码（0表示无异常）
 */
uint8_t Modbus_GetLastException(const Modbus_Handle_t *handle);

/**
 * @brief  获取通信统计信息
 * @param  handle: Modbus句柄
 * @param  success: 成功次数（输出）
 * @param  errors: 错误次数（输出）
 * @retval 无
 */
void Modbus_GetStats(const Modbus_Handle_t *handle, 
                    uint16_t *success, 
                    uint16_t *errors);

/**
 * @brief  清除统计信息
 * @param  handle: Modbus句柄
 * @retval 无
 */
void Modbus_ClearStats(Modbus_Handle_t *handle);

/**
 * @brief  获取错误描述字符串
 * @param  error: 错误码
 * @retval 错误描述字符串
 */
const char* Modbus_GetErrorString(Modbus_Error_t error);

/**
 * @brief  计算CRC16（Modbus）
 * @param  data: 数据指针
 * @param  len: 数据长度
 * @retval CRC16值
 */
uint16_t Modbus_CRC16(const uint8_t *data, uint16_t len);

#ifdef __cplusplus
}
#endif

#endif /* MODBUS_RTU_H */