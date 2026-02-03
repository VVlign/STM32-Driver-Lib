/**
 ******************************************************************************
 * @file    modbus_rtu.c
 * @brief   Modbus RTU协议驱动实现
 ******************************************************************************
 */

/* 头文件包含 ----------------------------------------------------------------*/
#include "modbus_rtu.h"
#include <string.h>

/* 私有宏定义 ----------------------------------------------------------------*/
#define MODBUS_EXCEPTION_FLAG   0x80    /* 异常响应标志位 */

/* 私有函数声明 --------------------------------------------------------------*/
static bool Modbus_CheckSlaveAddr(uint8_t slave_addr);
static Modbus_Error_t Modbus_SendRequest(Modbus_Handle_t *handle,
                                        const uint8_t *frame,
                                        uint16_t frame_len,
                                        uint16_t expect_rx_len);
static Modbus_Error_t Modbus_WaitResponse(Modbus_Handle_t *handle,
                                         uint8_t *rx_data,
                                         uint16_t *rx_len);
static bool Modbus_CheckCRC(const uint8_t *data, uint16_t len);

/* 导出函数实现 --------------------------------------------------------------*/

/**
 * @brief  初始化Modbus主机
 */
Modbus_Error_t Modbus_Init(Modbus_Handle_t *handle,
                          RS485_Handle_t *rs485,
                          uint8_t default_slave)
{
    if (handle == NULL || rs485 == NULL) {
        return MODBUS_ERR_INVALID_PARAM;
    }
    
    // 检查默认地址有效性（0表示无默认地址）
    if (default_slave != 0 && !Modbus_CheckSlaveAddr(default_slave)) {
        return MODBUS_ERR_INVALID_PARAM;
    }
    
    // 清零句柄
    memset(handle, 0, sizeof(Modbus_Handle_t));
    
    // 初始化
    handle->rs485 = rs485;
    handle->default_slave_addr = default_slave;
    handle->default_timeout_ms = MODBUS_DEFAULT_TIMEOUT;
    handle->last_error = MODBUS_OK;
    
    return MODBUS_OK;
}

/**
 * @brief  读保持寄存器（功能码0x03）
 */
Modbus_Error_t Modbus_ReadHoldingRegs(Modbus_Handle_t *handle,
                                     uint8_t slave_addr,
                                     uint16_t reg_addr,
                                     uint16_t reg_count,
                                     uint16_t *data)
{
    if (handle == NULL || data == NULL) {
        return MODBUS_ERR_INVALID_PARAM;
    }
    
    // 检查从机地址
    if (!Modbus_CheckSlaveAddr(slave_addr)) {
        handle->last_error = MODBUS_ERR_INVALID_PARAM;
        return MODBUS_ERR_INVALID_PARAM;
    }
    
    // 检查寄存器数量（Modbus限制：1-125个寄存器）
    if (reg_count == 0 || reg_count > 125) {
        handle->last_error = MODBUS_ERR_INVALID_PARAM;
        return MODBUS_ERR_INVALID_PARAM;
    }
    
    // ========== 构造Modbus请求帧 ========== 
    uint8_t tx_frame[8];
    tx_frame[0] = slave_addr;                      // 从机地址
    tx_frame[1] = MODBUS_FC_READ_HOLDING_REGS;     // 功能码
    tx_frame[2] = (reg_addr >> 8) & 0xFF;          // 起始地址高字节
    tx_frame[3] = reg_addr & 0xFF;                 // 起始地址低字节
    tx_frame[4] = (reg_count >> 8) & 0xFF;         // 数量高字节
    tx_frame[5] = reg_count & 0xFF;                // 数量低字节
    
    // 计算CRC
    uint16_t crc = Modbus_CRC16(tx_frame, 6);
    tx_frame[6] = crc & 0xFF;                      // CRC低字节
    tx_frame[7] = (crc >> 8) & 0xFF;               // CRC高字节
    
    // ========== 发送请求 ========== 
    uint16_t expect_rx_len = 5 + (reg_count * 2);  // 地址+功能码+字节数+数据+CRC
    Modbus_Error_t ret = Modbus_SendRequest(handle, tx_frame, 8, expect_rx_len);
    if (ret != MODBUS_OK) {
        return ret;
    }
    
    // ========== 等待响应 ========== 
    uint8_t rx_frame[MODBUS_MAX_FRAME_SIZE];
    uint16_t rx_len = 0;
    ret = Modbus_WaitResponse(handle, rx_frame, &rx_len);
    if (ret != MODBUS_OK) {
        return ret;
    }
    
    // ========== 解析响应帧 ========== 
    
    // 检查最小长度
    if (rx_len < 5) {
        handle->last_error = MODBUS_ERR_FRAME_ERROR;
        handle->error_count++;
        return MODBUS_ERR_FRAME_ERROR;
    }
    
    // 检查从机地址
    if (rx_frame[0] != slave_addr) {
        handle->last_error = MODBUS_ERR_FRAME_ERROR;
        handle->error_count++;
        return MODBUS_ERR_FRAME_ERROR;
    }
    
    // 检查是否是异常响应
    if (rx_frame[1] & MODBUS_EXCEPTION_FLAG) {
        handle->last_error = MODBUS_ERR_EXCEPTION;
        handle->last_exception_code = rx_frame[2];
        handle->error_count++;
        return MODBUS_ERR_EXCEPTION;
    }
    
    // 检查功能码
    if (rx_frame[1] != MODBUS_FC_READ_HOLDING_REGS) {
        handle->last_error = MODBUS_ERR_FRAME_ERROR;
        handle->error_count++;
        return MODBUS_ERR_FRAME_ERROR;
    }
    
    // 检查字节数
    uint8_t byte_count = rx_frame[2];
    if (byte_count != (reg_count * 2)) {
        handle->last_error = MODBUS_ERR_FRAME_ERROR;
        handle->error_count++;
        return MODBUS_ERR_FRAME_ERROR;
    }
    
    // 检查CRC
    if (!Modbus_CheckCRC(rx_frame, rx_len)) {
        handle->last_error = MODBUS_ERR_CRC;
        handle->error_count++;
        return MODBUS_ERR_CRC;
    }
    
    // 提取数据（大端序转换）
    for (uint16_t i = 0; i < reg_count; i++) {
        data[i] = (rx_frame[3 + i*2] << 8) | rx_frame[4 + i*2];
    }
    
    handle->last_error = MODBUS_OK;
    handle->success_count++;
    return MODBUS_OK;
}

/**
 * @brief  写单个寄存器（功能码0x06）
 */
Modbus_Error_t Modbus_WriteSingleReg(Modbus_Handle_t *handle,
                                    uint8_t slave_addr,
                                    uint16_t reg_addr,
                                    uint16_t value)
{
    if (handle == NULL) {
        return MODBUS_ERR_INVALID_PARAM;
    }
    
    if (!Modbus_CheckSlaveAddr(slave_addr)) {
        handle->last_error = MODBUS_ERR_INVALID_PARAM;
        return MODBUS_ERR_INVALID_PARAM;
    }
    
    // ========== 构造请求帧 ========== 
    uint8_t tx_frame[8];
    tx_frame[0] = slave_addr;
    tx_frame[1] = MODBUS_FC_WRITE_SINGLE_REG;
    tx_frame[2] = (reg_addr >> 8) & 0xFF;
    tx_frame[3] = reg_addr & 0xFF;
    tx_frame[4] = (value >> 8) & 0xFF;
    tx_frame[5] = value & 0xFF;
    
    uint16_t crc = Modbus_CRC16(tx_frame, 6);
    tx_frame[6] = crc & 0xFF;
    tx_frame[7] = (crc >> 8) & 0xFF;
    
    // ========== 发送并等待响应 ========== 
    Modbus_Error_t ret = Modbus_SendRequest(handle, tx_frame, 8, 8);  // 响应长度同请求
    if (ret != MODBUS_OK) {
        return ret;
    }
    
    uint8_t rx_frame[8];
    uint16_t rx_len = 0;
    ret = Modbus_WaitResponse(handle, rx_frame, &rx_len);
    if (ret != MODBUS_OK) {
        return ret;
    }
    
    // ========== 验证响应（写单个寄存器的响应应该与请求完全相同）========== 
    if (rx_len != 8 || memcmp(tx_frame, rx_frame, 6) != 0) {
        handle->last_error = MODBUS_ERR_FRAME_ERROR;
        handle->error_count++;
        return MODBUS_ERR_FRAME_ERROR;
    }
    
    if (!Modbus_CheckCRC(rx_frame, rx_len)) {
        handle->last_error = MODBUS_ERR_CRC;
        handle->error_count++;
        return MODBUS_ERR_CRC;
    }
    
    handle->last_error = MODBUS_OK;
    handle->success_count++;
    return MODBUS_OK;
}

/* 辅助函数实现 --------------------------------------------------------------*/

/**
 * @brief  设置默认从机地址
 */
void Modbus_SetDefaultAddr(Modbus_Handle_t *handle, uint8_t slave_addr)
{
    if (handle != NULL) {
        handle->default_slave_addr = slave_addr;
    }
}

/**
 * @brief  获取默认从机地址
 */
uint8_t Modbus_GetDefaultAddr(const Modbus_Handle_t *handle)
{
    return (handle != NULL) ? handle->default_slave_addr : 0;
}

/**
 * @brief  设置默认超时时间
 */
void Modbus_SetDefaultTimeout(Modbus_Handle_t *handle, uint32_t timeout_ms)
{
    if (handle != NULL) {
        handle->default_timeout_ms = timeout_ms;
    }
}

/**
 * @brief  获取最后错误码
 */
Modbus_Error_t Modbus_GetLastError(const Modbus_Handle_t *handle)
{
    return (handle != NULL) ? handle->last_error : MODBUS_ERR_INVALID_PARAM;
}

/**
 * @brief  获取最后异常码
 */
uint8_t Modbus_GetLastException(const Modbus_Handle_t *handle)
{
    return (handle != NULL) ? handle->last_exception_code : 0;
}

/**
 * @brief  获取统计信息
 */
void Modbus_GetStats(const Modbus_Handle_t *handle, 
                    uint16_t *success, 
                    uint16_t *errors)
{
    if (handle == NULL) return;
    
    if (success != NULL) {
        *success = handle->success_count;
    }
    if (errors != NULL) {
        *errors = handle->error_count;
    }
}

/**
 * @brief  清除统计信息
 */
void Modbus_ClearStats(Modbus_Handle_t *handle)
{
    if (handle != NULL) {
        handle->success_count = 0;
        handle->error_count = 0;
    }
}

/**
 * @brief  获取错误描述字符串
 */
const char* Modbus_GetErrorString(Modbus_Error_t error)
{
    switch(error) {
        case MODBUS_OK:
            return "No error";
        case MODBUS_ERR_TIMEOUT:
            return "Timeout";
        case MODBUS_ERR_CRC:
            return "CRC error";
        case MODBUS_ERR_EXCEPTION:
            return "Modbus exception";
        case MODBUS_ERR_INVALID_PARAM:
            return "Invalid parameter";
        case MODBUS_ERR_FRAME_ERROR:
            return "Frame error";
        case MODBUS_ERR_RS485:
            return "RS485 error";
        default:
            return "Unknown error";
    }
}

/**
 * @brief  计算CRC16（Modbus）
 */
uint16_t Modbus_CRC16(const uint8_t *data, uint16_t len)
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

/* 私有函数实现 --------------------------------------------------------------*/

/**
 * @brief  检查从机地址有效性
 */
static bool Modbus_CheckSlaveAddr(uint8_t slave_addr)
{
    return (slave_addr >= MODBUS_MIN_SLAVE_ADDR && 
            slave_addr <= MODBUS_MAX_SLAVE_ADDR);
}

/**
 * @brief  发送Modbus请求
 */
static Modbus_Error_t Modbus_SendRequest(Modbus_Handle_t *handle,
                                        const uint8_t *frame,
                                        uint16_t frame_len,
                                        uint16_t expect_rx_len)
{
    RS485_Error_t ret = RS485_TransmitReceive(handle->rs485,
                                              frame,
                                              frame_len,
                                              expect_rx_len,
                                              handle->default_timeout_ms);
    
    if (ret != RS485_OK) {
        handle->last_error = MODBUS_ERR_RS485;
        handle->error_count++;
        return MODBUS_ERR_RS485;
    }
    
    return MODBUS_OK;
}

/**
 * @brief  等待Modbus响应
 */
static Modbus_Error_t Modbus_WaitResponse(Modbus_Handle_t *handle,
                                         uint8_t *rx_data,
                                         uint16_t *rx_len)
{
    // 等待RS485接收完成
    uint32_t start_time = HAL_GetTick();
    
    while(1) {
        RS485_Process(handle->rs485);  // 处理RS485状态机
        
        RS485_State_t state = RS485_GetState(handle->rs485);
        
        if (state == RS485_STATE_RX_DONE) {
            // 接收完成，复制数据
            *rx_len = handle->rs485->rx_size;
            memcpy(rx_data, handle->rs485->rx_buffer, *rx_len);
            
            // 恢复RS485状态
            handle->rs485->state = RS485_STATE_IDLE;
            
            return MODBUS_OK;
            
        } else if (state == RS485_STATE_TIMEOUT || state == RS485_STATE_ERROR) {
            // 超时或错误
            handle->rs485->state = RS485_STATE_IDLE;
            handle->last_error = MODBUS_ERR_TIMEOUT;
            handle->error_count++;
            return MODBUS_ERR_TIMEOUT;
        }
        
        // 检查总超时
        if (HAL_GetTick() - start_time > handle->default_timeout_ms + 500) {
            handle->last_error = MODBUS_ERR_TIMEOUT;
            handle->error_count++;
            return MODBUS_ERR_TIMEOUT;
        }
    }
}

/**
 * @brief  检查CRC
 */
static bool Modbus_CheckCRC(const uint8_t *data, uint16_t len)
{
    if (len < 3) {
        return false;
    }
    
    uint16_t crc_received = data[len-2] | (data[len-1] << 8);
    uint16_t crc_calculated = Modbus_CRC16(data, len - 2);
    
    return (crc_received == crc_calculated);
}