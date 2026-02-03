
Markdown
# STM32 可移植驱动库

一个从零开始构建的STM32驱动库，用于个人学习和项目实战。

## 🎯 项目目标

构建一套可复用、易移植的STM32驱动模块，支持：
- STM32F1系列（F103、F105）
- STM32L4系列（L431）
- STM32G0系列（G030）

## 📁 项目结构

STM32-Driver-Lib/ ├── Drivers/ # 驱动模块 │ ├── RS485/ # RS485通信驱动 ✅ │ ├── Flash/ # Flash存储驱动 (规划中) │ └── UART/ # UART通用驱动 (规划中) ├── Examples/ # 使用示例 │ └── RS485_Example/ # RS485测试例程 ✅ ├── Docs/ # 文档 └── Templates/ # 工程模板

Code

## 🚧 当前进度

**阶段1：基础驱动开发**（进行中）
- [x] 仓库初始化
- [x] RS485驱动开发（中断模式）
- [x] RS485驱动开发（DMA模式）
- [x] RS485测试例程
- [ ] Flash驱动开发
- [ ] UART驱动开发

## 📦 驱动模块

### RS485通信驱动 ✅

**位置**: `Drivers/RS485/`

**功能特性**：
- ✅ 支持中断（IT）和DMA两种模式
- ✅ 自动DE/RE控制
- ✅ 完整状态机管理
- ✅ 超时检测和自动恢复
- ✅ 错误统计
- ✅ 回调函数支持

**详细文档**: [RS485驱动说明](Drivers/RS485/README.md)

**测试例程**: [RS485例程说明](Examples/RS485_Example/README.md)

---

### Flash存储驱动 ⏸️

**状态**: 规划中

**计划功能**：
- 统一的Flash读写接口
- 跨MCU适配（F1/L4/G0）
- 参数存储管理

---

### UART通用驱动 ⏸️

**状态**: 规划中

**计划功能**：
- IT和DMA模式
- 环形缓冲区
- 不定长接收

---

## 🚀 快速开始

### 1. 克隆仓库

```bash
git clone https://github.com/VVlign/STM32-Driver-Lib.git
2. 使用RS485驱动
C
#include "rs485_driver.h"

RS485_Handle_t rs485;

int main(void)
{
    // 初始化RS485（UART1，DE引脚PA12，DMA模式）
    RS485_Init(&rs485, &huart1, GPIOA, GPIO_PIN_12, RS485_MODE_DMA);
    
    while(1) {
        // 发送数据
        uint8_t tx_data[] = {0x01, 0x03, 0x00, 0x00, 0x00, 0x02};
        RS485_TransmitReceive(&rs485, tx_data, 6, 9, 1000);
        
        // 处理状态机
        RS485_Process(&rs485);
        
        // 检查接收
        if(RS485_GetState(&rs485) == RS485_STATE_RX_DONE) {
            // 处理数据
            rs485.state = RS485_STATE_IDLE;
        }
    }
}
详细使用方法请查看 RS485驱动文档。

🎓 设计理念
分层架构：应用层 → 驱动层 → HAL层
硬件抽象：使用 void* 传递硬件句柄，跨MCU移植
完整错误处理：检测 → 记录 → 通知 → 恢复
够用就好：不过度设计，根据需求扩展
📖 文档
RS485驱动说明
RS485测试例程
移植指南（待完善）
🐛 常见问题
问题1：符号重复定义
解决：只在 .c 文件中定义变量，其他地方用 extern 声明。

问题2：状态机卡死
解决：在 RS485_Process() 中添加了超时检测，会自动恢复。

问题3：数据发送不完整
解决：DE引脚拉高后延迟10us再发送。

📝 更新日志
v0.2.0 (2025-01-30)
✅ 新增RS485驱动DMA模式
✅ 完善错误处理机制
✅ 添加发送超时检测
✅ 添加错误统计功能
v0.1.0 (2025-01-29)
✅ 项目初始化
✅ RS485驱动（中断模式）
✅ 基础测试例程
📄 开源协议
MIT License

🙏 致谢
STMicroelectronics HAL库
各大嵌入式技术社区
最后更新: 2025-01-30
作者: @VVlign

如果这个项目对你有帮助，请给个 ⭐ Star！

Code

---

## 📝 **更新 Drivers/RS485/README.md**

**文件路径**: `Drivers/RS485/README.md`

```markdown
# RS485驱动模块

适用于STM32系列单片机的通用RS485通信驱动。

## 功能特性

- ✅ 硬件无关（兼容HAL/LL库）
- ✅ 支持中断（IT）和DMA两种模式
- ✅ 自动DE/RE控制
- ✅ 完整状态机管理（6种状态）
- ✅ 超时检测（发送超时 + 接收超时）
- ✅ 错误处理和自动恢复
- ✅ 错误统计
- ✅ 回调函数支持
- ✅ 多设备支持

## 硬件要求

- STM32的UART外设
- 一个GPIO引脚用于DE控制
- 外部RS485收发器（如MAX485、MAX3485）

## API接口

### 初始化和通信

```c
// 初始化
RS485_Error_t RS485_Init(RS485_Handle_t *handle, 
                         void *uart_handle,
                         void *de_port, 
                         uint16_t de_pin,
                         RS485_Mode_t mode);

// 发送并接收
RS485_Error_t RS485_TransmitReceive(RS485_Handle_t *handle,
                                    const uint8_t *tx_data,
                                    uint16_t tx_len,
                                    uint16_t rx_expected,
                                    uint32_t timeout_ms);

// 状态机处理（主循环调用）
RS485_Error_t RS485_Process(RS485_Handle_t *handle);
状态和错误
C
// 获取状态
RS485_State_t RS485_GetState(const RS485_Handle_t *handle);

// 获取错误
RS485_Error_t RS485_GetLastError(const RS485_Handle_t *handle);

// 清除错误
void RS485_ClearError(RS485_Handle_t *handle);

// 强制复位
void RS485_ForceReset(RS485_Handle_t *handle);

// 错误统计
void RS485_GetErrorStats(const RS485_Handle_t *handle, 
                        uint16_t *tx_errors,
                        uint16_t *rx_errors,
                        uint16_t *timeouts);
回调函数
C
// 发送完成回调（在HAL_UART_TxCpltCallback中调用）
void RS485_TxCpltCallback(RS485_Handle_t *handle);

// 接收完成回调（在HAL_UART_RxCpltCallback中调用）
void RS485_RxCpltCallback(RS485_Handle_t *handle);

// DMA接收事件回调（在HAL_UARTEx_RxEventCallback中调用）
void RS485_RxEventCallback(RS485_Handle_t *handle, uint16_t Size);
使用方法
1. 初始化
C
#include "rs485_driver.h"

RS485_Handle_t rs485;

// 使用UART1和PA12作为DE引脚，DMA模式
RS485_Init(&rs485, &huart1, GPIOA, GPIO_PIN_12, RS485_MODE_DMA);

// 或者使用中断模式
// RS485_Init(&rs485, &huart1, GPIOA, GPIO_PIN_12, RS485_MODE_IT);
2. 发送和接收
C
uint8_t tx_data[] = {0x01, 0x03, 0x00, 0x00, 0x00, 0x01};
RS485_TransmitReceive(&rs485, tx_data, 6, 7, 1000);  // 期望接收7字节，超时1秒
3. 在主循环中处理
C
while(1) {
    RS485_Process(&rs485);  // 周期性调用，处理超时检测
    
    if(RS485_GetState(&rs485) == RS485_STATE_RX_DONE) {
        // 处理接收到的数据
        uint8_t *rx_data = rs485.rx_buffer;
        uint16_t rx_len = rs485.rx_size;
        
        // 处理完成后，恢复到空闲状态
        rs485.state = RS485_STATE_IDLE;
    }
}
4. 链接中断回调
在 stm32xxxx_it.c 中：

C
extern RS485_Handle_t rs485;

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    if(huart->Instance == USART1) {
        RS485_TxCpltCallback(&rs485);
    }
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if(huart->Instance == USART1) {
        RS485_RxCpltCallback(&rs485);
    }
}

// DMA模式需要
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
    if(huart->Instance == USART1) {
        RS485_RxEventCallback(&rs485, Size);
    }
}
状态说明
状态	说明
RS485_STATE_IDLE	空闲状态，可以发送新请求
RS485_STATE_TX	正在发送
RS485_STATE_WAIT_RX	等待接收响应
RS485_STATE_RX_DONE	接收完成
RS485_STATE_TIMEOUT	通信超时（会自动恢复到IDLE）
RS485_STATE_ERROR	严重错误（需要手动恢复）
错误码说明
错误码	说明
RS485_OK	无错误
RS485_ERR_BUSY	设备忙
RS485_ERR_TIMEOUT	超时
RS485_ERR_CRC	CRC校验错误（预留）
RS485_ERR_INVALID_PARAM	无效参数
RS485_ERR_HW_FAULT	硬件故障
传输模式对比
特性	中断模式(IT)	DMA模式
CPU占用	高（每字节中断一次）	低（只有完成时中断）
适用场景	短帧、低速率	长帧、高速率
接收方式	定长接收	不定长（空闲检测）
配置复杂度	简单	需要配置DMA
移植指南
1. 包含对应的HAL头文件
在 rs485_driver.c 中修改：

C
#if defined(STM32F103xB) || defined(STM32F103xE)
    #include "stm32f1xx_hal.h"
#elif defined(STM32L431xx)
    #include "stm32l4xx_hal.h"
#elif defined(STM32G030xx)
    #include "stm32g0xx_hal.h"
#endif
2. 配置UART
使用STM32CubeMX生成UART初始化代码，注意：

配置波特率（如9600）
使能UART中断
如果用DMA模式，配置DMA
3. 配置DE引脚
配置一个GPIO输出引脚用于控制RS485的DE/RE。

注意事项
DE引脚在发送时为高电平，接收时为低电平
驱动使用中断方式，需要正确配置UART中断
超时检测在 RS485_Process() 中进行，需要周期性调用
接收完成后，需要手动将状态恢复到 IDLE
DMA模式需要较新的HAL库版本（支持 HAL_UARTEx_ReceiveToIdle_DMA）
示例
完整的测试例程请参考 Examples/RS485_Example/。

版本历史
v1.0 (2025-01-30): 正式版本

支持IT和DMA两种模式
完整的错误处理
超时自动恢复
v0.1 (2025-01-29): 初始版本

基础通信功能
中断模式
Code

---

## 📝 **更新 Examples/RS485_Example/README.md**

**文件路径**: `Examples/RS485_Example/README.md`

```markdown
# RS485驱动测试例程

本例程演示如何使用RS485驱动实现Modbus RTU通信。

## 功能说明

- ✅ 每2秒发送一次Modbus读保持寄存器命令
- ✅ 自动接收从机响应
- ✅ 超时检测和错误处理
- ✅ LED指示通信状态

## 硬件连接

### 最小系统

| 功能 | 引脚 | 说明 |
|------|------|------|
| UART TX | PA9 | 连接到RS485模块的DI |
| UART RX | PA10 | 连接到RS485模块的RO |
| RS485 DE | PA12 | 连接到RS485模块的DE和RE（短接） |
| LED | PC13 | 状态指示灯 |

### RS485模块接线

STM32 MAX485 PA9 -------> DI (数据输入) PA10 <------- RO (数据输出) PA12 -------> DE (发送使能) PA12 -------> RE (接收使能，与DE短接) 3.3V -------> VCC GND -------> GND

Code

## 使用方法

### 1. 导入工程

- 复制 `main.c` 和 `stm32xxxx_it.c` 到你的工程
- 添加 `rs485_driver.c` 和 `rs485_driver.h` 到工程

### 2. 配置参数

根据你的硬件修改以下参数：

```c
// main.c中
#define LED_PIN         GPIO_PIN_13     // LED引脚
#define LED_PORT        GPIOC           // LED端口

// RS485_Init()中
RS485_Init(&rs485_device, 
           &huart1,              // 使用的UART
           GPIOA,                // DE引脚端口
           GPIO_PIN_12);         // DE引脚编号
3. 编译下载
使用Keil或其他IDE编译
下载到板子
观察LED闪烁
测试方法
方法1：使用Modbus从机设备
准备一个Modbus从机设备（地址设为0x01）
连接RS485总线（A对A，B对B）
观察LED闪烁和串口输出
方法2：使用USB转RS485模块
准备USB转RS485模块
电脑上运行Modbus模拟软件（如Modbus Slave）
配置从机地址为0x01
添加2个保持寄存器（地址0x0000-0x0001）
连接RS485总线
观察通信数据
方法3：双板测试
使用两块STM32板子：

板子A运行本例程（作为主机）
板子B运行Modbus从机程序
交叉连接A/B线
运行效果
正常通信
LED每2秒闪烁一次（发送）
接收成功后再闪烁一次（接收）
通信超时
LED只闪烁一次（发送）
1秒后超时
继续下一次发送
调试技巧
使能调试输出
取消注释回调函数中的 printf 语句：

C
static void RS485_RxComplete_Callback(uint8_t *data, uint16_t len)
{
    printf("RX: ");  // 取消注释
    for(uint16_t i = 0; i < len; i++) {
        printf("%02X ", data[i]);
    }
    printf("\r\n");
}
Modbus帧格式
发送帧示例（读取从机0x01的寄存器0x0000，数量2个）：

Code
01 03 00 00 00 02 C4 0B
响应帧示例：

Code
01 03 04 12 34 56 78 XX XX
└─ 地址
   └─ 功能码
      └─ 字节数
         └─ 数据
                  └─ CRC
常见问题
Q1: LED不闪烁
检查GPIO初始化是否正确
检查引脚定义是否与��际硬件一致
Q2: 一直超时
检查RS485模块接线
检查从机地址是否正确
用示波器看DE引脚是否有电平变化
Q3: 接收到乱码
检查波特率设置（主从机需一致）
检查RS485的A/B线是否接反
代码说明
主要函数
C
// Modbus读保持寄存器
Modbus_ReadHoldingRegs(uint8_t slave, uint16_t addr, uint16_t count);

// CRC16计算
CRC16_Modbus(uint8_t *data, uint16_t len);

// RS485测试任务
RS485_Test_Task();
下一步
学会这个例程后，你可以：

修改Modbus命令（写寄存器、读线圈等）
添加CRC校验
实现完整的Modbus协议栈
集成到你的实际项目中
版本历史
v1.0 (2025-01-30): 初始版本
Code

---

## ✅ **你的任务**

### 1. 替换3个文档

- `README.md`（仓库根目录）
- `Drivers/RS485/README.md`
- `Examples/RS485_Example/README.md`

### 2. 提交到GitHub

```bash
git add .
git commit -m "docs: 更新所有README文档为精简清晰版

- 更新项目总览文档
- 更新RS485驱动说明文档
- 更新RS485例程说明文档
- 统一使用中文
- 简化表述，突出重点"
git push origin main