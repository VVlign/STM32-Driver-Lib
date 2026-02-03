# RS485驱动测试例程

## 功能说明

本例程演示如何使用RS485驱动实现Modbus RTU通信。

### 实现功能

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




## 使用方法

### 1. 导入工程

- 复制`main.c`和`stm32xxxx_it.c`到你的工程
- 添加`rs485_driver.c`和`rs485_driver.h`到工程

### 2. 配置参数

根据硬件修改以下参数：

```c
// main.c中
#define LED_PIN         GPIO_PIN_13     // LED引脚
#define LED_PORT        GPIOC           // LED端口

// RS485_Init()中
RS485_Init(&rs485_device, 
           &huart1,              // 使用的UART
           GPIOA,                // DE引脚端口
           GPIO_PIN_12);         // DE引脚编号