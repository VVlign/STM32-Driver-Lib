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
           GPIO_PIN_12,          // DE引脚编号
           RS485_MODE_DMA);      // 传输模式
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
└─ 地址
   └─ 功能码
      └─ 寄存器地址
            └─ 数量
                  └─ CRC
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
可能原因：

GPIO初始化不正确
引脚定义与实际硬件不一致
解决方法：

检查LED引脚配置
用万用表测试引脚电平
Q2: 一直超时
可能原因：

RS485模块接线错误
从机地址不正确
DE引脚未控制
解决方法：

检查A/B线是否接反
检查从机地址设置
用示波器看DE引脚是否有电平变化
Q3: 接收到乱码
可能原因：

波特率不一致
RS485总线干扰
解决方法：

检查主从机波特率设置
检查RS485线缆长度和终端电阻
代码说明
主要函数
C
// Modbus读保持寄存器
void Modbus_ReadHoldingRegs(uint8_t slave_addr, uint16_t reg_addr, uint16_t reg_count);

// CRC16计算
uint16_t CRC16_Modbus(uint8_t *data, uint16_t len);

// RS485测试任务
void RS485_Test_Task(void);
工作流程
Code
1. 初始化RS485驱动
2. 每2秒执行一次测试任务
3. 构造Modbus请求帧
4. 发送请求并等待响应
5. 处理接收到的数据
6. 显示通信状态
下一步
学会这个例程后，你可以：

修改Modbus命令（写寄存器、读线圈等）
添加多个从机轮询
实现完整的Modbus协议栈
集成到你的实际项目中
版本历史
v1.0 (2025-01-30)
✅ 初始版本
✅ 支持Modbus读保持寄存器
✅ 完整的错误处理
✅ LED状态指示


