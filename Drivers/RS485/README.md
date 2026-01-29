# RS485驱动模块

## 概述

适用于STM32系列单片机的通用RS485通信驱动。

## 功能特性

- ✅ 硬件无关（兼容HAL/LL库）
- ✅ 自动DE/RE控制
- ✅ 状态机管理
- ✅ 超时检测
- ✅ 回调函数支持
- ✅ 多设备支持

## 硬件要求

- STM32的UART外设
- 一个GPIO引脚用于DE控制
- 外部RS485收发器（如MAX485、MAX3485）

## 使用方法

### 1. 初始化

```c
#include "rs485_driver.h"

RS485_Handle_t rs485;

// 使用UART1和PA12作为DE引脚初始化
RS485_Init(&rs485, &huart1, GPIOA, GPIO_PIN_12);