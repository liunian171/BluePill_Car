# 移植详细记录 — F427 → F103C8T6 (Bluepill)

> 记录移植过程中所有底层驱动替换、头文件替换、细节修改、上层修改。
> 核心原则：**策略层与功能层零修改，只动平台绑定层。**

---

## 总述

从 STM32F427 移植到 STM32F103C8T6 涉及四类改动：

- **底层驱动替换**：5 个平台 ops 文件的 HAL 引用从 F4 切 F1，适配 F103 定时器差异
- **头文件替换**：全局 `stm32f4xx_hal.h` → `stm32f1xx_hal.h`，CubeMX 生成文件全套替换
- **细节修改**：定时器列表裁剪、残留实例清理、UART 句柄改名、CubeMX 参数调优
- **上层修改**：`main.c` 初始化代码、I2C 方案切换、PWM 实例重写、中断配置

---

## 一、底层驱动替换

### 1.1 平台 ops include 替换

所有平台 ops 文件的 HAL 引用从 F4 切 F1。HAL API 名称在 F1 和 F4 之间完全同名（`HAL_TIM_PWM_Start`、`__HAL_TIM_SET_COMPARE`、`HAL_UART_Transmit`、`HAL_GPIO_WritePin`、`HAL_TIM_Encoder_Start`），无需改动调用代码。

| 文件 | 旧 | 新 | 说明 |
|------|-----|-----|------|
| `pwm_platform_ops.c` | `stm32f4xx_hal.h` | `stm32f1xx_hal.h` | 通过 `#include "tim.h"` 间接引用，HAL API 同名兼容 |
| `encoder_platform_ops.c` | `stm32f4xx_hal.h` | `stm32f1xx_hal.h` | 同上 |
| `uart_platform_ops.c` | `stm32f4xx_hal.h` | `stm32f1xx_hal.h` | 同上 |
| `usergpio_platform.c` | `stm32f4xx_hal.h` | `stm32f1xx_hal.h` | 直接引用，显式修改 |
| `useri2c_ops.h` | `stm32f4xx_hal.h` | `stm32f1xx_hal.h` | 头文件，显式修改 |
| `pwm_platform_ops.h` | `stm32f4xx_hal.h` | `stm32f1xx_hal.h` | 头文件，显式修改 |
| `imu_bridge.cpp` | `stm32f4xx_hal.h` | `stm32f1xx_hal.h` | 源文件，显式修改 |
| `imu_uart_handler.cpp` | `stm32f4xx_hal.h` | `stm32f1xx_hal.h` | 源文件，显式修改 |

> **遗漏检查**：`servo.cpp`、`motor_protocol.cpp` 等文件的注释中残留 `stm32f4xx_hal_xxx` 字符串，不影响编译。可用 `grep -r "stm32f4xx" Core/Src/driver/` 确认所有引用已改。

### 1.2 `pwm_platform_ops.c` — 定时器列表裁剪

F427 有 14 个定时器，F103 只有 TIM1~4。`get_clk_freq` 函数中判断 APB1 定时器的列表需裁剪：

```c
// 旧（F427）— 14 个定时器
if (htim->Instance == TIM2 || htim->Instance == TIM3 ||
    htim->Instance == TIM4 || htim->Instance == TIM5 ||
    htim->Instance == TIM6 || htim->Instance == TIM7 ||
    htim->Instance == TIM12 || htim->Instance == TIM13 ||
    htim->Instance == TIM14)

// 新（F103）— 仅 TIM2/3/4 在 APB1 上
if (htim->Instance == TIM2 || htim->Instance == TIM3 ||
    htim->Instance == TIM4)
```

### 1.3 `pwm_platform_ops.c` — 32bit/16bit 判断

F427 的 TIM2/TIM5 是 32 位定时器，需要区分 CCR 写入宽度（32bit vs 16bit）。F103 所有定时器都是 16 位，不需要区分支。

```c
// 旧（F427）
if (htim->Instance == TIM2 || htim->Instance == TIM5)
    __HAL_TIM_SET_COMPARE(htim, channel, ccr);       // 32bit 写入
else
    __HAL_TIM_SET_COMPARE(htim, channel, (uint16_t)ccr); // 16bit 写入

// 新（F103）
/* F103 所有定时器都是 16 位 */
__HAL_TIM_SET_COMPARE(htim, channel, (uint16_t)ccr);
```

### 1.4 `encoder_platform_ops.c` — 无改动

TIM2 编码器模式在 F1 和 F4 上接口完全一致：

| 参数 | 值 | 兼容性 |
|------|-----|--------|
| `TIM_ENCODERMODE_TI12` | 双路编码 | ✅ 同名 |
| `HAL_TIM_Encoder_Start` | 启动编码器 | ✅ 同名 |
| `HAL_TIM_Encoder_Stop` | 停止编码器 | ✅ 同名 |
| `__HAL_TIM_GET_COUNTER` | 读计数器 | ✅ 同名 |
| `__HAL_TIM_SET_COUNTER` | 设计数器 | ✅ 同名 |
| IC1/IC2 Filter | 15 | ✅ 兼容 |

### 1.5 硬件 I2C 驱动（新增替代方案）

原工程使用软件 I2C（`useri2c_ops.c` 的 GPIO 比特流 + TIM6 中断时基）。F103C8T6 只有 4 个定时器（TIM1~4），全部分配给电机 PWM、编码器1、编码器2、舵机 PWM，没有多余的定时器给软件 I2C 做时基。改为硬件 I2C2。

**`oled_driver.cpp` — I2C 调用切换**

```cpp
// 旧（软件 I2C）
#include "useri2c.h"
extern I2C_Handle g_i2c_dev;
// ... 异步回调模式
i2c_write_reg_async(&g_i2c_dev, kOledAddr, reg,
                    (uint8_t *)data, len, oled_cb);

// 新（硬件 I2C2）
#include "i2c.h"            /* hi2c2 */
HAL_I2C_Mem_Write(&hi2c2, kOledAddr, reg, I2C_MEMADD_SIZE_8BIT,
                  (uint8_t *)data, len, 100);
```

**`main.c` — IMU 初始化句柄切换**

```c
// 旧：通过 I2C_Handle + ops 表
imu_bridge_init(0, IMU_MPU6050, &g_i2c_dev);

// 新：直接传 HAL 句柄
imu_bridge_init(0, IMU_MPU6050, &hi2c2);
```

> **待补项**：应新建 `i2c_hardware_ops.c` 实现硬件 I2C 的 ops 表（`I2C_PlatformOps_t`），内部调 `HAL_I2C_Mem_Write/Read_IT`。上层代码恢复通过 `I2C_Handle + ops` 传入，保持 ops 抽象一致性。当前直接调 HAL 是临时方案。

---

## 二、头文件替换

### 2.1 CubeMX 生成文件全套替换

| 旧（F427） | 新（F103） | 影响 |
|------------|------------|------|
| `Core/Inc/stm32f4xx_hal_conf.h` | `Core/Inc/stm32f1xx_hal_conf.h` | HAL 模块使能宏不同 |
| `Core/Inc/stm32f4xx_it.h` | `Core/Inc/stm32f1xx_it.h` | 中断向量声明 |
| `Core/Src/stm32f4xx_it.c` | `Core/Src/stm32f1xx_it.c` | 中断处理函数 |
| `Core/Src/stm32f4xx_hal_msp.c` | `Core/Src/stm32f1xx_hal_msp.c` | MSP 初始化 |
| `Core/Src/system_stm32f4xx.c` | `Core/Src/system_stm32f1xx.c` | 系统时钟初始化 |
| `Core/Inc/main.h` | 自动引用 `stm32f1xx_hal.h` | ✅ 自动切换 |
| `Core/Inc/tim.h` | `htim5`→`htim4`，函数声明不同 | 定时器句柄 |
| `Core/Inc/usart.h` | `huart7`→`huart2` | 串口句柄 |
| `Core/Inc/gpio.h` | 引脚宏定义不同（`GPIOF_PIN_0`→`GPIOB_PIN_10` 等） | 引脚配置 |
| `startup_stm32f427xx.s` | `startup_stm32f103xb.s` | 启动文件 + 中断向量表 |
| `STM32F427XX_FLASH.ld` | `STM32F103xx_FLASH.ld` | 链接脚本（FLASH/RAM 布局不同） |
| `Drivers/STM32F4xx_HAL_Driver/` | `Drivers/STM32F1xx_HAL_Driver/` | HAL 库全套替换 |
| `Drivers/CMSIS/Device/ST/STM32F4xx/` | `Drivers/CMSIS/Device/ST/STM32F1xx/` | CMSIS 设备文件 |
| `Drivers/CMSIS/Include/core_cm4.h` | `core_cm3.h` | F103 是 Cortex-M3 核 |

### 2.2 用户代码中的 include 替换汇总

| 文件 | 行号 | 旧 | 新 |
|------|------|-----|-----|
| `Core/Src/driver/pwm_platform_ops.c` | 2 | `#include "stm32f4xx_hal.h"` | `"stm32f1xx_hal.h"` |
| `Core/Src/driver/encoder_platform_ops.c` | 7 | `#include "stm32f4xx_hal.h"` | `"stm32f1xx_hal.h"` |
| `Core/Src/driver/uart_platform_ops.c` | - | `#include "stm32f4xx_hal.h"` | `"stm32f1xx_hal.h"` |
| `Core/Src/driver/usergpio_platform.c` | 2 | `#include "stm32f4xx_hal.h"` | `"stm32f1xx_hal.h"` |
| `Core/Inc/driver/pwm_platform_ops.h` | 5 | `#include "stm32f4xx_hal.h"` | `"stm32f1xx_hal.h"` |
| `Core/Inc/driver/useri2c_ops.h` | 24 | `#include "stm32f4xx_hal.h"` | `"stm32f1xx_hal.h"` |
| `Core/Src/driver/imu_bridge.cpp` | 9 | `#include "stm32f4xx_hal.h"` | `"stm32f1xx_hal.h"` |
| `Core/Src/driver/imu_uart_handler.cpp` | 11 | `#include "stm32f4xx_hal.h"` | `"stm32f1xx_hal.h"` |

---

## 三、细节修改

### 3.1 代码文件内部修改

| 文件 | 行号/位置 | 旧 | 新 | 原因 |
|------|-----------|-----|-----|------|
| `uart_cmd_parser.c` | 37 | `#define UART_BASE_HANDLE huart7` | `#define UART_BASE_HANDLE huart2` | F103 只有 USART2 |
| `uart_cmd_parser.c` | 24 | `#include "usart.h" /* 提供 huart7 */` | `/* 提供 huart2 */` | 注释更新 |
| `pwm.c` | 206~218 | `PWM_Handle pwm_tim5_ch4 = { ... }` | **删除** | F103 没有 `htim5` |
| `pwm.h` | 191 | `extern PWM_Handle pwm_tim5_ch4;` | `extern PWM_Handle pwm_tim1_ch1;`<br>`extern PWM_Handle pwm_tim1_ch2;`<br>`extern PWM_Handle pwm_tim4_ch3;` | 平台实例切换为 Bluepill 的 3 个 PWM 通道 |
| `syscalls.c` | 31 | `#include <sys/times.h>` | `/* #include <sys/times.h> */` | 某些工具链下不存在 |

### 3.2 新建文件

| 文件 | 说明 |
|------|------|
| `Core/Src/driver/pwm_instance.c` | Bluepill 的 3 个 `PWM_Handle` 实例定义 |
| `Core/Src/driver/encoder_platform_ops.c` | F427 原工程已有，Bluepill 工程新建（仅 include 不同） |

### 3.3 CubeMX 参数调整

| 定时器 | 参数 | 初始值 | 最终值 | 原因 |
|--------|------|--------|--------|------|
| **TIM1** | Mode | Output Compare | **PWM Generation CH1+CH2** | OC 模式不输出 PWM 波形 |
| **TIM1** | Period | 65535 | **3600** | 电机 20kHz（72MHz/3601≈20k） |
| **TIM1** | AutoReloadPreload | DISABLE | **ENABLE** | 影子寄存器防毛刺 |
| **TIM4** | PSC / Period | 0 / 65535 | **71 / 19999** | 舵机 50Hz（72MHz/72/20000=50） |
| **TIM3** | EncoderMode | TI1 | **TI12** | 双路编码精度更高（4×单路） |
| **TIM3** | IC1Filter / IC2Filter | 0 | **15** | 抗干扰（与 TIM2 一致） |
| **TIM3** | AutoReloadPreload | DISABLE | **ENABLE** | 对齐 TIM2 配置 |
| **USART2** | BaudRate | 9600 | **115200** | 串口命令响应速度 |

### 3.4 中断配置差异

| 中断功能 | F427 实现 | F103 实现 | 处理 |
|----------|-----------|-----------|------|
| 软件 I2C 时基 | `TIM6_DAC_IRQHandler` + `HAL_TIM_Base_Start_IT(&htim6)` | 不需要（改用硬件 I2C2） | 代码删除，CubeMX 中去掉 TIM6 |
| TIM4 中断 | 无 | 不需要（舵机仅 PWM 输出，不开中断） | 不启用 |
| USART 中断 | `UART7_IRQHandler(&huart7)` | `USART2_IRQHandler(&huart2)` | CubeMX 自动生成 |
| I2C 事件中断 | 无（软件 I2C 不需要） | `I2C2_EV_IRQHandler(&hi2c2)` | CubeMX 自动生成 |
| DMA 中断 | 无 | `DMA1_Channel5_IRQHandler(&hdma_i2c2_rx)` | CubeMX 自动生成 |
| TIM1 中断 | 无 | `TIM1_UP_IRQHandler + TIM1_CC_IRQHandler` | CubeMX 自动生成 |

### 3.5 定时器资源分配对比

| 定时器 | F427 用途 | F103 用途 |
|--------|-----------|-----------|
| **TIM1** | Servo PWM（备选） | **电机A/B PWM**（CH1/CH2） |
| **TIM2** | 编码器 | 编码器1 |
| **TIM3** | 无 | **编码器2** |
| **TIM4** | 无 | **舵机 PWM**（CH3） |
| **TIM5** | 电机 PWM | —（不存在） |
| **TIM6** | I2C 时基 | —（不存在） |

---

## 四、上层修改

### 4.1 main.c — include 列表

```c
/* USER CODE BEGIN Includes */
#include "driver/motor_bridge.h"
#include "driver/pwm.h"
#include "driver/usergpio.h"
#include "driver/usergpio_platform.h"
#include "driver/uart.h"
#include "driver/uart_platform_ops.h"
#include "driver/uart_cmd_parser.h"
#include "driver/encoder.h"          // 新增
#include "driver/servo_bridge.h"
#include "driver/imu_bridge.h"
#include "driver/imu_uart_handler.h"
#include "driver/oled_bridge.h"
#include "common/ringbuf.h"
#include <stdio.h>
/* USER CODE END Includes */
```

### 4.2 main.c — MX_ 初始化调用顺序

```c
// 旧（F427）
MX_GPIO_Init();
MX_TIM5_Init();      // 电机 PWM
MX_TIM1_Init();      // Servo PWM
MX_UART7_Init();     // 调试串口
MX_TIM6_Init();      // I2C 时基
MX_TIM2_Init();      // 编码器

// 新（F103）
MX_GPIO_Init();
MX_DMA_Init();       // 新增：I2C2 DMA
MX_TIM1_Init();      // 电机A PWM (CH1) + 电机B PWM (CH2)
MX_TIM2_Init();      // 编码器1
MX_TIM3_Init();      // 编码器2
MX_USART2_UART_Init(); // 调试串口
MX_TIM4_Init();      // 舵机 PWM (CH3)
MX_I2C2_Init();      // 硬件 I2C2
```

### 4.3 main.c — 全局变量

**UART 句柄**

```c
// 旧
UART_Handle uart_debug = {
    .huart = &huart7,
    .ops   = &uart_platform_ops_stm32,
};

// 新
UART_Handle uart_debug = {
    .huart = &huart2,           // USART2
    .ops   = &uart_platform_ops_stm32,
};
```

**电机 GPIO 句柄**

```c
// 旧（F427 — PF1/PF0）
UserGPIO_Handle motor_a_in1 = { GPIOF, 1, &usergpio_platform_ops_stm32 };
UserGPIO_Handle motor_a_in2 = { GPIOF, 0, &usergpio_platform_ops_stm32 };

// 新（F103 — PB12~PB15）
UserGPIO_Handle motor_a_in1 = { GPIOB, GPIO_PIN_13, &usergpio_platform_ops_stm32 };
UserGPIO_Handle motor_a_in2 = { GPIOB, GPIO_PIN_12, &usergpio_platform_ops_stm32 };
UserGPIO_Handle motor_b_in1 = { GPIOB, GPIO_PIN_14, &usergpio_platform_ops_stm32 };
UserGPIO_Handle motor_b_in2 = { GPIOB, GPIO_PIN_15, &usergpio_platform_ops_stm32 };
```

**编码器句柄（新增第 2 路）**

```c
// F427 原来只有 1 路，Bluepill 有 2 路
Encoder_Handle henc1;   /* TIM2, PA0/PA1 */
Encoder_Handle henc2;   /* TIM3, PA6/PA7（新增）*/
```

### 4.4 main.c — 初始化代码（USER CODE BEGIN 2）

```c
/* ---- UART ---- */
ringbuf_init(&g_ringbuf_debug);
uart_cmd_parser_init(&uart_debug, &g_ringbuf_debug);
uart_receive_IT(&uart_debug, &uart_debug.rx_byte);
uart_send(&uart_debug, (uint8_t *)"UART2 Ready!\r\n", 15);

/* ---- 编码器1 + 编码器2 ---- */
henc1.htim = &htim2; henc1.ops = encoder_platform_get_ops();
henc1.ppr = 1320; encoder_start(&henc1);
henc2.htim = &htim3; henc2.ops = encoder_platform_get_ops();
henc2.ppr = 1320; encoder_start(&henc2);

/* ---- 电机A + 电机B ---- */
pwm_set_freq(&pwm_tim1_ch1, 20000); pwm_start(&pwm_tim1_ch1);
pwm_set_freq(&pwm_tim1_ch2, 20000); pwm_start(&pwm_tim1_ch2);
motor_bridge_init(0, &pwm_tim1_ch1, &motor_a_in1, &motor_a_in2, NULL, 200.0f, 33.1f);
motor_bridge_init(1, &pwm_tim1_ch2, &motor_b_in1, &motor_b_in2, NULL, 200.0f, 33.1f);

/* ---- 舵机 ---- */
pwm_set_freq(&pwm_tim4_ch3, 50); pwm_start(&pwm_tim4_ch3);
servo_bridge_init(0, SERVO_PROTOCOL_PWM, &pwm_tim4_ch3);

/* ---- OLED ---- */
oled_bridge_init();
oled_bridge_show_string(0, 0, "BLUEPILL OK");

/* ---- IMU ---- */
imu_bridge_init(0, IMU_MPU6050, &hi2c2);
imu_uart_handler_init(0, 16384.0f, 131.0f);
```

### 4.5 pwm.h + pwm_instance.c — PWM 实例重写

**`pwm.h` extern 声明**

```c
// 旧（F427）
extern PWM_Handle pwm_tim5_ch4;

// 新（Bluepill）
extern PWM_Handle pwm_tim1_ch1;   // 电机A PWM (PA8)
extern PWM_Handle pwm_tim1_ch2;   // 电机B PWM (PA9)
extern PWM_Handle pwm_tim4_ch3;   // 舵机 PWM (PB8)
```

**`pwm_instance.c`（新建）**

```c
#include "pwm.h"
#include "pwm_platform_ops.h"
#include "tim.h"

PWM_Handle pwm_tim1_ch1 = {
    .htim = &htim1, .ops = &pwm_platform_ops_stm32,
    .Channel = TIM_CHANNEL_1, .Ch_State = PWM_Ch_State_OK,
};
PWM_Handle pwm_tim1_ch2 = {
    .htim = &htim1, .ops = &pwm_platform_ops_stm32,
    .Channel = TIM_CHANNEL_2, .Ch_State = PWM_Ch_State_OK,
};
PWM_Handle pwm_tim4_ch3 = {
    .htim = &htim4, .ops = &pwm_platform_ops_stm32,
    .Channel = TIM_CHANNEL_3, .Ch_State = PWM_Ch_State_OK,
};
```

### 4.6 `pwm.c` — 删除旧实例定义

```c
// 旧（pwm.c 末尾）— F427 实例
PWM_Handle pwm_tim5_ch4 = {
    .htim = &htim5, .ops = &pwm_platform_ops_stm32,
    .Channel = TIM_CHANNEL_4, .Ch_State = PWM_Ch_State_OK,
};

// 新 — 删除，实例已移至 pwm_instance.c
```

### 4.7 CMakeLists.txt

```cmake
# 旧
enable_language(C ASM)

# 新 — 添加 C++ 支持
enable_language(C CXX ASM)

# 新增 — 自动扫描驱动源文件
file(GLOB USER_SOURCES
    Core/Src/driver/*.c
    Core/Src/driver/*.cpp
    Core/Src/common/*.c
    Core/Src/common/*.cpp
)
target_sources(${CMAKE_PROJECT_NAME} PRIVATE ${USER_SOURCES})

# 新增 — include 路径
target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
    Core/Inc/driver
    Core/Inc/common
)

# 新 — 链接 C++ 标准库
target_link_libraries(${CMAKE_PROJECT_NAME}
    stm32cubemx
    stdc++
)
```

### 4.8 servo_bridge_init 接口修正

```c
// 旧（F427 main.c 中错误调用）
servo_bridge_init(0, &pwm_tim4_ch3, 500, 2500, 0, 180);

// 新（按 servo_bridge.h 接口签名修正）
servo_bridge_init(0, SERVO_PROTOCOL_PWM, &pwm_tim4_ch3);
```

`servo_bridge.h` 的接口签名是 `(uint8_t id, ServoProtocol_t protocol, void *handle)`，不是带 min/max pulse 的多参数形式。

### 4.9 stm32f1xx_it.c — 中断处理

F427 版本需要在 `stm32f4xx_it.c` 中手动添加：

```c
// F427 需要手动添加的（F103 版本不需要）
#include "useri2c_ops.h"
extern I2C_SoftwareContext g_software_i2c;

void TIM6_DAC_IRQHandler(void) {
    if (__HAL_TIM_GET_FLAG(&htim6, TIM_FLAG_UPDATE)) {
        __HAL_TIM_CLEAR_FLAG(&htim6, TIM_FLAG_UPDATE);
        i2c_software_timer_isr(&g_software_i2c);
    }
    HAL_TIM_IRQHandler(&htim6);
}
```

F103 版本改用硬件 I2C2，CubeMX 自动生成以下中断，无需手动添加：

- `I2C2_EV_IRQHandler(&hi2c2)` — I2C2 事件中断
- `USART2_IRQHandler(&huart2)` — 串口中断
- `DMA1_Channel5_IRQHandler(&hdma_i2c2_rx)` — I2C2 DMA 中断
- `TIM1_UP_IRQHandler(&htim1)` — TIM1 更新中断
- `TIM1_CC_IRQHandler(&htim1)` — TIM1 捕获比较中断

---

## 五、全流程文件改动汇总

| 类别 | 新增 | 修改 | 删除/替换 |
|------|------|------|-----------|
| **平台 ops (include 改 F1)** | 0 | `pwm_platform_ops.c`、`encoder_platform_ops.c`、`uart_platform_ops.c`、`usergpio_platform.c`、`pwm_platform_ops.h`、`useri2c_ops.h`、`imu_bridge.cpp`、`imu_uart_handler.cpp` | 0 |
| **平台 ops (逻辑改)** | 0 | `pwm_platform_ops.c` 定时器列表 + 32bit 判断 | 0 |
| **HAL 库** | 0 | 0 | `Drivers/` 全部替换为 F1 版本 |
| **CubeMX 生成** | 0 | `tim.c`/`gpio.c`/`usart.c`/`i2c.c`/`dma.c`/`main.c` 骨骼 + 引脚宏 | `.ioc` 重新生成 |
| **策略层** | `pwm_instance.c` | `pwm.h` extern 声明 | `pwm.c` 末尾旧实例定义删除 |
| **功能层** | 0 | `oled_driver.cpp` I2C 方案切换 | 0 |
| **上层** | 0 | `main.c` include/PV/初始化代码/while 循环 | 0 |
| **构建** | 0 | `CMakeLists.txt` 加 CXX + stdc++ + include 路径 | 0 |
| **中断** | 0 | `stm32f1xx_it.c` CubeMX 自动生成 | 删除 F427 的手动 TIM6 ISR 代码 |
| **启动/链接** | 0 | 0 | `startup.s`、`FLASH.ld` 替换为 F1 |
| **新建文档** | `Bluepill_Porting_Plan.md`、`Bluepill_Porting_Pitfalls.md`、`Porting_Detail_Record.md`（本文档） | — | — |

---

## 六、通信协议兼容性

两方向通信协议在移植后保持一致：

### PC → STM32（命令帧）

```
0xAA | cmd(1B) | [id(1B)] | [参数...] | 0xFF | 0xFF
```

| 命令码 | 功能 | 参数 |
|--------|------|------|
| `0x01` | 设电机 RPM | id(1B) + rpm(float,4B) |
| `0x02` | 设电机 m/s | id(1B) + mps(float,4B) |
| `0x03` | 电机制动 | id(1B) |
| `0x10` | 设舵机角 | id(1B) + angle(float,4B) |
| `0x30` | 读 IMU 数据 | id(1B) |
| `0x31` | IMU 校准 | id(1B) + samples(uint16,2B) |
| `0xF0` | 心跳 | 无 |

### STM32 → PC（响应帧）

```
0xAA | cmd(1B) | id(1B) | status(1B) | 0xFF | 0xFF
```

| 状态码 | 含义 |
|--------|------|
| `0xA0` | OK |
| `0xA1` | Error |
| `0xA2` | 无效命令 |
| `0xA3` | 无效 ID |
| `0xA4` | 无效参数 |
| `0xA5` | 忙 |

### STM32 → PC（IMU 数据帧 — 特殊）

```
0xAA | 0x30 | id(1B) | roll(4B) | pitch(4B) | yaw(4B) | 0xFF | 0xFF
= 17 字节
```

### STM32 → PC（里程计主动上报 — 待实现）

```
0xAA | ΔX(4B float) | ΔY(4B float) | Δθ(4B float) | timestamp(4B ms)
= 21 字节（无帧尾，上位机按固定长度拆包）
```
