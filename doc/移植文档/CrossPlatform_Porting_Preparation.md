# 跨平台移植准备指南

> 分析当前代码在跨平台移植中的短板，以及移植前需要做的准备工作。
> 以 STM32F4 → STM32F1（同家族）和 STM32F4 → ESP32/Linux SBC（跨平台）对比为基础。

---

## 一、同平台 vs 跨平台移植对比

| 对比维度 | 同家族（F4→F1） | 跨平台（如 ESP32 / Linux SBC） |
|----------|------------------|-------------------------------|
| HAL 库 | API 同名，只改 include | **整个 HAL 库不存在**，全部重写 |
| CubeMX | 重新生成即可 | **不存在**，没有 ioc 工程 |
| 定时器 API | `HAL_TIM_PWM_Start` 同名 | 完全不同（如 ESP32 `ledc_*`） |
| GPIO API | `HAL_GPIO_WritePin` 同名 | 完全不同（如 ESP32 `gpio_set_level`） |
| I2C API | `HAL_I2C_Mem_Write` 同名 | 完全不同（如 ESP32 `i2c_master_*`） |
| UART API | `HAL_UART_Transmit` 同名 | 完全不同 |
| 中断机制 | NVIC 向量表，同名 | 完全不同（如 ESP32 `esp_intr_alloc`） |
| C++ 支持 | GCC arm-none-eabi | 工具链可能不同 |
| 启动文件 | startup_xxx.s 替换 | 完全不同 |
| 链接脚本 | FLASH.ld 替换 | 完全不同 |
| 引脚编号 | `GPIOB, GPIO_PIN_15` | 需要字符串或数字编号 |
| 工作量 | 基准 | **3~5 倍**于同平台移植 |

---

## 二、当前代码中的隐藏平台依赖

这些位置在当前 STM32 工程中能正常编译，但换到其他平台会直接报错。

### 2.1 `pwm.h:5` — 头文件中直接引用 `tim.h`

```c
#include "tim.h"
```

**问题**：`pwm.h` 标称"与具体芯片无关"，但第一行就引了 CubeMX 生成的 `tim.h`。原因是需要 `TIM_CHANNEL_1/2/3/4` 这几个枚举值给 `PWM_Handle.Channel` 字段用。换到 ESP32 / Linux，这个头文件根本不存在。

**影响**：策略层头文件 `pwm.h` 无法跨平台编译。

### 2.2 `pwm_platform_ops.h:5` — 硬编码 `stm32f4xx_hal.h`

```c
#include "stm32f4xx_hal.h"
```

**问题**：虽然这个文件本身就是平台层，理应引用平台 HAL 头文件，但文件名中的 "platform_ops" 暗示可以通过条件编译处理不同平台。跨平台时这个头文件必须替换成目标平台的 SDK。

### 2.3 `uart_cmd_parser.c` — 硬编码 HAL 类型

```c
#define UART_BASE_HANDLE    huart7       // STM32 特有的全局变量名
#define UART_HAL_TYPE       UART_HandleTypeDef  // STM32 HAL 类型名
```

**问题**：这两个宏驱动了 `handle_to_id()` 的句柄映射逻辑——利用 STM32 HAL 中 huart 句柄在 `.bss` 段连续排列的特性来 O(1) 定位。换平台后这个技巧完全失效。

**影响**：`uart_cmd_parser.c` 需修改（同平台只要改 `huart7` → `huart2`，跨平台则需重写映射逻辑）。

### 2.4 `tool.c` 中的 `handle_to_id()` — 依赖 HAL 内存布局

```c
uint8_t handle_to_id(void *base, uint32_t type_size, void *target) {
    return (uint8_t)(((uint32_t)target - (uint32_t)base) / type_size);
}
```

**问题**：这假设不同 UART 实例的 HAL 句柄在内存中连续排列且等间距，是 STM32 CubeMX 生成代码的产物。其他平台 SDK 不保证这一点。

**影响**：非 STM32 平台上此函数完全不可用。

### 2.5 `encoder_platform_ops.c` — 硬编码 `TIM_HandleTypeDef` 强转

```c
HAL_TIM_Encoder_Start((TIM_HandleTypeDef *)htim, TIM_CHANNEL_ALL);
```

**问题**：平台 ops 文件中强转是合理的，但每个定时器 API（Start/Stop/GetCounter/SetCounter）在跨平台时都必须全部重写。

### 2.6 无硬件 I2C ops 实现

**问题**：当前只有软件 I2C ops（`useri2c_ops.c`），没有硬件 I2C ops 文件。Bluepill 移植时临时走了 `oled_driver.cpp` 和 `main.c` 直调 HAL，绕过了 ops 接口。

**影响**：换到有硬件 I2C 的平台时，又要临时改 `oled_driver.cpp` 和 `imu_bridge.cpp`，无法享受 ops 抽象的零改动红利。

### 2.7 软件 I2C 依赖平台中断

**问题**：`I2C_SoftwareContext.timer_handle` 存了 `void *`（实际是 `TIM_HandleTypeDef*`），`timer_isr` 依赖平台中断机制驱动 FSM 状态机。

**影响**：跨平台时定时器中断注册方式完全不同，需要重新实现。

---

## 三、移植时各层改动预估

| 层次 | 文件 | 同平台(F4→F1) | 跨平台(ESP32等) |
|------|------|:---:|:---:|
| **策略层** | `pwm.c`, `encoder.c`, `uart.c`, `usergpio.c`, `useri2c.c` | 0 改动 | **微调**（修 pwm.h 的 tim.h 泄漏后即 0 改动） |
| **策略层头文件** | `pwm.h`, `encoder.h`, `uart.h`, `usergpio.h`, `useri2c.h` | 0 改动 | **需修 pwm.h 去掉 tim.h** |
| **平台 ops 头文件** | `pwm_platform_ops.h` 等 | 改 1 行 include | **全量重写** |
| **平台 ops 实现** | `pwm_platform_ops.c`, `uart_platform_ops.c`, `encoder_platform_ops.c`, `usergpio_platform.c`, `useri2c_ops.c` | 0~5 行 | **全部重写** |
| **I2C 硬件 ops** | `i2c_hardware_ops.c` | **未写** | **必须写** |
| **实例文件** | `pwm_instance.c` | 改句柄名+通道号 | **同类型重写** |
| **上层初始化** | `main.c` (USER CODE) | 改引脚宏+MX_调用 | **全部重写** |
| **平台配置** | `tim.c`, `gpio.c`, `usart.c`, `i2c.c` | CubeMX 重新生成 | **不存在**，用 SDK 原生 API |
| **中断处理** | `it.c` | USER CODE 调整 | **全部重写** |
| **启动/链接** | `startup.s`, `FLASH.ld` | 直接替换 | **SDK 提供** |
| **构建系统** | `CMakeLists.txt` | 微调（加 C++） | **全部重写** |
| **HAL 库** | `Drivers/` | 全套替换 | **不存在** |
| **命令解析** | `uart_cmd_parser.c` | 改 `huart7`→`huart2` | **需修 `handle_to_id` 逻辑** |
| **功能层** | `motor_*.cpp`, `servo_*.cpp`, `imu_*.cpp`, `oled_*.cpp` | 0 改动 | **0 改动** ✅ |
| **桥接层** | `motor_bridge.*`, `servo_bridge.*`, `imu_bridge.*`, `oled_bridge.*` | 0 改动 | **0 改动** ✅ |
| **通用工具** | `ringbuf.c`, `pid.c`, `tool.c` | 0 改动 | 0 改动 ✅ (`handle_to_id` 移走后) |

---

## 四、移植前需要做的 6 项准备

### 准备 1：修 `pwm.h` — 去掉 `#include "tim.h"` 依赖

**当前问题**：`pwm.h` 声称"与具体芯片无关"，却引用了 CubeMX 生成的 `tim.h`。

**修改方案**：在 `pwm.h` 中定义平台无关的通道枚举类型，替代 `TIM_CHANNEL_x`：

```c
// pwm.h — 替代 #include "tim.h"
typedef enum {
    PWM_CHANNEL_1 = 1,
    PWM_CHANNEL_2 = 2,
    PWM_CHANNEL_3 = 3,
    PWM_CHANNEL_4 = 4,
} PWM_Channel_t;
```

**改动范围**：
- `pwm.h`：删 `#include "tim.h"`，新增 `PWM_Channel_t` 枚举
- `pwm_instance.c`：将 `TIM_CHANNEL_1` 改为 `PWM_CHANNEL_1`
- `pwm_platform_ops.c`：内部映射 `PWM_Channel_t` → `TIM_CHANNEL_x`

**收益**：策略层头文件真正实现零平台依赖。

---

### 准备 2：修 `uart_cmd_parser.c` — 去掉 HAL 类型依赖

**当前问题**：`UART_BASE_HANDLE` 和 `UART_HAL_TYPE` 硬编码宏 + `handle_to_id` 技巧不可跨平台。

**修改方案**：将句柄映射从编译期宏改为运行时注册，增加显式 `id` 参数：

```c
// 旧接口
void uart_cmd_parser_init(UART_Handle *hUART, RingBuffer *ringbuf);

// 新接口
void uart_cmd_parser_init(UART_Handle *hUART, RingBuffer *ringbuf, uint8_t id);
```

同时将 `UART_BASE_HANDLE` / `UART_HAL_TYPE` 宏和 `handle_to_id` 调用从 `uart_cmd_parser.c` 中移除，改用传入的 `id` 参数。

**改动范围**：
- `uart_cmd_parser.c`：去掉两个宏 + `handle_to_id` 调用，用传入的 `id` 替代
- `main.c`：初始化时多传一个 `id` 参数

**收益**：换平台时不需要改 `uart_cmd_parser.c` 内部的任何宏。

---

### 准备 3：实现 `i2c_hardware_ops.c`

**当前问题**：只有软件 I2C ops，无硬件 I2C ops 实现。Bluepill 移植时临时走了 HAL 直调。

**修改方案**：新建 `i2c_hardware_ops.c`，实现 `I2C_OpsTable` 中所有函数指针（`read`、`write`、`mem_read`、`mem_write`），内部调用 HAL I2C 函数。让 `oled_driver.cpp` 和 `imu_bridge.cpp` 统一走 `I2C_Handle` + ops。

**改动范围**：
- 新建 `Core/Src/driver/i2c_hardware_ops.c`
- 新建 `Core/Inc/driver/i2c_hardware_ops.h`
- 改 `oled_driver.cpp`：改用 `I2C_Handle` + ops 接口（替换 HAL 直调）
- 改 `main.c`：IMU 初始化改回 ops 接口

**收益**：换平台时只需要重写 `i2c_hardware_ops.c`，上层代码不动。

---

### 准备 4：将 `handle_to_id` 移出 `tool.c`

**当前问题**：`tool.c` 标称"通用工具"，但 `handle_to_id` 是 STM32 HAL 特有的实现技巧。

**修改方案**：将 `handle_to_id` 从 `tool.c` 移到新的 `stm32_platform_utils.c` 中。

**改动范围**：
- 新建 `Core/Src/driver/stm32_platform_utils.c` / `.h`，移入 `handle_to_id`
- `tool.c` 删除 `handle_to_id`
- 更新 `CMakeLists.txt` 文件列表

**收益**：换到非 STM32 平台时不需要改 `tool.c`。

---

### 准备 5：完善 ops 表"接口契约"注释

**当前问题**：`pwm.h` 的注释很详尽（每个 ops 函数指针都有语义说明），但 `encoder.h`、`uart.h` 等其他驱动就没有这么详细。

**修改方案**：为每个 ops 函数指针补充标准化的契约注释：

```
| 函数指针    | 参数              | 返回值   | 语义                           |
|-------------|-------------------|----------|--------------------------------|
| start       | void *htim        | 无       | 启动编码器计数                  |
| stop        | void *htim        | 无       | 停止计数，CNT 保持当前值        |
| get_counter | void *htim        | int32_t  | 读取当前 CNT 值                |
| set_counter | void *htim, int32_t | 无     | 写入 CNT 值                    |
```

**需要补充的文件**：
- `Core/Inc/driver/encoder.h`
- `Core/Inc/driver/uart.h`
- `Core/Inc/driver/usergpio.h`
- `Core/Inc/driver/useri2c.h`

**收益**：跨平台时可以直接对照契约实现新的平台 ops，不需要回看 STM32 源码理解语义。

---

### 准备 6：创建"移植模板"文件

**当前问题**：跨平台移植时，所有平台 ops 文件都需要重写，但没有参考模板。

**修改方案**：为每个平台 ops 文件生成一个 `.template` 版本，标注每个函数需要实现什么：

```c
// pwm_platform_ops_template.c
#include "pwm.h"
// TODO: 引入目标平台的 PWM API 头文件

static void pwm_xxx_start(void *htim, uint32_t channel) {
    // TODO: 启动 channel 对应的 PWM 输出
    // STM32 参考: HAL_TIM_PWM_Start((TIM_HandleTypeDef *)htim, channel);
}

static void pwm_xxx_stop(void *htim, uint32_t channel) {
    // TODO: 停止 channel 对应的 PWM 输出
}

static void pwm_xxx_set_duty(void *htim, uint32_t channel, uint16_t value) {
    // TODO: 设置 channel 的 CCR 值
    // 注意: value 范围是 0~1000（0E3 千分比），需映射到平台的实际寄存器范围
}

// ... 其余函数同格式
```

**需要的模板文件**：
- `pwm_platform_ops_template.c`
- `uart_platform_ops_template.c`
- `encoder_platform_ops_template.c`
- `usergpio_platform_template.c`
- `i2c_hardware_ops_template.c`
- `i2c_software_ops_template.c`

**存储位置**：`Core/Src/driver/templates/`

**收益**：移植时拿到模板，对着目标平台 SDK 填空即可，效率提升显著。

---

## 五、移植前 AI Agent 需要的输入清单

| 输入 | 说明 | 获取方式 |
|------|------|----------|
| 目标平台 SDK 文档 | PWM/GPIO/UART/I2C/定时器的 API 参考 | 用户提供或 web_search |
| 引脚映射表 | 功能 → 引脚 → 定时器通道的完整对应关系 | **用户提供** |
| 平台特殊约束 | 定时器位宽、通道数限制、中断控制器差异等 | 用户告知或 SDK 文档 |
| 构建工具链信息 | 编译器路径、CMake 工具链文件、SDK 目录结构 | 用户提供或 SDK 文档 |
| 已有移植记录 | `Porting_Detail_Record.md`、`Bluepill_Porting_Pitfalls.md` 等 | 工作区已就绪 ✅ |
| 当前工程代码 | 四层架构完整源码 | 工作区已就绪 ✅ |

**最关键的是引脚映射表和平台特殊约束**——这两个只能由用户提供，其余可以通过搜索和读取 SDK 获取。

---

## 六、准备工作优先级

| 优先级 | 准备项 | 理由 |
|--------|--------|------|
| **P0** | 准备 1：修 `pwm.h` 去 `tim.h` 依赖 | 当前最大的设计漏洞，堵上后策略层头文件真正跨平台 |
| **P0** | 准备 2：修 `uart_cmd_parser.c` 去 HAL 依赖 | 每次移植都要改同一个文件，一劳永逸 |
| **P1** | 准备 3：实现 `i2c_hardware_ops.c` | 补全 ops 体系，避免以后的 HAL 直调散落各处 |
| **P1** | 准备 5：完善接口契约注释 | 移植效率的直接保证 |
| **P2** | 准备 4：移走 `handle_to_id` | 让 `tool.c` 真正通用 |
| **P2** | 准备 6：创建移植模板 | 验收当前 ops 接口的完整性 |

---

## 七、准备工作完成后的移植流程

准备工作全部做完后，跨平台移植的标准流程如下：

1. 用户提供**引脚映射表** + **平台特殊约束**
2. AI Agent 搜索/读取目标平台 SDK 文档
3. 对照移植模板，为每个 `xxx_platform_ops.c` 填写目标平台的实现
4. 创建 `pwm_instance.c`（根据引脚映射表定义实例）
5. 重写 `main.c` 的初始化部分（替代 CubeMX 的 `MX_*_Init`）
6. 修改 `CMakeLists.txt`（工具链 + SDK 依赖）
7. 编译验证 → 功能验证

**预估耗时**：用户从提供引脚映射到完成编译，约 **1 小时**（策略层和功能层零改动）。

---

## 八、相关文档

| 文档 | 说明 |
|------|------|
| `Design_Philosophy.md` | 四层架构设计思想 |
| `Design_Patterns.md` | ops 表 + 依赖注入模式 |
| `Porting_Guide.md` | 同平台移植注意事项 |
| `Bluepill_Porting_Plan.md` | F4→F1 移植执行清单 |
| `Bluepill_Porting_Pitfalls.md` | 移植踩坑记录 |
| `Porting_Detail_Record.md` | 逐文件差异对比 |
