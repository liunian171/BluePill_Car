# 移植注意事项

> 将驱动代码从 STM32F427 移植到其他平台（STM32F1/GD32/AT32 等）时需要注意的所有事项。
> 以 F427→F103C8T6（Bluepill）移植经验为基础总结。

---

## 一、需要修改的文件清单

### 必须改（平台绑定层）

| 文件 | 修改内容 | 修改量 |
|------|---------|--------|
| `Core/Src/driver/pwm_platform_ops.c` | HAL include + 定时器列表 | ~5 行 |
| `Core/Src/driver/encoder_platform_ops.c` | HAL include | ~1 行 |
| `Core/Src/driver/uart_platform_ops.c` | HAL include | ~1 行 |
| `Core/Src/driver/usergpio_platform.c` | HAL include | ~1 行 |
| `Core/Inc/driver/pwm_platform_ops.h` | HAL include | ~1 行 |
| `Core/Inc/driver/useri2c_ops.h` | HAL include | ~1 行 |
| `Core/Src/driver/imu_bridge.cpp` | HAL include | ~1 行 |
| `Core/Src/driver/imu_uart_handler.cpp` | HAL include | ~1 行 |
| `Core/Src/driver/uart_cmd_parser.c` | `UART_BASE_HANDLE` | ~1 行 |

### 根据引脚分配改

| 文件 | 修改内容 |
|------|---------|
| `Core/Src/driver/pwm_instance.c` | 全部重写（PWM 实例定义） |
| `Core/Inc/driver/pwm.h` | `extern` 声明 |
| `Core/Src/main.c` | USER CODE 段：UART 句柄、GPIO 句柄、I2C 上下文、MX_ 调用顺序 |
| `Core/Src/stm32f1xx_it.c` | 中断处理函数（如有新增中断需求） |

### 根据 I2C 方案改

| 方案 | 修改内容 |
|------|---------|
| 软件 I2C | 保留 `useri2c.c/h`、`useri2c_ops.c/h`，配一个基本定时器做时基 |
| 硬件 I2C | 删软件 I2C，改 `oled_driver.cpp` 和 IMU 初始化直接调 HAL |
| 硬件 I2C + ops 表（推荐） | 新建 `i2c_hardware_ops.c`，上层代码不动 |

---

## 二、常见陷阱

### 2.1 定时器差异

| 问题 | F427 | F103 | 影响 |
|------|------|------|------|
| 定时器位宽 | TIM2/TIM5 32bit，其余 16bit | **全部 16bit** | `pwm_platform_ops.c` 中 `set_ccr` 的 32/16bit 分支 |
| TIM5 是否存在 | 有 | **无** | APB1 定时器列表要去掉 |
| TIM6/TIM7 是否存在 | 有 | **无** | 同上 |
| 定时器数量 | 14 个 | 4 个 | 功能复用冲突（见 2.2） |
| TIM1 互补通道 | CH1N/CH2N/CH3N 全有 | 有 | CubeMX 中别误配成 OC 模式 |

### 2.2 定时器冲突解决

当定时器不够分时，优先方案：

1. **编码器 + PWM 不能共用同一个定时器**（共用 CNT 冲突）
2. **硬件 I2C 不需要定时器时基**，省下一个定时器
3. 如果仍不够，考虑飞线或换引脚

F103C8T6 的典型分配（以 Bluepill 移植为例）：

| 定时器 | 用途 |
|--------|------|
| TIM1 | 电机A/B PWM（CH1/CH2） |
| TIM2 | 编码器1 |
| TIM3 | 编码器2 |
| TIM4 | 舵机 PWM（CH3） |

### 2.3 HAL include 遗漏

最容易遗漏的文件（按出现频率排序）：
1. `pwm_platform_ops.h` — 头文件中的 include
2. `useri2c_ops.h` — 头文件中的 include
3. `usergpio_platform.c` — 源文件中的 include
4. `imu_bridge.cpp` — 源文件中的 include
5. `imu_uart_handler.cpp` — 源文件中的 include

**解决方法：** 全局搜索 `stm32f4xx`，将所有引用改为 `stm32f1xx`（或其他目标平台）。

### 2.4 残留的旧平台实例

`pwm.c` 末尾可能残留旧平台的 PWM 实例定义（如 `pwm_tim5_ch4`），新平台没有对应的定时器句柄（`htim5`），编译报错。

**解决方法：** 删除 `pwm.c` 末尾的实例定义，新建 `pwm_instance.c` 存放新平台的实例。

### 2.5 F427 特有的外设引用

`pwm_platform_ops.c` 中的 `get_clk_freq` 函数可能引用了 F427 特有的定时器（TIM5/6/7/12/13/14），新平台编译时报 `undeclared`。

**解决方法：** 改为目标平台实际有的定时器列表。

### 2.6 UART 句柄名称

`uart_cmd_parser.c` 中 `UART_BASE_HANDLE` 定义为 F427 的 `huart7`，新平台可能是 `huart2` 或 `huart1`。

**解决方法：** 根据目标平台的 UART 外设名称修改。

### 2.7 Servo bridge 接口签名

`servo_bridge_init` 的接口是 `(id, protocol_type, handle)`，不是 `(id, pwm_handle, min, max, ...)`。

**解决方法：** 检查 main.c 中的调用方式。

---

## 三、工具链注意事项

### 3.1 必须使用 ARM 交叉编译器

- 目标平台：`arm-none-eabi-gcc` / `arm-none-eabi-g++`
- 不能使用 `mingw-gcc`（x86_64 架构无法解析 ARM 汇编）

### 3.2 CMake 配置

必须通过 preset 或手动指定工具链文件：

```bash
cmake -S . -B build/Debug -G Ninja \
  -DCMAKE_BUILD_TYPE=Debug \
  -DCMAKE_TOOLCHAIN_FILE=cmake/gcc-arm-none-eabi.cmake
```

### 3.3 C++ 支持

如果原 CMakeLists.txt 只有 `enable_language(C ASM)`，需要添加：

```cmake
enable_language(C CXX ASM)
target_link_libraries(${CMAKE_PROJECT_NAME} stm32cubemx stdc++)
```

### 3.4 syscalls.c

CubeMX 生成的 `syscalls.c` 中 `#include <sys/times.h>` 在某些工具链下可能不存在，编译报错。

**解决方法：** 注释掉该行。CubeMX 重新生成后需要重新注释。

---

## 四、验证清单

### 编译验证
- [ ] 所有 `stm32f4xx` → `stm32f1xx`（或目标平台）替换完成
- [ ] 所有 F427 特有的定时器引用已去除
- [ ] C++ 编译正常（`arm-none-eabi-g++`）
- [ ] 链接通过，无未定义符号

### 功能验证
- [ ] UART 打印正常
- [ ] CMD_PING 响应
- [ ] 编码器读数正确
- [ ] 电机正反转控制
- [ ] 舵机角度控制
- [ ] IMU 欧拉角数据
- [ ] I2C 通信正常（逻辑分析仪确认时序）
- [ ] OLED 显示正常

---

## 五、文档更新

移植完成后建议更新以下文档：

| 文档 | 更新内容 |
|------|---------|
| `motor_arch_design.md` | 编码器/PID 状态改✅，新增文件清单 |
| `Implementation_Backlog.md` | 已完成项改✅，新增移植相关待办 |
| `Document_Index.md` | 更新状态 |
| 新建 `xxx_Porting_Plan.md` | 记录移植方案和引脚分配 |
| 新建 `xxx_Porting_Pitfalls.md` | 记录遇到的问题和解决方法 |
