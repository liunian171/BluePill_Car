# 设计文档索引与状态

> **本文档分两部分，注意区分（2026-09-11 修订）**：
> - **§0 本仓库文档索引** —— BluePill_Car 自身文档，实体位于 `D:\666WorkSpace\STM32Cube\BluePill_Car\doc\`
> - **§一~§四 上游驱动工程索引快照** —— 原驱动框架工程 `A_board_pwm_driver_test`（RoboMaster A 板，STM32F427）的设计文档索引；**这些文档实体在上游工程目录，不在本仓库**，此处仅作追溯参考
>
> 历史沿革：本文件随移植从上游工程复制而来，早期版本曾把上游文档当作本仓库文档索引，易误导。

---

## 〇、本仓库文档索引（BluePill_Car / STM32F103C8T6）

| 文档 | 定位 | 与代码一致性 |
|------|------|-------------|
| `AGENTS.md`（工作区根） | 综合引导：项目定义/硬件方案/架构现状/开发规范 | ✅ 2026-09-11 按双栈模型修订分层命名 |
| `doc/工程文档.md` | 架构 / 引脚 / 协议 / 初始化流程权威参考 | ✅ |
| `doc/开发跟踪.md` | 阶段与条目状态跟踪（W 无线 / SS 舵机 / L 巡线 / 架构治理） | ✅ 2026-09-11 补齐 09-10~11 日志 |
| `doc/巡线逻辑设计文档.md` | 巡线状态机唯一权威（决策表 + 状态机 + 参数） | ✅ 与 `line_follower.c` 四状态机实现一致 |
| `doc/舵机转向设计文档.md` | 舵机阿克曼转向命令域 / 标定参数权威 | ✅ 输出域绝对角（直行 -90）已固化 |
| `doc/调试总结.md` | 调试时间线 + 技术要点 + 电机参数 + 命令速查 | ✅ 2026-09-11 回写 §11/§12/§13 证据行与资源占用 |
| `doc/架构评审文档.md` | 耦合性/可配置性评审快照 + 治理优先级（C1-C8 / D1-D3） | ✅ 2026-09-19 已回写治理状态（C1/C3/C5/C6/C7/C8/D1/D2 ✅，C2/C4/D3 开放） |
| `doc/代码风格与模块衔接指南.md` | 代码形态规范：分层性质/命名注释/机制规则/新模块衔接义务 8 项 | ✅ 2026-09-19 现状列已同步代码 |
| `doc/结构优化顺序分析.md` | 治理顺序推演（契约先行 / 实现后搬）+ 学习期进度登记 | ✅ 2026-09-19 已回写（C1/C2/C3 均落地） |
| `doc/移植文档/Document_Index.md` | 本文件（本仓库索引 + 上游快照） | — |
| `doc/移植文档/CrossPlatform_Porting_Preparation.md` | 跨平台移植短板与隐藏平台依赖清单 | ✅ |
| `doc/移植文档/Porting_Guide.md` / `Porting_Detail_Record.md` | 移植指南 / 详细记录 | ✅ |
| `backup/` | 历史版本备份，只读不维护 | — |

### 本仓库代码-文档一致性已知缺口

| 项 | 说明 | 状态 |
|----|------|------|
| D1 | `txt_cmd.h` / `txt_cmd.c` 舵机命令域注释曾写"居中域"，与实现的"输出域绝对角"矛盾 | ✅ 2026-09-11 已修正 |
| D2 | `line_follower.h` 头注释为旧四阶段策略（STRAIGHT/编码器驱动），与实现不符 | ✅ 2026-09-11 已修正 |
| D3 | `pwm.h` 预留未用结构（include tim.h 部分已修） | ⬜ 预留结构归拢说明未加（tim.h include ✅ 已修 2026-09-11） |
| C1 | ~~`uart_cmd_parser.c` 死代码仍在编译~~ | ✅ 2026-09-13 已删除（639 行批次） |
| C2 | `line_follower.c` 直调 `HAL_GPIO_ReadPin`，PC 桩不可编译 | ⬜ 仍开放（随巡线感知拆分处理） |

---

## 上游驱动工程索引快照（A_board_pwm_driver_test）

> 以下为原驱动框架工程的设计文档索引，文档实体位于上游工程目录。原文件头注"位于 `A_board_pwm_driver_test/` 根目录"即指此。

## 一、架构设计

### 1. Design_Philosophy.md — 驱动框架设计思想
- **状态**：✅ 已完成，与实际代码一致
- **内容**：四层架构（应用层→桥接层→功能层→协议层→驱动层），依赖注入，C/C++ 桥接，0E3 千分比约定
- **适用范围**：PWM、Servo、Motor 及所有遵循此模式的驱动模块

### 2. Design_Patterns.md — 嵌入式 C 通用设计模式
- **状态**：✅ 已完成
- **内容**：结构体指针 + ops 表 + 对象池"三件套"模式，跨平台抽象层设计

### 3. C_CPP_Bridge_Mechanism.md — C/C++ 桥接机制
- **状态**：✅ 已完成，与实际代码一致
- **内容**：`extern "C"` 桥接、`new` 对象 + 虚函数派发、全局指针数组管理

---

## 二、驱动模块设计

### 4. PWM_Driver_Design.md — PWM 驱动
- **状态**：✅ 已完成
- **注意**：设计文档中 `pwm_tim5_ch4` 实例在 `pwm.c` 末尾。Bluepill 移植版已分离到 `pwm_instance.c`

### 5. Servo_Driver_Design.md — Servo 驱动
- **状态**：✅ 已完成
- **注意**：当前 `servo_bridge_init` 接口签名为 `(id, protocol_type, handle)`，与设计文档一致

### 6. motor_arch_design.md — Motor 驱动
- **状态**：⚠️ 部分过时
- **需更新**：
  - P5（编码器）实际已实现 ✅，文档仍标"待开始"
  - P6（PID）实际已实现 ✅（`common/pid.c`），文档仍标"待开始"
  - P7（闭环扩展）待开始 ⏳
  - 新增 `encoder.c/h` + `encoder_platform_ops.c` 未在文档中体现

### 7. UART_Serial_Design.md — UART 驱动与串口协议
- **状态**：✅ 已完成
- **内容**：三层分离 + 基于枚举的二进制命令协议 + ringbuf 中断处理

### 8. UserI2C_Design.md — 软件 I2C 驱动
- **状态**：✅ 已完成
- **内容**：三层 FSM 状态机、时序分析、已知限制
- **注意**：Bluepill 移植版改用硬件 I2C2，软件 I2C 相关文件（`useri2c.c/h`、`useri2c_ops.c/h`）未使用但保留

### 9. I2C_Flow_Walkthrough.md — I2C 收发流程详解
- **状态**：✅ 已完成
- **内容**：完整事务轨迹、状态变量说明、关键数字汇总

### 10. IMU_Driver_Design.md — IMU 驱动
- **状态**：✅ 已完成
- **内容**：参数表驱动模式、MPU6050 实现、互补滤波

### 11. DMA_Driver_Analysis.md — DMA 驱动分析
- **状态**：✅ 结论：不需要 DMA 驱动

### 12. Timer_NonBlocking_Pattern.md — 定时器非阻塞模式
- **状态**：✅ 已完成
- **内容**：软件 bit-bang 协议（I2C/SPI/OneWire）的定时器中断驱动模式

### 13. State_Transition_Reference.md — I2C FSM 状态转换
- **状态**：✅ 已完成
- **内容**：完整写/读事务的 macro_state 轨迹

---

## 三、项目管理

### 14. Implementation_Backlog.md — 功能点清单
- **状态**：⚠️ 需要更新
- **需更新**：
  - P0 全部已完成（I2C 驱动、ISR 集成、motor_bridge 语法）✅
  - P1 部分完成（电机初始化仍 `#if 0` ❌，uart_flush_rx 未实现 ❌）
  - P2 大部分未开始
  - 新增 PID 模块、里程计上报、Encoder 驱动等未记录
  - 新增 Bluepill 移植相关事项未记录

---

## 四、移植相关文档

### 15. Bluepill_Porting_Plan.md — 移植执行清单
- **状态**：✅ 已完成
- **位置**：根目录
- **内容**：CubeMX 配置、代码迁移、验证步骤

### 16. Bluepill_Porting_Pitfalls.md — 移植踩坑记录
- **状态**：✅ 已完成
- **位置**：根目录
- **内容**：12 个移植问题的分析及解决方案

### 17. CrossPlatform_Porting_Preparation.md — 跨平台移植准备指南
- **状态**：✅ 已完成
- **位置**：根目录
- **内容**：同平台 vs 跨平台移植对比、代码中的隐藏平台依赖、移植前需要做的 6 项准备工作

---

## 文档与实际代码的一致性检查

| 文档 | 状态 | 说明 |
|------|------|------|
| Design_Philosophy.md | ✅ 一致 | 架构未变 |
| Design_Patterns.md | ✅ 一致 | 模式未变 |
| C_CPP_Bridge_Mechanism.md | ✅ 一致 | 桥接机制未变 |
| PWM_Driver_Design.md | ⚠️ 实例位置 | 原工程一致，Bluepill 实例已分离 |
| Servo_Driver_Design.md | ✅ 一致 | 接口未变 |
| motor_arch_design.md | ⚠️ 进度落后 | 编码器/PID 已实现但文档未更新 |
| UART_Serial_Design.md | ✅ 一致 | 协议未变 |
| UserI2C_Design.md | ⚠️ Bluepill 改用硬件 I2C | 原工程一致 |
| I2C_Flow_Walkthrough.md | ✅ 一致 | 时序未变 |
| IMU_Driver_Design.md | ✅ 一致 | 驱动未变 |
| Implementation_Backlog.md | ⚠️ 严重落后 | 大量已完成项未更新 |

---

> **上表为上游工程（A_board_pwm_driver_test）快照，不代表本仓库状态。**
> 本仓库（BluePill_Car）的文档-代码一致性请看本文档 §〇末的"本仓库代码-文档一致性已知缺口"。
> 上游驱动设计文档（Design_Philosophy / Design_Patterns / C_CPP_Bridge_Mechanism / motor_arch_design / PWM 与 Servo 驱动设计等）实体在
> `D:\666WorkSpace\STM32Cube\A_board_pwm_driver_test\`，本仓库仅在 `doc/移植文档/` 保留移植相关的 4 篇。
