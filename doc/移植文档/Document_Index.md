# 设计文档索引与状态

> 工程所有设计文档的总索引，标注每个文档的当前完成状态和与实际代码的一致性。
> 位于 `A_board_pwm_driver_test/` 根目录。

---

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
