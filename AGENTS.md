<!-- 编码: UTF-8 -->
# AGENTS.md — BluePill_Car 循迹小车

> 本文件为 Agent 在本项目的综合引导文件：项目定义、硬件方案、架构现状、开发规范。
> 创建: 2026-09-09（借鉴 `Opencode\AGENTS.md`、`Opencode\Hexapod\AGENTS.md`、`Opencode\SimpleCar\AGENTS.md` 提炼）
> 工作区: `D:\666WorkSpace\STM32Cube\BluePill_Car`（全英文路径）

---

## 一、项目定义

**基于 STM32F103C8T6 (BluePill) 的两轮差速循迹小车**：

```
决策层   line_follower   5路灰度 → pos/EMA → PD + 三层阻尼 + 直角弯状态机   (50ms 周期)
执行层   PID 速度环      目标 RPM vs 编码器实测 → PWM 占空比                 (每电机)
驱动层   ops 抽象        motor/servo/encoder/pwm/uart/gpio/i2c 策略层+平台层
硬件层   STM32 HAL       CubeMX 生成代码 (main.c 用户段之外勿动)
```

- **代码权威源**: 本仓库 `Core/`（CubeMX + CMake + Ninja 工程）——一手真相
- **文档区**: `doc/`（架构/调试/巡线设计/移植，见 §六 索引）；`backup/` 为历史备份，只读
- **状态**: 基础驱动链路全部跑通（电机/编码器/IMU/OLED/串口/灰度），巡线状态机已集成进 main.c，参数标定部分待完成

## 二、硬件方案（全部已实测确认）

| 子系统 | 器件 | 关键参数 |
|--------|------|---------|
| 主控 | STM32F103C8T6 | 72MHz (HSE 8M+PLL×9)，Flash 64KB / RAM 20KB **紧约束** |
| 电机 | TB6612 ×2 通道 | TIM1 PWM 20kHz (PA8/PA9)，方向 PB12-15，M1 方向软件取反 |
| 编码器 | TIM2/TIM3 编码器模式 | **PPR=1466（实测）** |
| 轮组 | 车轮周长 20.5cm | max_rpm=319（实测标定，勿用 200） |
| 巡线 | 5 路灰度 PB3-PB7 | 黑线窄于间距，居中仅 S3 亮，二值采样 50ms/帧 |
| IMU | MPU6050 (I2C2 0x68) | 互补滤波 + 动态α + 运行中零偏补偿 |
| 显示 | SSD1315 OLED (I2C2 0x3C) | 128×64，4 行 8×16 |
| 舵机 | TIM4_CH3 (PB8) | 50Hz PWM |
| 串口 | USART2 (PA2/PA3) | 115200-8N1，接 BT04 蓝牙；二进制帧协议 + 文本调参命令 |

引脚全表见 `doc/工程文档.md`；**`.ioc` 无绝对把握不擅自修改**（用户铁律），改引脚/外设先问用户。

## 三、构建与烧录（Agent 直驱）

工具链: STM32CubeCLT（ARM GCC / CMake / Ninja / CubeProgrammer），LN 机已验证。

```powershell
# 配置（首次或改 CMakeLists 后）
cmake --preset Debug
# 编译
cmake --build build/Debug
# 编译+烧录 一键（SWD, 烧完自动复位）
.\flash.bat
# 仅烧录
STM32_Programmer_CLI.exe -c port=SWD -w build/Debug/BluePill_Car.elf 0x08000000 -rst
```

**硬件在环两级模式**（沿用 SimpleCar 协议）：
- **一级（优先）**: Agent 直驱——用户接线完毕告知 → Agent 执行 flash.bat 烧录 + 串口抓日志判读
- **二级（兜底）**: 指令-回读——Agent 给完整命令行 + 预期现象清单，用户复制执行后回读输出
- 串口日志是第一证据来源；每 250ms 主动上行 `IMU R:.. P:.. Y:.. ENC1:.. ENC2:..`（校准完成后）

## 四、工程架构现状

### 4.1 分层结构（ops 抽象模式；🔶 层命名 2026-09-11 按双栈模型修订，详见 doc/代码风格与模块衔接指南 §1，代码目录迁移随实施期）

```
组装层  main.c 用户区（实例创建 + 节拍接线 + 接管权仲裁 + 显示组版）
Core/Src/driver/
  ├─ 桥接层   motor_bridge/servo_bridge/oled_bridge/imu_bridge ← C 门面: 查 id → 转发
  ├─ 组件层   决策: line_follower.c(巡线状态机)                ← 物理量→运动意图
  │          感知: 姿态/巡线感知（待从 imu_bridge/line_follower 拆出）← 裸数→物理量
  │          执行: steering.c(转向) ✅已建 / 速度环（待从 main 拆出）← 意图→器件级目标
  │          解析: txt_cmd.c(文本命令) uart_cmd_parser.c(协议帧, 死代码待处置)
  ├─ 执行对象层 motor.cpp servo.cpp (旧称功能对象)               ← 器件级目标→器件语言
  ├─ 器件层   TB6612MotorProtocol / PWMServoProtocol / MPU6050(IIMU) (旧称业务对象) ← 器件翻译
  ├─ 驱动层   策略层 pwm.c encoder.c uart.c usergpio.c useri2c.c ← 平台无关
  │          平台层 *_platform_ops.c / i2c_hardware_ops.c      ← 唯一碰 HAL 的地方
Core/Src/common/     通用算法层: ringbuf / pid / imu_filter(滤波核心) / tool
```

数据方向：感知链上行（裸数→物理量）→ 决策（物理量→意图）→ 执行链下行（意图→寄存器）；命令链与显示链经桥接层/组装层正交接入。

- **跨平台约定**: 策略层禁止 include HAL/CubeMX 头；平台依赖全部收敛到 `*_platform_ops.c`（详 pits 见 `doc/移植文档/CrossPlatform_Porting_Preparation.md`）
- **C/C++ 边界**: `*_bridge.h` 提供 `extern "C"` 接口；C++ 对象一律 `placement new` + 静态池，**禁堆 new**（heap 仅 512B，分配失败静默返回 NULL → 硬错误，已踩坑）

### 4.2 主循环时序

```
while(1):
  uart_cmd_parser_tick()      ← while 循环清空 ringbuf（逐字节会丢帧，已踩坑）
  每 50ms:  line_follower_update → PID 速度环 ×2 → motor_bridge_set_speed_rpm
  每 100ms: OLED 刷新（先清行再写）+ VOFA+ 上报
  每 250ms: IMU 滤波更新（前 100 次 ≈25s 为零偏校准期）+ 文本状态上报
```

### 4.3 巡线状态机（详见 doc/巡线逻辑设计文档.md，唯一权威）

- `FOLLOW(PD+三层阻尼) → LINE_EXIT(弯前刹车200ms) → TURNING(单轮支点转弯) → 回正冷却800ms → FOLLOW`；追线丢线 → `SEARCH(800ms 周期摆扫, 4s 超时停车)`
- 三层阻尼 = 回中反向阻尼(corr×-0.5 持续2帧) + 过冲衰减 g_decay(×0.5/次) + 迟滞(连续2帧确认)
- 直角判定看 **00000 前一帧图案**（右侧残留=右直角），TURNING 中全白是正常态禁超时
- 调参命令: `GK/GD/GS/L/LA/GI/GC/GT`

## 五、Agent 开发规范（执行文档中的开发规范 = 本节）

### 5.1 核心行为

1. **一手真相**: 源码 > `doc/` 文档 > 会话记忆；文档与代码冲突时以代码为准并回写文档
2. **不臆想**: 电机系数/PI 参数/硬件现象等缺失时先问用户或标"⚠️ 待标定"；用标准假设必须显式标注来源
3. **文档驱动**: 改代码前先确认方案；改协议/引脚/架构先更新 `doc/` 再动代码
4. **确认机制**: 不确定的接线/参数/现象，先提问再动手；确认过的决策写入文档避免重复确认

### 5.2 资源约束编码铁律（本项目特色，F103 64K/20K）

1. **禁堆 new / malloc**: C++ 对象用 `placement new` + 静态池（现成范本 `motor.cpp`）
2. **禁 `%f`**: nano.specs 不含浮点 printf，一律整数拆分 `%d.%d`
3. **ISR 极简**: 中断里只 `ringbuf_write` + 重挂接收，业务逻辑全在主循环
4. **阻塞调用限时**: I2C 超时 10ms（100ms 曾造成命令延迟 5s，已踩坑）；显示刷新与命令解析分离
5. **每次改动后看 Flash/RAM 占用**（链接输出），Flash > 90% 需预警用户

### 5.3 测试友好编码（继承 Hexapod §6.5，5 条铁律）

1. **业务逻辑与硬件分离**（最重要）: 算法/状态机/协议层禁止直调 HAL；硬件访问走 ops 表/注入接口。反例：巡线逻辑里直接 `HAL_GPIO_ReadPin`
2. **Mock/Stub + UNIT_TEST 宏**: 外设读写封装独立接口，头文件 `#ifdef UNIT_TEST` 提供静态模拟
3. **DEBUG_LOG 分级宏代替裸 printf**: 状态机跳转处必打日志（串口日志 = Agent 的眼睛）
4. **时间基准外部化**: 延时/超时一律宏定义，tick 获取抽接口，逻辑层禁硬调 `HAL_GetTick()`
5. **测试点清单交付**: 每模块附测试前置条件：测试项 / 输入条件 / 预期日志 / 是否需万用表示波器

### 5.4 条目完成闭环（每完成一个条目立即执行，不攒到收尾）

```
1. 更新 doc/开发跟踪.md（条目状态 + 验证证据）    ← 尚未建立，首个开发条目时创建
2. 记录 doc/调试总结.md（现象 → 根因 → 解决）     ← 成功验证也记（现象=验证内容，解决=实测数据）
3. 更新 TODO 勾选 + 当前进行中区
4. 本地 git commit（信息写明模块与验证状态；本地优先，远端推送可选）
```

**收尾自查**: 准备 commit 前过一遍——调试记录漏没漏、TODO 是否挂着已完成的 `[x]`、文档状态与代码是否一致。

### 5.5 状态标注约定

- ✅ 完成（附验证证据）/ 🔄 进行中 / ⬜ 未开始 / ⚠️ 待确认·待标定
- **文档 ≠ 完成**: 只有明确标注"✅ 完成/跑通/已验证"才算完成，无标注一律视为未完成
- **PC 测试通过 ≠ 硬件验证通过**——两类证据分开标注

### 5.6 已踩坑速查（新码规避，完整版见 doc/调试总结.md）

| 坑 | 规避 |
|----|------|
| `&hi2c2` 当 ops 句柄解引用 → 硬错误 | HAL 句柄必须经 `i2c_hardware_ops.c` 封装 |
| GPIO 掩码二次移位 `1<<pin` 溢出 | `gpio_pin` 本身就是掩码，直传 HAL |
| APB2 定时器时钟多乘 2 | 有分频才 ×2：`PCLK2 < HCLK` 判断 |
| TB6612 stby 空指针 | GPIO 写前空指针保护（`safe_gpio_write`） |
| 轮询收串口丢字节 | 一律中断 + ringbuf，主循环 while 清空 |
| 编码器方向 | M1/E2 方向软件取反已做，改动前确认符号约定 |

## 六、文档索引

| 文档 | 定位 |
|------|------|
| `doc/工程文档.md` | 架构/引脚/协议/初始化流程权威参考 |
| `doc/开发跟踪.md` | 阶段/条目状态跟踪（W 无线 / SS 舵机 / 巡线 / 架构治理） |
| `doc/巡线逻辑设计文档.md` | 巡线状态机唯一权威（决策表+状态机+参数） |
| `doc/舵机转向设计文档.md` | 舵机阿克曼转向命令域/标定参数权威 |
| `doc/舵机代码结构对照分析.md` | 舵机链框架合规审查（S1-S11 偏差 + 三问分层判定 + 治理顺序） |
| `doc/调试总结.md` | 调试时间线 + 技术要点 + 电机参数 + 命令速查 |
| `doc/架构评审文档.md` | 耦合性/可配置性评审快照 + 治理优先级（C1-C8 发现编号） |
| `doc/代码风格与模块衔接指南.md` | 代码形态规范：分层性质/命名注释/机制规则/新模块衔接义务 8 项（🔶=新确立条款） |
| `doc/结构优化顺序分析.md` | 优化治理顺序推演（契约先行/实现后搬）；⚠️ 当前为学习思考期，未启动实施 |
| `doc/移植文档/Document_Index.md` | 文档总索引（§0 本仓库索引 + 上游驱动工程索引快照） |
| `doc/移植文档/CrossPlatform_Porting_Preparation.md` | 跨平台移植短板与隐藏平台依赖清单 |
| `backup/` | 历史版本备份，只读不维护 |

## 七、当前待办 / 待标定

- [ ] 电机速度系数标定（转一圈 + 走一米闭环验证；PPR/周长/max_rpm 已实测）
- [ ] PWM_SET_DUTY 命令实现（协议已定义 0x20）
- [ ] 巡线 PD 参数 + 三层阻尼实车整定（GK/GD/GS 在线调）
- [ ] 直角弯/SEARCH 状态机实车验证（PC 逻辑推演过，真机未验）
- [x] 帧接收超时重同步（`main.c:454`，100ms 无新字节丢弃半帧）✅ 已实现
- [ ] 帧加 CRC 校验（低优先级）

> 待确认项用 ⚠️ 标注；新条目完成按 §5.4 闭环更新本节与 doc/。
> 架构治理（C1-C8）当前挂起：处于用户学习思考期，启动时按 `doc/结构优化顺序分析.md` 序列执行。
