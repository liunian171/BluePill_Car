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
- **状态（2026-09-15）**: 基础驱动链路全通（电机/编码器/IMU/OLED/串口/灰度）；**执行栈两侧组件已组件化**（`steering` 转向 ✅真机 / `speed_loop` 速度环 ✅真机）；**感知侧组件化推进中**（`odom` ✅ / `attitude` 姿态 ✅真机（C3，含 `common/imu_filter.c` 纯 C 移植）；巡线感知拆分待做）；**桥接层家族契约关键项真机通过**（`M1 30` 不发散 + SV 链回归，⬜ 剩故障注入）；**上位机对接批次落地并真机验证**（odom 组件 + ODOM/ATT 帧 + 看门狗 + 0x20，实转回归 14/14）；闭环调参工具链 + **IMU 调试工具链**（`ITEL/IGAIN/IDRIFT/IRATE/ICAL` + SOP）可用；**IMU-1 yaw 失真已修复**（漂移补偿陀螺门限，调试总结 §20）；巡线状态机已集成（实车整定未做）；上位机对接规范见 `README.md`
- **下次开工第一件事**: **提速决策**（ODOM 50Hz 需 USART3/USART2 提速，动 `.ioc` 需用户拍板）+ **C1 故障注入**（P0 唯一未跑项）；当前进度与待办见 `doc/开发跟踪.md` §当前进行中

## 二、硬件方案（全部已实测确认）

| 子系统 | 器件 | 关键参数 |
|--------|------|---------|
| 主控 | STM32F103C8T6 | 72MHz (HSE 8M+PLL×9)，Flash 64KB / RAM 20KB **紧约束** |
| 电机 | TB6612 ×2 通道 | TIM1 PWM 20kHz (PA8/PA9)，方向 PB12-15，M1 方向软件取反 |
| 编码器 | TIM2/TIM3 编码器模式 | **PPR=1466（实测）** |
| 轮组 | 车轮周长 20.5cm | max_rpm=319（实测标定，勿用 200） |
| 巡线 | 5 路灰度 PB3-PB7 | 黑线窄于间距，居中仅 S3 亮，二值采样 50ms/帧 |
| IMU | MPU6050 (I2C2 0x68) | **Mahony 四元数融合**(Kp=0.5, Ki=0) + 静止时陀螺零偏漂移补偿；启动 50 次采样初始零偏校准 |
| 显示 | SSD1315 OLED (I2C2 0x3C) | 128×64，4 行 8×16 |
| 舵机 | TIM4_CH3 (PB8) | 50Hz PWM |
| 串口 | USART2 (PA2/PA3) | **9600-8N1**，接 BT04 蓝牙（⚠️ 旧文档曾误写 115200，代码真值 9600）；二进制帧协议 + 文本调参命令 |
| 通信接口规范 | — | **上位机对接看 [`README.md`](README.md) §3**（上行四类 / 下行两类 / 时序带宽 / 已知限制） |

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
- 串口日志是第一证据来源。**默认无周期上报**（2026-09-11 真机核实），上行基础三类：
  ① 开机横幅 `UART2 Ready`（复位后一次）；② 命令应答 `ack()`（如 `SV:-90.0` / `M0:60RPM`）；③ 巡线诊断事件 `tx_raw`（`line_follower` 事件回调，仅巡线使能时）
  ④ **按需**（默认关）：调参遥测（`TEL/STEP/DUMP`）+ 上位机里程计帧（`ODOM 1` → 二进制 ODOM 0x51 @20Hz + ATT 0x52 @10Hz，格式见 README §3.8.1）
- IMU 欧拉角 / 编码器 / PID 等周期数据**默认只刷 OLED（页 0-7）**；开 `ODOM 1` 后欧拉角经 ATT 帧上串口
- **命令看门狗 `WD <ms>`**（默认关）：超时无下行字节自动急停——上位机断链兜底（PDF 安全机制条款），ROS 对接时建议 500ms
- 真机串口端口：BT04 出 SPP 口（本机实测 **COM15**，9600-8N1；端口号会随配对变化，可用 `python -c "import serial.tools.list_ports as p;[print(x.device,x.description) for x in p.comports()]"` 查）

## 四、工程架构现状

### 4.1 分层结构（ops 抽象模式；🔶 层命名 2026-09-11 按双栈模型修订，详见 doc/代码风格与模块衔接指南 §1，代码目录迁移随实施期）

```
组装层  main.c 用户区（实例创建 + 节拍接线 + 接管权仲裁 + 显示组版）
Core/Src/driver/
  ├─ 桥接层   motor_bridge/servo_bridge/oled_bridge/imu_bridge ← C 门面: 查 id → 转发
  ├─ 组件层   决策: line_follower.c(巡线状态机)                ← 物理量→运动意图
  │          感知: attitude.c(姿态 ✅C3 真机) / 巡线感知（待从 line_follower 拆出）← 裸数→物理量
  │          执行: steering.c(转向) / speed_loop.c(速度环) ✅ 均已成 ← 意图→器件级目标
  │          解析: txt_cmd.c(文本命令) + main.c 内联二进制帧分发（原 uart_cmd_parser 死代码已于 2026-09-13 删除）
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
  ① while 清空 ringbuf → 文本命令解析(txt_cmd) / 二进制帧解析(帧超时 100ms 重同步)
     （上行接口（帧/应答/遥测格式）权威见 README.md §3）
  每 50ms:  看门狗判定(WD 使能时) → line_follower_update(now, 意图缓冲) → 推给 speed_loop → speed_loop_update(now)
            → odom_update(now)（增量位姿；ODOM 1 时发 ODOM 0x51 + ATT 0x52 帧）
            （0 速 / 内轮停车 = 释放 PID + 物理刹停，语义在 speed_loop 组件内）
  每 100ms: imu_bridge_update_filter(0, now) + 读编码器 + OLED **单页轮转**（8 页 800ms 一轮）
            （2026-09-11 核实：无串口上报；遥测须按需开启，见 README.md §3.5/§3.8.1）
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
| `&hi2c2` 当 ops 句柄解引用 → 硬错误 | HAL 句柄必须经 `i2c_hardware_ops.c` 封装（或按新契约注入"写事务函数"） |
| GPIO 掩码二次移位 `1<<pin` 溢出 | `gpio_pin` 本身就是掩码，直传 HAL |
| APB2 定时器时钟多乘 2 | 有分频才 ×2：`PCLK2 < HCLK` 判断 |
| TB6612 stby 空指针 | GPIO 写前空指针保护（`safe_gpio_write`） |
| 轮询收串口丢字节 | 一律中断 + ringbuf，主循环 while 清空 |
| 编码器方向 | M1/E2 方向软件取反；**驱动侧 `drive_sign` 与反馈侧 `fb_sign` 必须镜像**（组装层相邻声明，改一个必须同时改另一个，否则正反馈飞车） |
| 用 `UART2 Ready` 横幅判断 init 成功 | 横幅在桥 init **之前**发送 → 判据应为 `PING` 有 `PONG`（`Error_Handler()` 会 `__disable_irq()` 死循环 → 无应答） |
| **蓝牙 SPP"连不上"三态**（2026-09-15，详调试总结 §19） | ① open 立即拒绝=口被僵尸进程占（测试脚本必带 `write_timeout`；工具静默死掉必查 `tasklist` python 残留并 `Stop-Process`）② open 阻塞 20s+=微软栈在建链（正常，调用超时给足 90s）③ 写超时=设备不可达（查板子供电/手机抢占单连接/设置里点"连接"）。BT04 找口：`Get-PnpDevice` InstanceId 匹配 MAC `98DA20045F4F`；配对 PIN `1234` |
| ⚠️ **本机环境：对 `.git/` 的写入会被拦截/回滚** | 症状：`git fetch` 报 `[new branch]` 但跟踪引用不落地（`[gone]`）、`git update-ref` 返回成功却写不进、`rm` 删 1 个文件却删掉多个（曾一次清空 70 个）、**`git rm <单文件>` 留 stale `index.lock` + 连带删工作区 35 个无关源文件 + 丢未跟踪新文件**（2026-09-15 实锤）→ **对策**：① **本仓库禁用 `git rm`**——用普通 `rm <单文件>` + `git add` + 即时 `git status --short` 校验 ② 中招处置：`rm .git/index.lock` 清锁 → `git restore -- <目录>` 逐目录恢复 → 未跟踪新文件重写 ③ 跟踪引用失联时手工写 `.git/packed-refs`（标准格式，两行即可）④ 怀疑杀软实时防护监控了工作区 |

## 六、文档索引

| 文档 | 定位 |
|------|------|
| `README.md` | **上位机对接入口**：串口通信接口（上行四类 / 下行两类 / 时序带宽 / 已知限制）+ 构建烧录 |
| `doc/工程文档.md` | 架构/引脚/协议/初始化流程权威参考 |
| `doc/开发跟踪.md` | 阶段/条目状态跟踪（W 无线 / SS 舵机 / 巡线 / 架构治理） |
| `doc/巡线逻辑设计文档.md` | 巡线状态机唯一权威（决策表+状态机+参数） |
| `doc/舵机转向设计文档.md` | 舵机阿克曼转向命令域/标定参数权威 |
| `doc/舵机代码结构对照分析.md` | 舵机链框架合规审查（S1-S11 偏差 + 三问分层判定 + 治理顺序） |
| `doc/调试总结.md` | 调试时间线 + 技术要点 + 电机参数 + 命令速查 |
| `doc/架构评审文档.md` | 耦合性/可配置性评审快照 + 治理优先级（C1-C8 发现编号） |
| `doc/代码风格与模块衔接指南.md` | 代码形态规范：分层性质/命名注释/机制规则/新模块衔接义务 8 项（🔶=新确立条款） |
| `doc/桥接层家族契约.md` | **四桥（motor/servo/imu/oled）共同形状与规则**：init 签名 / 错误通道 / 静态池 / 校验 / 知识归属 / oled 单例豁免 + 落地与真机清单 |
| `doc/结构优化顺序分析.md` | 优化治理顺序推演（契约先行/实现后搬）；C1/C2 已落地，当前进入硬件验证批次 |
| `doc/调参工具链规划.md` | 闭环调参工具链**补充件**：带宽/阻塞定量测算 + 与 C2 的先后关系（顺序结论在此） |
| `doc/调参工具链定位分析.md` | 调参工具链**主线**：A/B 两层产物 + 需求/风险/决策点 + P0/P1/P2 优先级 + 工作顺序 |
| `doc/闭环控制分析报告.md` | 闭环环节清单 + 周期/采样核算 + **问题分级（P0-1 PID 抗饱和 / P0-2 测速丢方向）** + 改进方向 |
| `doc/移植文档/Document_Index.md` | 文档总索引（§0 本仓库索引 + 上游驱动工程索引快照） |
| `doc/移植文档/CrossPlatform_Porting_Preparation.md` | 跨平台移植短板与隐藏平台依赖清单 |
| `backup/` | 历史版本备份，只读不维护 |

## 七、当前待办 / 待标定

**功能/标定**
- [ ] 电机速度系数标定（转一圈 + 走一米闭环验证；PPR/周长/max_rpm 已实测）
- [ ] PWM_SET_DUTY 命令实现（协议已定义 0x20）
- [ ] 巡线 PD 参数 + 三层阻尼实车整定（GK/GD/GS 在线调）
- [ ] 直角弯/SEARCH 状态机实车验证（PC 逻辑推演过，真机未验）
- [x] 帧接收超时重同步（`main.c:454`，100ms 无新字节丢弃半帧）✅ 已实现
- [ ] 帧加 CRC 校验（低优先级）

**当前主线（2026-09-13 定序：先"数据可信 + 看得见"，再动结构）**
- [x] ① **P0-1 `pid.c` 抗饱和修复**（钳位后回写 `prev_output`）✅ `9c926c3`
- [x] ② **P0-2 带符号测速** ✅ `9c926c3`；M1 镜像取反修正 `8f21a2d`
- [x] ③ **遥测通道** ✅ `fa012ad`（实际实现为 **10Hz CSV 文本** `TEL 1`，**不是** FireWater 二进制帧）
- [x] ④ 阶跃测试命令（`STEP`）+ PC 辨识脚本 + 操作规范 ✅ `fa012ad`/`778244b`/`4f8305d`；M0/M1 模型辨识完成 `4ede7bf`
- [x] ⑤ **C2 抽 `speed_loop`** ✅ **2026-09-13 完成**：`Core/Inc/driver/speed_loop.h` + `Core/Src/driver/speed_loop.c`（纯 C / 零 HAL / 输出·刹停·编码器读全注入 / tick 外部化）；`main.c` 删 13 个符号、只留标定表 `g_spd_cfg` + 3 个 IO 绑定，**连 `common/pid.h` 不再 include**
      证据：PC 桩 `test/host_speed_loop_test.c` **54/54**；阶跃搬迁前后稳态 **196.3→197.0 RPM（0.36%）**；真机 M0/M1/倒车 30RPM + 增益读回全过；Flash 45848B(69.96%) / RAM 6248B(30.51%)
      顺带修掉旧 bug：`PID_SET`/`SWAP`/OLED 页5 改为从组件读真值 → **二进制 `0xE0` 改参后不再回显陈旧的 ×100 镜像**（真机已验证）
- [x] ⑥ **C1 桥接层家族契约** ✅ **代码已落地 2026-09-13**：新增 `Core/Inc/driver/bridge_ret.h`（家族返回码）+ `doc/桥接层家族契约.md`（契约正文）
      四桥按契约重写：init 一律 `xxx_bridge_init(id, const xxx_cfg_t*) → bridge_ret_t`；补齐 id/handle/区间校验（`motor_bridge` **原先 init 都不查 id = 越界写静态池**）；错误通道定为**返回码**（桥不做 I/O → 保零依赖）
      **`M1 方向取反` 上提为 `motor_bridge_cfg_t.drive_sign`**（删除 3 处 `if (id == 1)` 特例）→ 与速度环 `fb_sign` 在组装层**相邻声明 + 注释互指**（飞车隐患根因：两侧符号分散在两地）
      imu 去堆 `new`（静态池）+ 校准状态 per-id（修 `cal_progress(id)` 忽略 id 的 bug）+ **去 `HAL_GetTick`**（时间基准传入，全桥家族零 HAL）；oled 改"写事务注入"（新增 `oled_platform_ops.h/.c`，器件层不再硬编码 `&hi2c2`）；顺带删 `imu_types[]` 死变量
      证据：Release **Flash 46268B(70.60%) / RAM 6408B(31.29%)**，仅 2 处历史告警；桩回归 txt_cmd 77/77 / steering 33/33 / speed_loop 54/54；⬜ **真机台架回归待硬件**
- [ ] ⑦ **硬件验证批次 ← 下一步（只差接线）**：① C1 台架回归（契约 §5.3，含 **`M1 30` 不发散**）② 示波器 PB8 脉宽 ③ 舵机动作目视 ④ OLED/IMU 目视
- 顺序理由：**P0 修复与 C2 搬迁范围重叠**（PID 段、测速段都在那 ~50 行里）→ 先修后搬是唯一不返工路径，详见 `doc/调参工具链规划.md` §7

**舵机链收尾**（详 `doc/舵机代码结构对照分析.md` §8）
- [ ] 示波器实测 PB8 脉宽（`SV -90`→833µs / `SV 0`→1019µs / `SV -200`→648µs）
- [ ] 舵机物理动作目视（上电回直行位、`SV 0` 到满舵）
- [x] 自查项 N1~N3 ✅ 2026-09-13：`servo_bridge_stop` 已删、`Servo::get_angle()` 已删、`steering_is_init/get_lim_*` 保留（PC 桩在用，属有用接口）
- [ ] 桥接层家族契约批次：S5（UART 空壳）/ S6（handle 校验）/ S8（错误通道形态）/ S11（感叹号注释）——与 IMU 链、oled 归一合并定方案

**代码卫生**
- [x] 死代码清理 ✅ **2026-09-13 执行完毕**（4 文件 639 行 + 3 处符号）：`uart_cmd_parser.c/.h`(465) + `imu_uart_handler.cpp/.h`(174) + `servo_bridge_stop`(N1) + `Servo::get_angle`(N2) + `tool.h` 的 `handle_to_id`；附带修正 `uart.h` 引用死路径的注释
      验证：CMake 重配置 + 编译通过（Flash 44636B/RAM 5784B，主要来自工具链新功能；**死代码本就被 `--gc-sections` 剔除，故删它不省 Flash，省的是认知成本**）；PC 桩回归 txt_cmd 77/77 + steering 33/33
      判定与纪律已固化：`doc/代码风格与模块衔接指南.md` §7（死代码 vs 预留代码的区分、三条判据、删除纪律）
- [ ] 有意预留项加显式标注（**勿删**）：`useri2c.c/.h`+`useri2c_ops.c/.h`（软 I2C，本工程用硬件 I2C2）、`pwm.h` 的 `PWM_Ch_State`/`Ch_State`/`TIM_PWM_g_Param`
- [ ] `main.c` 的 `firewater_send()`（20 行）**保留但属"预留未启用"**：遥测实际走 10Hz CSV（`g_tel_on`），该 FireWater 二进制帧至今零调用；**启用条件**=需 >10Hz 带宽或 VOFA+ 波形时启用，**否则按死代码删除**（判据见指南 §7）

**闭环参与**（⬇️ 已降级：用户 2026-09-11 决定巡线暂不使用、结构保留）
- [ ] `steering` 接入巡线 `corr → 转向`（**非当前路径**）；`steering` 目前维持"命令驱动的舵机"
- [ ] T1 转向模型（符号统一 / 非线性曲线 / 阿克曼差速）——同样等巡线重启后再做

> 待确认项用 ⚠️ 标注；新条目完成按 §5.4 闭环更新本节与 doc/。
> 架构治理（C1-C8）：已从"纯学习期"转为"边定契约边落地"（C6 ✅ / steering ✅）；顺序按 `doc/结构优化顺序分析.md` §8.2，组装层减肥与 C4 仲裁排在组件稳定之后。
