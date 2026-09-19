<!-- 编码: UTF-8 -->
# 节拍拉长专案 — USB 迁移后 ODOM 10Hz 之谜（2026-09-19 开案）

> 状态：🔴 待验证（主嫌疑已锁定，一个动作即可判死判活）
> 上游：USB CDC 双链路迁移（见 `双链路仲裁设计文档.md`、`调试总结.md` §23）
> 本专案解决：为什么 USB 链路上 ODOM 只有 10Hz（设计 20Hz）、TEL 5Hz、ITEL 612ms

---

## 1. 现象时间线（2026-09-19 下午）

| 时间 | 事件 | 观察 |
|------|------|------|
| 17:56 | power cycle 后 P2 仲裁验证 | **10/10 全过**，USB 链路正常 |
| 18:14 | 首次 ODOM 速率测试 | ODOM **10.1Hz**（ts 间隔精确 100ms，零丢帧），ODOM+TEL 并行吞吐 380 B/s |
| 18:16 | ITEL 抽样 | euler 有值（-18.0/-21.5/-14.8°），数据活着 |
| 18:07 前后 | 用户拔掉 OLED 的 **VCC 和 SDA 两根线**（SCL/GND 仍连着） | — |
| 18:19 | ITEL 复测 | **全零行**（gyro 0,0,0 = 读失败零值回退）+ 间隔 **612ms** |

## 2. 已排除项（逐条有证据）

| 嫌疑 | 排除证据 |
|------|---------|
| USB 链路丢包 | ODOM 帧时间戳连续无跳变，101/101 帧零丢失；600 B/s 远低于 FS 能力 |
| OLED 线接触不良拖慢 I2C | **OLED 整体拔掉后 ODOM 仍 10.2Hz**（判定实验 18:14） |
| IMU 节拍占用 | `IRATE 1000` 后 TEL 间隔仍 200ms，无改善 |
| odom_update 拒帧 | `ODOM_DT_MAX_MS` 逻辑正常，帧连续无 resync 断点 |
| Error_Handler 挂死 | SWD HotPlug 读 uwTick 持续前进；dev_state/主循环均活 |
| 仲裁误切 | 全程 owner=USB（LINK USB 显式抢回后） |

## 3. 主嫌疑机理：I2C 总线寄生供电（半断电陷阱）

**拔线方式 = 只拔 OLED 的 VCC+SDA，SCL/GND 仍连。** 后果链：

```
OLED 失去 VCC → 但 SCL/GND 仍连 → OLED 芯片经 I2C 引脚 ESD 二极管
从总线寄生取电 → 芯片处于半上电病态 → 拉扯总线电平
→ 同一条 I2C2 总线上的 MPU6050 读写开始失败（每次 HAL 超时 10ms）
→ 每轮主循环吃 1~N 个 10ms 超时 → pass 从 ~49ms 膨胀到 100~300ms+
→ 50ms 控制环"每 2 轮才凑满 50ms" → ODOM 20Hz→10Hz、TEL 10Hz→5Hz
→ ITEL 612ms 间隔 = pass≈306ms 的自洽证据
```

**佐证**：
- ITEL 全零行 = main.c 中 `read_gyro_raw` 失败的零值回退路径被触发（MPU 读死）
- 18:16（刚拔线时）euler 还活着 → 18:19 全零 → **渐进劣化**，符合接触/寄生状态漂移
- ODOM ts 间隔"精确 100ms"→"ITEL 612ms"两个观测分别对应 pass=100ms 和 306ms 两个劣化阶段

## 4. 下一步验证（新会话按此执行，判死判活一个动作）

1. **OLED 4 根线全部恢复连接**（VCC/GND/SDA/SCL 齐全、插牢）——不要只插一半
2. **拔插 USB 一次**（= VBUS power cycle，板子唯一电源；F103 软复位不重枚举，见调试总结 §23）
3. 等 Windows 枚举出 COMx（`tools/usb_ping_test.py <COMx>` 或 PING 验证）
4. `LINK USB` → `ODOM 1` → 采 10s 数 ODOM 帧（0xAA 0x51 … FF FF 定长 20B）
5. **判定**：
   - ODOM 回到 **~20Hz** → 专案结案：根因 = I2C 总线寄生供电 + 初始化/使用期间的 I2C 超时膨胀；USB 链路本身无任何问题；同步解释 OLED 黑屏（同一次接线松动）
   - 仍 10Hz → 排除外设，下一层嫌疑 = USB 栈与主循环相互作用（届时用 DWT->CYCCNT 测 pass 耗时定位到块）

## 5. 连带修复项（结案后处理）

| 项 | 内容 |
|----|------|
| RST 后 OLED 黑屏 | 与本专案同源（接线松 → I2C 异常），恢复连接后应自愈；若复现再查 |
| 测试纪律 | **硬件拔插实验必须整组线一起处理**（电源+信号），单拔 VCC 的"半断电"状态本身就是故障源 |
| 架构观察（低优先） | 显示器件 init/运行失败是否应触发"init 失败即停"值得再权衡——显示属非关键外设，挂死整车代价大（本次 OLED 缺席时固件仍活着，说明运行期容忍良好，但 init 期行为待核） |

## 6. 复现/测量工具速查

| 工具 | 命令 |
|------|------|
| SWD 判活 | `STM32_Programmer_CLI -c port=SWD mode=HotPlug -r 0x20000368 4 t.bin`（uwTick@0x20000368，两次对比；**不加 mode=HotPlug 会复位 MCU 并打死 USB**） |
| USB 栈状态 | HotPlug 读 0x20000B04 起 48B：byte0=dev_state(3=CONFIGURED)，+40 处 pClassData（NULL=未配置） |
| ODOM 速率 | pyserial 收 `AA 51 ... FF FF` 20B 帧，数帧率 + 帧内 ts(ms) 差分 |
| 注意 | 板子唯一电源 = USB VBUS；拔插 USB = 真 power cycle；SWD attach(默认模式) = 复位（会打死 USB 枚举） |
