/* USER CODE BEGIN Header */
/**
  * @file           : main.c
  * @brief          : 完整版 — 串口命令 + PWM + OLED + IMU + 编码器 + 灰度
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "dma.h"
#include "i2c.h"
#include "tim.h"
#include "usart.h"
#include "usb_device.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "car_config.h"          /* 组装层配置单: 功能/链路编译开关 (铁律: 仅组装层+CMake 可 include) */
#include "usbd_cdc_if.h"
#include "driver/link_arbiter.h"
#include "app/app_tx.h"
#include "app/app_control.h"
#include "app/app_display.h"           /* 100ms 显示节拍宿主 (2026-09-20 拆分) */          /* 50ms 控制节拍宿主 (2026-09-20 拆分) */                /* 发送出口/路由/编号换算 (2026-09-20 拆分) */
#include "driver/pwm.h"
#include "driver/uart.h"
#include "driver/uart_platform_ops.h"
#include "driver/motor_bridge.h"
#include "driver/usergpio.h"
#include "driver/usergpio_platform.h"
#include "driver/encoder.h"
#include "driver/oled_bridge.h"
#include "driver/oled_platform_ops.h"
#include "driver/imu_bridge.h"
#include "driver/i2c_hardware_ops.h"
#include "driver/line_follower.h"
#include "driver/txt_cmd.h"
#include "driver/odom.h"
#include "driver/servo_bridge.h"
#include "driver/steering.h"
#include "driver/speed_loop.h"
#include "common/ringbuf.h"
#include <string.h>
#include <stdarg.h>
#include <stdio.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* [P1 双链路] 链路编号与消费预算 (link_arbiter P2 落地前的过渡定义) */
#define LINK_UART         0
#define LINK_USB          1
#define LINK_COUNT        2
#define LINK_BYTE_BUDGET  64    /* 每链路每轮主循环最多消费字节数 (时间片封顶) */

/* ---- [P3 开关收口] 链路编译开关落到组装层接线 ----
 * 真值源 = car_config.h（本文件是唯一被允许 include 它的 C 文件；组件层/桥接层禁 include）。
 * 语义 = "这条链路的接线不存在"，而不是"上层记得别调"：
 *   CAR_FEATURE_USB=0  → 消费循环整条跳过 / 发送 no-op / 判活恒假 / 仲裁固定 UART
 *   CAR_FEATURE_UART=0 → 不开中断接收 / 发送 no-op / 仲裁固定 USB
 *                        （平台 ops 同时由 CMake 换成 uart_platform_ops_stub.c）
 * 双 0 无意义（车失联）→ CMakeLists configure 期已 FATAL_ERROR 拒绝。 */
#define LINK_UART_ENABLED  CAR_FEATURE_UART
#define LINK_USB_ENABLED   CAR_FEATURE_USB

#if (LINK_UART_ENABLED == 0) && (LINK_USB_ENABLED == 0)
#error "car_config.h: CAR_FEATURE_UART/USB 不能同时为 0 (车将没有可用控制链路)"
#endif

/* 双链路连通时仲裁自主切换；单链路时仲裁"固定"在存活链路（不喂伪判活、不发 failover） */
#define LINK_BOTH_ENABLED  (LINK_UART_ENABLED && LINK_USB_ENABLED)
/* 唯一存活链路（单链路模式下仲裁初始判活的固定值 + 禁用链路的发送回退目标） */
#if LINK_USB_ENABLED
#define LINK_FALLBACK      LINK_USB
#else
#define LINK_FALLBACK      LINK_UART
#endif
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
UART_Handle    uart_debug = {
    .huart      = &huart2,
    .ops        = &uart_platform_ops_stm32,
};
extern USBD_HandleTypeDef hUsbDeviceFS;   /* USB 判活/发送保护用 (usb_device.c 定义) */
/* [P1 双链路] 命令链路缓冲: UART(BT04) 与 USB CDC 各一 — 单生产者设计,
 * 两个 ISR 必须各写各的 buf, 共写会竞态 (doc/双链路仲裁设计文档.md §3.1) */
RingBuffer g_ringbuf_uart;
RingBuffer g_ringbuf_usb;
static RingBuffer *const g_link_rb[LINK_COUNT] = { &g_ringbuf_uart, &g_ringbuf_usb };
/* [P3 开关收口] 链路启用表 (与 g_link_rb 同序): 0 = 该链路编译期下线,
 * 消费循环整条跳过 —— 禁用链路的 ringbuf 即便被写也无人读, 不留"半通"状态 */
static const uint8_t g_link_on[LINK_COUNT] = { LINK_UART_ENABLED, LINK_USB_ENABLED };
/* 应答路由状态: 最近一条命令的来源链路 (应答/遥测按来源回; owner 判定归 link_arbiter)
 * [P3] 初值取存活链路 (USB 下线时不该默认把应答往不存在的链路送) */
static uint8_t g_active_link = LINK_FALLBACK;
/* 链路活动时刻 (OLED 页7 显示): UART 侧 MCU 无法感知 SPP 是否连接,
 * 用"距最近收字节的秒数"作活动指标; USB 用 dev_state+pClassData 判枚举配置态 */
static uint32_t g_last_uart_rx_tick = 0;
static uint8_t  g_uart_seen = 0;
/* [P2 双链路仲裁] USB 判活信号 (usbd_cdc_if.c DTR 捕获) */
extern volatile uint8_t g_usb_dtr;

UserGPIO_Handle motor_a_in1 = { GPIOB, GPIO_PIN_13, &usergpio_platform_ops_stm32 };
UserGPIO_Handle motor_a_in2 = { GPIOB, GPIO_PIN_12, &usergpio_platform_ops_stm32 };
UserGPIO_Handle motor_b_in1 = { GPIOB, GPIO_PIN_14, &usergpio_platform_ops_stm32 };
UserGPIO_Handle motor_b_in2 = { GPIOB, GPIO_PIN_15, &usergpio_platform_ops_stm32 };

Encoder_Handle henc1, henc2;
static I2C_Handle imu_i2c = {
    .i2c_context = &hi2c2,
    .ops         = &i2c_hardware_platform_ops_stm32,
};

/* ==== 桥接层配置表（C1 家族契约：init 一律"配置结构注入"）====
 * 桥只做"查 id → 校验 → 转发"，所有硬件接线知识（引脚/句柄/方向/地址）在此注入。 */

/* ⚠️ drive_sign（驱动侧方向）与下方速度环标定表的 fb_sign（反馈侧符号）**必须镜像**。
 *    两者是同一个硬件事实的两面（MOTOR B 的驱动与编码器接线均反相）；
 *    只改一侧 = 正反馈飞车（2026-09-13 真机实证：两侧失配时 M1 上电持续加速）。
 *    故两者必须相邻声明，改动时成对修改 —— 数值真值源现已集中到 car_config.h
 *    的 CAR_M1_DRIVE_SIGN / CAR_M1_FB_SIGN（2026-09-19 标定集中）。 */
static const motor_bridge_cfg_t g_motor_cfg[2] = {
    { .protocol = MOTOR_PROTOCOL_TB6612,
      .pwm = &pwm_tim1_ch1, .ain1 = &motor_a_in1, .ain2 = &motor_a_in2, .stby = NULL,
      .max_rpm = CAR_MOTOR_MAX_RPM, .wheel_radius_mm = CAR_WHEEL_RADIUS_MM,
      .drive_sign =  1 },
    { .protocol = MOTOR_PROTOCOL_TB6612,
      .pwm = &pwm_tim1_ch2, .ain1 = &motor_b_in1, .ain2 = &motor_b_in2, .stby = NULL,
      .max_rpm = CAR_MOTOR_MAX_RPM, .wheel_radius_mm = CAR_WHEEL_RADIUS_MM,
      .drive_sign = CAR_M1_DRIVE_SIGN },   /* ← 必须与 g_spd_cfg.ch[1].fb_sign 同源 */
};

static const servo_bridge_cfg_t g_servo_cfg = {
    .protocol = SERVO_PROTOCOL_PWM,
    .handle   = &pwm_tim4_ch3,
};

static const oled_cfg_t g_oled_cfg = {
    .i2c_context = &hi2c2,
    .write       = oled_platform_write_stm32,   /* 平台绑定（器件层零 HAL） */
    .addr7       = 0x3C,
};

static const imu_bridge_cfg_t g_imu_cfg = {
    .type       = IMU_MPU6050,
    .i2c_handle = &imu_i2c,
};

/* ==== 速度环执行组件 (speed_loop) 的标定表 —— 组装层只留"配置 + 接线" ====
 * PID / 前馈 / 测速 / 限幅 / 停车语义 / 状态保持 已全部归位到组件内部（C2, 2026-09-13）。
 * 数值真值源集中到 car_config.h「整车标定表」（2026-09-19 标定集中）：
 *   ppr        : 编码器每转脉冲数（init 时用 henc 实测值覆盖，标定表值作缺省回退）
 *   enc_span   : 16 位定时器计数模值（回绕校正用）；换 32 位编码器置 0
 *   fb_sign    : 反馈符号，必须镜像驱动侧取反（E2 硬件接反 → id1 = CAR_M1_FB_SIGN）
 *   ff_gain    : 前馈系数（FF 命令在线整定，辨识值 1/K ≈ 1.0）
 *   out ±max   : 输出限幅 = 电机 max_rpm 实测（与电机桥同源，改一处即可）
 *   kp/ki/kd   : SIMC 阶跃辨识值（2026-09-13，见 tools/step_ident.py） */
static speed_loop_cfg_t g_spd_cfg = {
    .ch_count = 2,
    .ch = {
        /* id0 (MOTOR A, E1 同向) */
        { .ppr = (float)CAR_PPR, .enc_span = CAR_ENC_SPAN, .fb_sign =  1,
          .ff_gain = CAR_SPD_FF_GAIN, .out_min = -CAR_MOTOR_MAX_RPM, .out_max = CAR_MOTOR_MAX_RPM,
          .kp = CAR_SPD_KP, .ki = CAR_SPD_KI, .kd = CAR_SPD_KD },
        /* id1 (MOTOR B, E2 硬件接反 → 反馈符号与驱动侧同源镜像) */
        { .ppr = (float)CAR_PPR, .enc_span = CAR_ENC_SPAN, .fb_sign = CAR_M1_FB_SIGN,
          .ff_gain = CAR_SPD_FF_GAIN, .out_min = -CAR_MOTOR_MAX_RPM, .out_max = CAR_MOTOR_MAX_RPM,
          .kp = CAR_SPD_KP, .ki = CAR_SPD_KI, .kd = CAR_SPD_KD },
    }
};

/* 页4/页6 数据源已迁 app_display (note_cmd/note_resp 喂数) */

/* ==== [app_control 拆分 2026-09-20] 调参/IMU 工具链/上位机上报/WD 的状态与
 * 50ms 执行体已迁往 Core/Src/app/app_control.c —— 本文件仅保留:
 *   g_last_rx_tick (owner 链最近下行字节, 消费循环写入, 经 io 读出喂 WD) ==== */
static uint32_t     g_last_rx_tick  = 0;       /* 最近下行字节时刻 (主循环每字节刷新) */

/* ==== 里程计感知组件 (odom) 的标定表 + IO 绑定 — 车知识集中注入 ====
 * 数值真值源 = car_config.h「整车标定表」（2026-09-19 标定集中） */
static const odom_cfg_t g_odom_cfg = {
    .ppr    = { (float)CAR_PPR, (float)CAR_PPR },   /* 编码器实测 */
    .wheel_circ_mm   = CAR_WHEEL_CIRC_MM,           /* 轮周长 (README 权威 20.5cm) */
    .wheel_track_mm  = CAR_WHEEL_TRACK_MM,          /* 轮距 ⚠️ 未标定 — 本车走 IMU yaw 差分, 不参与 */
    .enc_span        = (float)CAR_ENC_SPAN,
    .sign            = { 1, CAR_M1_FB_SIGN },       /* ⚠️ 与 g_spd_cfg.ch[].fb_sign 同源 (E2 接反) */
    .yaw_sign        = CAR_YAW_SIGN,                /* MPU6050 安装方向 (顺时针为正 → 取反)
                                                     * (2026-09-14 真机: 顺时针转车头 yaw +16.5°, 见开发跟踪 R-1) */
};
static int32_t odom_io_read_enc(uint8_t id)
{
    return (id == 0) ? encoder_get_count(&henc1) : encoder_get_count(&henc2);
}
static void odom_io_get_yaw(float *yaw_deg)
{
    *yaw_deg = imu_bridge_get_yaw(0);        /* ° , 逆时针为正 (Mahony 融合输出) */
}
static const odom_io_t g_odom_io = {
    .read_enc = odom_io_read_enc,
    .get_yaw  = odom_io_get_yaw,             /* NULL 则回退编码器差分(需轮距) */
};
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
/* ==== 速度环组件的 IO 绑定（组装层职责：把组件接口接到器件/传感器）====
 * 组件只持函数指针（不 include 桥接层头），故执行栈向上不反向依赖。
 * 换驱动芯片 / 换编码器接口只改这三个绑定，组件与上层不动。 */
static void spd_io_set_rpm(uint8_t id, float rpm) { (void)motor_bridge_set_speed_rpm(id, rpm); }
static void spd_io_brake  (uint8_t id)            { (void)motor_bridge_brake(id); }
static int32_t spd_io_read_enc(uint8_t id)        { return (id == 0) ? encoder_get_count(&henc1)
                                                                      : encoder_get_count(&henc2); }
static const speed_loop_io_t g_spd_io = {
    .set_rpm  = spd_io_set_rpm,
    .brake    = spd_io_brake,
    .read_enc = spd_io_read_enc,
};

/* 状态读出小工具: 带符号浮点 → 四舍五入到 int（遥测/显示共用，避免各处重复写） */
static int spd_to_int(float v)
{
    return (int)((v >= 0.0f) ? (v + 0.5f) : (v - 0.5f));
}

/* 中断回调 (UART 链路禁用时不接中断, 本回调不会被触发; 门控只为消除"半通"歧义)
 * [P1-1 补回该层 2026-09-20] 重挂改经策略层 uart_receive_IT() → ops → HAL,
 * 不再直调; UART=0 时经 stub 自然 no-op (P3 门控升级为真两层) */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *hal_huart)
{
    if (hal_huart->Instance != USART2) return;
#if LINK_UART_ENABLED
    ringbuf_write(&g_ringbuf_uart, uart_debug.rx_byte);
#endif
    (void)uart_receive_IT(&uart_debug, &uart_debug.rx_byte);
}

/* ==== [app_tx 拆分 2026-09-20] 发送出口/路由/编号换算迁往 Core/Src/app/app_tx.c ====
 * 原始出口（usb_send_raw/uart_send_raw）与 owner 编号换算本体已入 app_tx（HAL 直调
 * 收口到该模块）; 本文件仅留三个薄包装, 保持既有调用点不动（随 app_link 迁移收编）。
 * g_active_link 暂留于此（应答路由的"最近命令来源", 随 app_link 模块迁移）。 */
static void link_tx(const char *buf, int n)
{
    app_tx_send_by_link(g_active_link, buf, n);
}

/* [P2 双链路仲裁] 按 owner 链路发送 (仲裁广播 USB LOST / USB READY 专用 —
 * 与应答路由不同: 广播必须到达指挥权方, 而不是最后发命令的一方) */
static void link_owner_tx(const char *buf, int n)
{
    app_tx_send_to_owner(buf, n);
}

/* [P2 双链路仲裁] 调参会话激活判据 (会话锁注入, 设计文档 §2.3)
 * [app_control 拆分] 状态收编后判据本体在 app_control_session_active() */
static uint8_t arb_session_active(void)
{
    return app_control_session_active();
}

/* [P2 双链路仲裁] 广播出口注入 */
static void arb_broadcast(const char *msg)
{
    link_owner_tx(msg, (int)strlen(msg));
}

/* [P2/P3] 编号换算薄包装: 本体在 app_tx_owner_main_link()（唯一收口） */
static uint8_t owner_main_link(void)
{
    return app_tx_owner_main_link();
}

/* ==== [app_control 拆分] io 绑定 (组装层职责: 把宿主接到 HAL/显示/发送) ==== */
static void tx_raw(const char *buf, int n);      /* 定义于下方助手区 (前向声明) */
static void cmd_note(const char *fmt, ...);
static void ack(const char *fmt, ...);
static uint8_t ctl_usb_alive(void *ctx)
{
    (void)ctx;
    return (hUsbDeviceFS.dev_state == USBD_STATE_CONFIGURED &&
            hUsbDeviceFS.pClassData != NULL && g_usb_dtr) ? 1u : 0u;
}
static uint32_t ctl_last_rx_tick(void *ctx) { (void)ctx; return g_last_rx_tick; }
static int32_t  ctl_enc_count(uint8_t id, void *ctx)
{
    (void)ctx;
    return (id == 0) ? encoder_get_count(&henc1) : encoder_get_count(&henc2);
}
static void ctl_send_line(const char *s, void *ctx) { (void)ctx; tx_raw(s, (int)strlen(s)); }
static void ctl_note_cmd(const char *s, void *ctx)  { (void)ctx; cmd_note("%s", s); }
static void ctl_resp(const char *s, void *ctx)      { (void)ctx; ack("%s", s); }
static void ctl_delay_ms(uint32_t ms, void *ctx)    { (void)ctx; HAL_Delay(ms); }
static const char *disp_owner_str(void *ctx)        { (void)ctx; return app_tx_owner_str(); }
static uint8_t  disp_usb_on(void *ctx)              { (void)ctx; return app_tx_usb_on(); }
static uint16_t disp_rx_overflow(void *ctx)
{
    (void)ctx;
    /* [P3] 禁用链路的 ringbuf 无人消费, 溢出计数不计入 (与 P2 行为一致) */
    uint16_t ovf = (uint16_t)ringbuf_overflow(&g_ringbuf_uart);
#if LINK_USB_ENABLED
    ovf = (uint16_t)(ovf + ringbuf_overflow(&g_ringbuf_usb));
#endif
    return ovf;
}
static uint32_t disp_uart_silence_s(void *ctx)
{
    (void)ctx;
    if (!g_uart_seen) return 0xFFFFFFFFu;   /* 从未收到 (页7 显 "-") */
    return (HAL_GetTick() - g_last_uart_rx_tick) / 1000;
}
static uint8_t disp_gray_read(uint8_t idx, void *ctx)
{   /* [P2-4 收敛点] 页 3 灰度读经此注入, main.c 直读 HAL 收敛到唯一一处 */
    (void)ctx;
    static const uint32_t ports[5] = {
        (uint32_t)OUT1_GPIO_Port, (uint32_t)OUT2_GPIO_Port, (uint32_t)OUT3_GPIO_Port,
        (uint32_t)OUT4_GPIO_Port, (uint32_t)OUT5_GPIO_Port };
    static const uint16_t pins[5] = {
        OUT1_Pin, OUT2_Pin, OUT3_Pin, OUT4_Pin, OUT5_Pin };
    if (idx > 4) return 0;
    return (uint8_t)HAL_GPIO_ReadPin((GPIO_TypeDef *)ports[idx], pins[idx]);
}

/* 裸发送: 不记录 OLED 应答页 (寻迹诊断等高频事件用) */
static void tx_raw(const char *buf, int n)
{
    link_tx(buf, n);
}

/* 文本回应: 发送 + 记录到 OLED 页4 (指令接收后的回复, 无线调试确认) */
static void ack(const char *fmt, ...)
{
    char buf[80];
    va_list ap;
    va_start(ap, fmt);
    int n = vsnprintf(buf, sizeof(buf), fmt, ap);
    va_end(ap);
    if (n > 0) {
        tx_raw(buf, n);
        app_display_note_resp(buf, NULL);   /* [app_display 拆分] 页4 喂数 (截断在模块内) */
    }
}

/* 寻迹诊断代理：转发 line_follower 事件到串口 (高频, 不占用应答页) */
static void line_diag(const char *msg)
{
    char buf[64];
    int n = snprintf(buf, sizeof(buf), "%s\r\n", msg);
    if (n > 0) tx_raw(buf, n);
}

/* float → 1位小数的十分位数字 (替代 %.1f: newlib-nano 缺 _printf_float) */
static int dec1(float v)
{
    if (v < 0) v = -v;
    int d = (int)((v - (float)(int)v) * 10.0f + 0.5f);
    return (d > 9) ? 9 : d;
}

/* [app_control 拆分] fmt_f2 已随 ITEL 遥测迁往 app_control.c (原实现原样保留) */

/* ---- 转向执行组件 (steering) 配置 + 输出绑定 ----
 * 坐标系: 输出域绝对角(°) — 直行 = -90, 右满舵 = -115(下限), 左满舵 = -65(上限), 行程 ±25
 * 标定来源: SS2 实车手动探边 (2026-09-09), 详见 doc/舵机转向设计文档.md §2
 * 物理角 = 输出角 + SERVO_PHYS_OFFSET (135° 电气中位 = 前轮正前, 摆臂安装偏置)
 * ⚠️ 方向备忘: 指令正向(朝 -65) = 用户标"左"——摆臂偏装 ~90° 且方向与常规相反,
 *   SS3 转向模型统一符号; 直行位 ≠ 0 由实测决定, SV 0 会 clamp 到左满舵
 * 注: 标定值现已迁入 car_config.h「整车标定表」(2026-09-19 标定集中) */
static const steering_cfg_t g_steering_cfg = {
    .lim_min     = CAR_SERVO_LIM_MIN,
    .lim_max     = CAR_SERVO_LIM_MAX,
    .center      = CAR_SERVO_CENTER,
    .phys_offset = CAR_SERVO_PHYS_OFFSET,
    .lim_abs     = CAR_SERVO_LIM_ABS,
    .servo_id    = 0,
};

/* steering → 桥接层 输出注入
 * (组件不 include 桥接层头, 保证 steering.c 零依赖、PC 桩可编译 — 指南 §3.2) */
static void steering_output_to_servo(uint8_t id, float phys_angle)
{
    (void)servo_bridge_set_angle(id, phys_angle);   /* 显示/控制链不处理桥错误码（契约 §2.3） */
}

/* 记录最近执行的命令, OLED 页6 显示, 用于无线命令执行确认 */
static void cmd_note(const char *fmt, ...)
{
    char buf[24];
    va_list ap;
    va_start(ap, fmt);
    vsnprintf(buf, sizeof(buf), fmt, ap);
    va_end(ap);
    app_display_note_cmd(buf, NULL);   /* [app_display 拆分] 页6 喂数 (截断在模块内) */
}

/* [app_display 拆分] oled_line 已随页组版迁往 app_display.c */
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */
  /* [免拔插 USB 重枚举 — 试验结论 2026-09-19 ❌ 本板无效，已回退]
   * 试过在 MX_USB_DEVICE_Init() 之前把"复位后的 USB 断电态"显式保持 250ms
   * （思路：让主机确凿看到拔出事件 → 自动重枚举）。真机 3 次尝试均仍报
   * 错误 31（设备无响应）→ **固件侧无法制造断开事件**。推断本板 DP 上拉为
   * 外部固定 1.5k（不随外设断电态释放），或主机端口状态机必须由端口复位触发。
   * 结论：维持"物理拔插"，改用**流程自动化**规避（`tools/flash_flow.py`：
   * 烧录 → 提示拔插 → 自动等端口重现 → 自动 PING 验证）。
   * 若要根治，需硬件级方案：把 DP 上拉的 1.5k 改由 GPIO 控制（固件即可 detach）。
   * 详调试总结 §27。 */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_USART2_UART_Init();
  MX_TIM4_Init();
  MX_I2C2_Init();
  MX_USB_DEVICE_Init();
  /* USER CODE BEGIN 2 */
#ifdef USB_ECHO_TEST
    /* ---- USB 独立链路测试模式 (临时) ----
     * USB 已在上方 MX_USB_DEVICE_Init 完成; 此处不再 init 任何车功能
     * (电机/编码器/PWM/舵机/OLED/IMU/巡线/串口全不启动), 主循环空转,
     * 收发逻辑全部在 usbd_cdc_if.c 的 CDC_Receive_FS (中断上下文)。
     * 验证目标: 枚举成 COMx + PING->PONG + 回显完整性。 */
    for (;;) { HAL_Delay(1); }
#endif
    /* ---- UART (BT04 蓝牙串口) ----
     * PA2/PA3 现接 BT04, 透明传输波特率 9600, 覆盖 CubeMX 默认 115200
     * [P3 开关收口] CAR_FEATURE_UART=0 → 不重配 9600、不开中断接收、不发横幅;
     *   平台 ops 亦由 CMake 换成 no-op 空壳 → 该链路的硬件通道整体不存在 */
#if LINK_UART_ENABLED
    huart2.Init.BaudRate = 9600;
    if (HAL_UART_Init(&huart2) != HAL_OK) Error_Handler();
#endif
    ringbuf_init(&g_ringbuf_uart);
    ringbuf_init(&g_ringbuf_usb);   /* [P1 双链路] */

    /* ---- [app_tx 拆分 2026-09-20] 发送出口模块: 开关/超时经 cfg 注入 ----
     * 必须先于任何发送（含仲裁广播与 UART2 Ready 横幅） */
    {
        app_tx_cfg_t tx_cfg = {
            .uart_enabled         = LINK_UART_ENABLED,
            .usb_enabled          = LINK_USB_ENABLED,
            .usb_busy_timeout_ms  = 10,     /* 原直调口径 */
            .uart_block_timeout_ms = 100,
        };
        if (app_tx_init(&tx_cfg) != BRIDGE_OK) Error_Handler();
    }

    /* ---- [app_control 拆分 2026-09-20] 50ms 控制节拍宿主: io 绑定注入 ---- */
    {
        app_control_cfg_t ctl_cfg = {
            .ctrl_period_ms = CAR_CTRL_PERIOD_MS,
            .rec_max        = 128,               /* 原 REC_MAX */
            .imu_period_ms  = CAR_IMU_PERIOD_MS,
            .both_links     = LINK_BOTH_ENABLED,
            .wd_default_off = 1,                 /* WD 命令可开, 默认关 */
        };
        app_control_io_t ctl_io = {
            .usb_alive    = ctl_usb_alive,
            .last_rx_tick = ctl_last_rx_tick,
            .enc_count    = ctl_enc_count,
            .send_line    = ctl_send_line,
            .note_cmd     = ctl_note_cmd,
            .resp         = ctl_resp,
            .delay_ms     = ctl_delay_ms,
            .ctx          = NULL,
        };
        if (app_control_init(&ctl_cfg, &ctl_io) != BRIDGE_OK) Error_Handler();
    }

    /* ---- [app_display 拆分 2026-09-20] 100ms 显示节拍宿主: io 绑定注入 ---- */
    {
        app_display_cfg_t disp_cfg = {
            .disp_period_ms = CAR_DISP_PERIOD_MS,
            .usb_enabled    = LINK_USB_ENABLED,   /* P3: 下线时页 7 不宣称在线 */
        };
        app_display_io_t disp_io = {
            .owner_str     = disp_owner_str,
            .usb_on        = disp_usb_on,
            .rx_overflow   = disp_rx_overflow,
            .uart_silence_s = disp_uart_silence_s,
            .enc_count     = ctl_enc_count,       /* 复用 app_control 的编码器绑定 */
            .gray_read     = disp_gray_read,
            .ctx           = NULL,
        };
        if (app_display_init(&disp_cfg, &disp_io) != BRIDGE_OK) Error_Handler();
    }

    /* ---- [P2 双链路仲裁] 上电 usb_alive 初值 ----
     * 双链路: 未知(-1), 枚举异步完成, 由宽限期逻辑兜底 (超时不活 → 自动切 UART)
     * [P3] USB=0 → 直接判死(0) = 仲裁固定走 UART; UART=0 → 直接判活(1) = 固定走 USB。
     *      单链路模式下主循环不再喂判活 (LINK_BOTH_ENABLED 门控), 状态机保持初值。 */
    {
        LinkArbCfg ac = { .session_active = arb_session_active,
                          .broadcast      = arb_broadcast,
                          .boot_grace_ms  = 3000 };
#if !LINK_USB_ENABLED
        const int8_t alive_init = 0;    /* 无 USB 链路 → 固定 UART */
#elif !LINK_UART_ENABLED
        const int8_t alive_init = 1;    /* 无 UART 链路 → 固定 USB */
#else
        const int8_t alive_init = -1;   /* 未知, 待主循环喂判别活 */
#endif
        link_arb_init(&ac, alive_init, HAL_GetTick());
    }
#if LINK_UART_ENABLED
    /* [P1-1 补回该层 2026-09-20] 收发首挂/横幅改经策略层, 不再直调 HAL */
    (void)uart_receive_IT(&uart_debug, &uart_debug.rx_byte);
    (void)uart_send(&uart_debug, (const uint8_t *)"UART2 Ready\r\n", 13);
#endif

    /* ---- 编码器 ---- */
    henc1.htim  = &htim2;  henc1.ops = encoder_platform_get_ops();
    henc1.ppr   = CAR_PPR; henc1.position = 0;
    encoder_start(&henc1);
    henc2.htim  = &htim3;  henc2.ops = encoder_platform_get_ops();
    henc2.ppr   = CAR_PPR; henc2.position = 0;
    encoder_start(&henc2);

    /* ---- PWM 电机 20kHz 初始 0% ---- */
    pwm_set_freq(&pwm_tim1_ch1, CAR_MOTOR_PWM_HZ); pwm_set_duty_0E3(&pwm_tim1_ch1, 0); pwm_start(&pwm_tim1_ch1);
    pwm_set_freq(&pwm_tim1_ch2, CAR_MOTOR_PWM_HZ); pwm_set_duty_0E3(&pwm_tim1_ch2, 0); pwm_start(&pwm_tim1_ch2);

    /* ---- 电机桥（C1：配置结构注入 + init 失败即停）---- */
    if (motor_bridge_init(0, &g_motor_cfg[0]) != BRIDGE_OK ||
        motor_bridge_init(1, &g_motor_cfg[1]) != BRIDGE_OK)
        Error_Handler();

    /* ---- 速度环执行组件 (参数 = SIMC 辨识建议[平衡]档, 2026-09-13 阶跃辨识,
     *      A/B 实测验证: 上升 90% 150ms/零超调 vs 旧参 >3s; 详见 tools/step_ident.py) ----
     * ppr 一律以编码器实测值为准（单一数据源），标定表里的字面量只作缺省回退 */
    g_spd_cfg.ch[0].ppr = (float)henc1.ppr;
    g_spd_cfg.ch[1].ppr = (float)henc2.ppr;
    if (speed_loop_init(&g_spd_cfg, &g_spd_io) != SPEED_LOOP_OK)
        Error_Handler();

    /* ---- 舵机 50Hz ---- */
    pwm_set_freq(&pwm_tim4_ch3, CAR_SERVO_PWM_HZ); pwm_start(&pwm_tim4_ch3);

    /* ---- 舵机桥 (PB8) + 转向执行组件: 上电回直行位 = 安全铁律 ---- */
    if (servo_bridge_init(0, &g_servo_cfg) != BRIDGE_OK)
        Error_Handler();
    (void)servo_bridge_start(0);
    if (steering_init(&g_steering_cfg, steering_output_to_servo) != STEERING_OK)
        Error_Handler();
    steering_center();

    /* ---- OLED ---- */
    if (oled_bridge_init(&g_oled_cfg) != BRIDGE_OK)
        Error_Handler();
    oled_bridge_show_string_small(0,0,"BLUEPILL PID OK");
    oled_bridge_show_string_small(2,0,"Boot...");

    /* ---- 寻迹 ---- */
    line_follower_init(CAR_LF_BASE_SPD, CAR_LF_KP);
    line_follower_set_event_cb(line_diag);

    /* ---- IMU ---- */
    if (imu_bridge_init(0, &g_imu_cfg) != BRIDGE_OK)
        Error_Handler();

    /* ---- 里程计感知组件 (ΔX/ΔY/Δθ 增量位姿, 上位机 ODOM 上报数据源) ----
     * 依赖 imu_bridge 已 init（yaw 注入）。基准在主循环首拍由 update 自动建立。
     * （C1 故障注入验证记录 2026-09-20：临时注入 bad cfg(wheel_circ_mm=0) →
     *   ODOM_ERR_BAD_CFG → Error_Handler 停机，真机 4×PING 全程无 PONG——
     *   "init 失败即停"实锤，详调试总结 §22。验证后已还原此行。） */
    if (odom_init(&g_odom_cfg, &g_odom_io) != ODOM_OK)
        Error_Handler();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  /* ---- 主循环局部状态 (原在 USER CODE 外, 2026-09-19 regen 曾被 CubeMX 整段吞掉 — 教训: 主循环内容必须住 USER CODE 区) ---- */
  /* [P1 双链路] 解析状态按链路独立: 命令级交织消费要求两链路帧/行状态互不干扰
   * 注: txt/txt_len/b 在下方循环内声明 (2026-09-19 清理: 此处原有一份 regen 残留
   *     重复声明, 被循环内同名变量遮蔽, 属死声明) */
  uint8_t  frame[LINK_COUNT][32];
  uint8_t  f_len[LINK_COUNT] = {0, 0};
  uint8_t  in_frame[LINK_COUNT] = {0, 0};
  uint32_t t_frame[LINK_COUNT] = {0, 0};
  uint32_t t_auto = 0;    /* 巡线自动启动仲裁门 (100ms, 与显示同拍) */

  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
        /* ① 串口命令解析 — 一次读完 ringbuf 所有积压字节 */
        static uint8_t txt[LINK_COUNT][20];
        static uint8_t txt_len[LINK_COUNT];
        uint8_t b;
        /* [P1 双链路] 命令级交织消费: 每链路每轮最多 LINK_BYTE_BUDGET 字节
         * (时间片封顶, 见 doc/双链路仲裁设计文档.md §3.2); 执行完一条命令
         * 再回来读下一条, 双链路按行/帧粒度交错, 谁也不饿死谁 */
        for (uint8_t li = 0; li < LINK_COUNT; li++) {
          if (!g_link_on[li]) continue;   /* [P3 开关收口] 该链路编译期下线: 不消费 */
          uint8_t budget = LINK_BYTE_BUDGET;
          while (budget-- && ringbuf_read(g_link_rb[li], &b) == 0) {
            g_active_link = li;   /* 应答路由: 谁的命令回谁 (P2 由 link_arbiter 取代) */
            if (li == LINK_UART) { g_last_uart_rx_tick = HAL_GetTick(); g_uart_seen = 1; }
            if (li == owner_main_link()) {   /* [P2] WD 只盯 owner 链路 (设计文档 §6) */
                g_last_rx_tick = HAL_GetTick();   /* 看门狗: owner 链路下行字节算"活" */
                app_control_wd_rearm();           /* 解除触发闩锁 (状态在 app_control) */
            }
            if (b == 0xAA && !in_frame[li]) { f_len[li] = 0; in_frame[li] = 1; t_frame[li] = HAL_GetTick(); }
            if (in_frame[li]) {
                if (f_len[li] >= sizeof(frame[li])) { in_frame[li] = 0; continue; }
                frame[li][f_len[li]++] = b;
                t_frame[li] = HAL_GetTick();
                if (f_len[li] >= 2 && frame[li][f_len[li]-2] == 0xFF && frame[li][f_len[li]-1] == 0xFF) {
                    uint8_t cmd = frame[li][1], flen = f_len[li] - 2;
                    /* [P2 仲裁门控] 二进制帧: 非 owner 仅放行心跳 0xF0, 其余回 BUSY 拒绝 */
                    if (li != owner_main_link() && cmd != 0xF0) {
                        cmd_note("BUSY");
                        ack("BUSY:%s\r\n", (link_arb_owner() == LINK_ARB_USB) ? "USB" : "UART");
                    }
                    else if      (cmd == 0x01 && flen >= 7 && frame[li][2] < 2) { uint8_t id = frame[li][2]; float v; memcpy(&v,&frame[li][3],4); line_follower_enable(0); speed_loop_set_target(id, v); cmd_note("M%d=%dRPM", id, (int)((v>0)?v:-v)); ack("M%d:%dRPM\r\n",id,spd_to_int(v)); }
                    else if (cmd == 0x02 && flen >= 7 && frame[li][2] < 2) { uint8_t id = frame[li][2]; float v; memcpy(&v,&frame[li][3],4); line_follower_enable(0); (void)motor_bridge_set_speed_mps(id,v); cmd_note("M%d=%dcm/s", id, (int)(v*100)); ack("M%d:%dcm/s\r\n",id,spd_to_int(v*100.0f)); }
                    else if (cmd == 0x03 && flen >= 3 && frame[li][2] < 2) { uint8_t id = frame[li][2]; line_follower_enable(0); speed_loop_stop(id); cmd_note("BRK%d", id); ack("M%d:BRAKE\r\n",id); }
                    else if (cmd == 0x10 && flen >= 7 && frame[li][2] < 1) { float v; memcpy(&v,&frame[li][3],4); steering_set(v); float a = steering_get(); cmd_note("SV=%d.%d", (int)a, dec1(a)); ack("SV:%d.%d\r\n", (int)a, dec1(a)); }
                    else if (cmd == 0xE0 && flen >= 15 && frame[li][2] < 2) { uint8_t id = frame[li][2]; float kp,ki,kd; memcpy(&kp,&frame[li][3],4); memcpy(&ki,&frame[li][7],4); memcpy(&kd,&frame[li][11],4); speed_loop_set_gains(id,kp,ki,kd); cmd_note("PID%d", id); ack("OK\r\n"); }
                    else if (cmd == 0x20 && flen >= 7 && frame[li][2] < 2) { uint8_t id = frame[li][2]; float v; memcpy(&v,&frame[li][3],4); if (v >= -100.0f && v <= 100.0f) { line_follower_enable(0); (void)motor_bridge_set_rate_0E3(id, (int16_t)(v * 10.0f)); cmd_note("DUTY%d", id); ack("D%d:%d.%d\r\n", id, (int)v, dec1(v)); } else { cmd_note("DUTY BAD"); ack("?\r\n"); } }
                    else if (cmd == 0xF0) { cmd_note("PING->PONG"); ack("PONG\r\n"); }
                    else { cmd_note("ERR:%02X", cmd); ack("?\r\n"); }
                    in_frame[li] = 0; f_len[li] = 0;
                }
                continue;
            }
            if (b == '\r') continue;  /* 兼容手机APP "\r\n" 行尾: 否则 "PING\r" 匹配失败 */
            if (b == '\n') txt[li][txt_len[li]] = 0;
            else if (txt_len[li] < 19) { txt[li][txt_len[li]++] = b; continue; }
            if (txt_len[li] > 0) {
                /* 文本命令解析已模块化到 txt_cmd.c (PC 测试桩覆盖 41 用例),
                 * 手写数值解析替代 sscanf %f (newlib-nano 缺 _scanf_float 静默失败) */
                TxtCmd tc;
                if (txt_cmd_parse((char *)txt[li], &tc)) {
                    /* [P2 仲裁门控] 非 owner 链路仅放行 安全例外命令 (STOP/PING/LINK,
                     * 设计文档 §2.2); 其余回 BUSY:<owner> 拒绝 */
                    if (li != owner_main_link() &&
                        tc.type != TXTCMD_PING && tc.type != TXTCMD_STOP &&
                        tc.type != TXTCMD_LINK) {
                        cmd_note("BUSY");
                        ack("BUSY:%s\r\n",
                            (link_arb_owner() == LINK_ARB_USB) ? "USB" : "UART");
                    }
                    else switch (tc.type) {
                    case TXTCMD_PING:
                        cmd_note("PING->PONG");
                        ack("PONG\r\n");
                        break;
                    case TXTCMD_STOP:
                        for (int i = 0; i < 2; i++) speed_loop_stop(i);
                        /* 急停语义 = 彻底停: 必须同步关巡线+自动启动,
                         * 否则 50ms 后状态机重写目标, 车会"复活"(实测踩坑) */
                        line_follower_enable(0);
                        line_follower_set_auto(0);
                        steering_center();  /* 前轮回直行位 (实测 -90, 非 0) */
                        /* 调参工具链随急停关闭 (阶跃测试中途=安全终止, 遥测/记录回默认关)
                         * [app_control 拆分] 状态收编后经 stop_all 一口关闭 (看门狗保持武装) */
                        app_control_stop_all();
                        cmd_note("STOP ALL");
                        ack("STOP OK LINE OFF\r\n");
                        break;
                    case TXTCMD_MOTOR: {
                        int id = tc.i0;
                        float v = tc.f0;
                        line_follower_enable(0);  /* 手动指令优先: 退出巡线接管, 否则 50ms 后被覆盖 */
                        speed_loop_set_target(id, v);   /* 带符号目标: 方向由符号统一表达 */
                        cmd_note("M%d=%dRPM", id, (int)((v >= 0) ? v : -v));
                        ack("M%d:%dRPM\r\n", id, (int)(v + ((v < 0) ? -0.5f : 0.5f)));
                        break;
                    }
                    case TXTCMD_MOTOR_BOTH: {
                        float v = tc.f0;
                        line_follower_enable(0);  /* 手动指令优先 */
                        speed_loop_set_target(0, v);
                        speed_loop_set_target(1, v);
                        cmd_note("MS=%dRPM", (int)((v >= 0) ? v : -v));
                        ack("MS:%dRPM\r\n", (int)(v + ((v < 0) ? -0.5f : 0.5f)));
                        break;
                    }
                    case TXTCMD_BRAKE: {
                        int id = tc.i0;
                        line_follower_enable(0);  /* 手动指令优先 */
                        speed_loop_stop(id);
                        cmd_note("BRK%d", id);
                        ack("M%d:BRAKE\r\n", id);
                        break;
                    }
                    case TXTCMD_PID_SET: {
                        int lo = (tc.i0 < 0) ? 0 : tc.i0, hi = (tc.i0 < 0) ? 1 : tc.i0;
                        for (int i = lo; i <= hi; i++) {
                            speed_loop_set_gains(i, tc.i1*0.01f, tc.i2*0.01f, tc.i3*0.01f);
                        }
                        if (tc.i0 < 0) ack("PID ALL:%d %d %d\r\n", tc.i1, tc.i2, tc.i3);
                        else           ack("PID%d:%d %d %d\r\n", tc.i0, tc.i1, tc.i2, tc.i3);
                        cmd_note("PID SET");
                        break;
                    }
                    case TXTCMD_PID_SWAP: {
                        /* 增益真值从组件读（不再维护 ×100 镜像数组 → 消除"二进制改参后镜像失效"的漂移） */
                        float kp[2], ki[2], kd[2];
                        speed_loop_get_gains(0, &kp[0], &ki[0], &kd[0]);
                        speed_loop_get_gains(1, &kp[1], &ki[1], &kd[1]);
                        speed_loop_set_gains(0, kp[1], ki[1], kd[1]);
                        speed_loop_set_gains(1, kp[0], ki[0], kd[0]);
                        cmd_note("PID SWAP");
                        ack("SWAP:M0 %d %d %d  M1 %d %d %d\r\n",
                            spd_to_int(kp[1]*100.0f), spd_to_int(ki[1]*100.0f), spd_to_int(kd[1]*100.0f),
                            spd_to_int(kp[0]*100.0f), spd_to_int(ki[0]*100.0f), spd_to_int(kd[0]*100.0f));
                        break;
                    }
                    case TXTCMD_LINE_EN: {
                        int en = tc.i0 ? 1 : 0;
                        line_follower_enable(en);
                        if (!en) { speed_loop_set_target(0, 0.0f); speed_loop_set_target(1, 0.0f); }
                        cmd_note("LINE %s", en ? "ON" : "OFF");
                        ack("LINE:%s\r\n", en ? "ON" : "OFF");
                        break;
                    }
                    case TXTCMD_LINE_AUTO:
                        line_follower_set_auto(tc.i0 ? 1 : 0);
                        cmd_note("AUTO %d", tc.i0 ? 1 : 0);
                        ack("LA:%d\r\n", tc.i0 ? 1 : 0);
                        break;
                    case TXTCMD_GK:
                        line_follower_set_kp(tc.f0);
                        cmd_note("GK=%d.%d", (int)tc.f0, dec1(tc.f0));
                        ack("GK:%d.%d\r\n", (int)tc.f0, dec1(tc.f0));
                        break;
                    case TXTCMD_GD:
                        line_follower_set_kd(tc.f0);
                        cmd_note("GD=%d.%d", (int)tc.f0, dec1(tc.f0));
                        ack("GD:%d.%d\r\n", (int)tc.f0, dec1(tc.f0));
                        break;
                    case TXTCMD_GS:
                        line_follower_set_speed(tc.f0);
                        cmd_note("GS=%d.%d", (int)tc.f0, dec1(tc.f0));
                        ack("GS:%d.%d\r\n", (int)tc.f0, dec1(tc.f0));
                        break;
                    case TXTCMD_GI:
                        line_follower_invert();
                        cmd_note("GI");
                        ack("GI:%d\r\n", line_follower_inverted());
                        break;
                    case TXTCMD_GC:
                        line_follower_set_straight_cnt(tc.i0);
                        cmd_note("GC=%d", tc.i0);
                        ack("GC:%d\r\n", tc.i0);
                        break;
                    case TXTCMD_GT:
                        line_follower_set_turn_cnt(tc.i0);
                        cmd_note("GT=%d", tc.i0);
                        ack("GT:%d\r\n", tc.i0);
                        break;
                    case TXTCMD_PPR:
                        if (tc.i0 == 0) henc1.ppr = (uint16_t)tc.i1;
                        else            henc2.ppr = (uint16_t)tc.i1;
                        cmd_note("PPR%d=%d", tc.i0, tc.i1);
                        ack("PPR%d:%d\r\n", tc.i0, tc.i1);
                        break;
                    case TXTCMD_SERVO: {
                        steering_set(tc.f0);
                        float a = steering_get();            /* 回显实际生效角(钳位后) */
                        cmd_note("SV=%d.%d", (int)a, dec1(a));
                        ack("SV:%d.%d\r\n", (int)a, dec1(a));
                        break;
                    }
                    case TXTCMD_SERVO_NUDGE: {
                        steering_nudge((float)tc.i0);
                        float a = steering_get();
                        cmd_note("SV=%d.%d", (int)a, dec1(a));
                        ack("SV:%d.%d\r\n", (int)a, dec1(a));
                        break;
                    }
                    case TXTCMD_SERVO_LIM: {
                        /* 限位=配置态: 由 steering 组件唯一持有并归一化
                         * (下限<上限自动交换、限幅到 ±lim_abs、直行位拉回区间内) */
                        steering_ret_t r;
                        if      (tc.i0 == 0) r = steering_set_limit_min(tc.f0);
                        else if (tc.i0 == 1) r = steering_set_limit_max(tc.f0);
                        else                 r = steering_set_center(tc.f0);
                        /* 非法值: 组件已钳位保底, 此处回显请求值并标记 BAD (义务 6: 不静默) */
                        cmd_note("S%c=%d.%d%s", "LRC"[tc.i0], (int)tc.f0, dec1(tc.f0),
                                 (r == STEERING_OK) ? "" : "!");
                        ack("S%c:%d.%d%s\r\n", "LRC"[tc.i0], (int)tc.f0, dec1(tc.f0),
                            (r == STEERING_OK) ? "" : " BAD");
                        break;
                    }
                    /* ==== 调参工具链命令 (工程支持层, 可开关默认关, STOP 随时安全终止) ====
                     * [对未完善部分的假设] ① 接管权仲裁(C4)/桥接层契约(C1)尚未实施, 本工具暂按
                     *   "手动命令同模式" 取接管权: line_follower_enable(0) + speed_loop_stop()
                     *   (清目标 + 释放 PID + 物理刹停); 框架完善后应改为注册制接管, 对接点仅此一处;
                     * ② 开环输出经 motor_bridge_set_rate_0E3 直通(桥接层稳定边界), 千分比域
                     *   与 TB6612Protocol 一致, 换驱动芯片时仅桥内适配 */
                    case TXTCMD_STEP: {
                        /* [app_control 拆分] 校验/激活/安全前置全部在模块内 */
                        bridge_ret_t sr = app_control_step_start(
                            (uint8_t)tc.i0, (int16_t)tc.i1,
                            (uint32_t)tc.i2, (uint8_t)tc.i3, HAL_GetTick());
                        if (sr == BRIDGE_OK) {
                            cmd_note("STEP%d %d", tc.i0, tc.i1);
                            ack("STEP GO %d %d %d %d\r\n", tc.i0, tc.i1, tc.i2, tc.i3);
                        } else if (sr == BRIDGE_ERR_BUSY) {
                            ack("STEP BUSY\r\n"); cmd_note("STEP BUSY");
                        } else {
                            ack("STEP BAD\r\n"); cmd_note("STEP BAD");
                        }
                        break;
                    }
                    case TXTCMD_TEL:
                        app_control_tel_set((uint8_t)tc.i0);
                        cmd_note("TEL %d", tc.i0 ? 1 : 0);
                        ack("TEL:%d\r\n", tc.i0 ? 1 : 0);
                        break;
                    case TXTCMD_FF:
                        if (tc.i0 < 0 || tc.i0 > 500) { ack("FF BAD\r\n"); cmd_note("FF BAD"); break; }
                        for (int i = 0; i < 2; i++) speed_loop_set_ff_gain(i, tc.i0 / 100.0f);
                        cmd_note("FF %d", tc.i0);
                        ack("FF:%d.%02d\r\n", tc.i0 / 100, tc.i0 % 100);
                        break;
                    case TXTCMD_REC:
                        app_control_rec_set((uint8_t)tc.i0);
                        cmd_note("REC %d", tc.i0 ? 1 : 0);
                        ack("REC:%d\r\n", tc.i0 ? 1 : 0);
                        break;
                    case TXTCMD_DUMP:
                        /* [app_control 拆分] 重放输出/延时/收尾全部在模块内 */
                        app_control_rec_dump();
                        break;
                    case TXTCMD_ODOM:
                        app_control_odom_set((uint8_t)tc.i0);
                        cmd_note("ODOM %d", tc.i0 ? 1 : 0);
                        ack("ODOM:%d\r\n", tc.i0 ? 1 : 0);
                        break;
                    case TXTCMD_WD:
                        /* 范围 0~60000ms, 0=关 (校验在 app_control); 使能时刷新喂狗时刻 */
                        if (app_control_wd_set((uint32_t)tc.i0) != BRIDGE_OK) {
                            ack("WD BAD\r\n"); cmd_note("WD BAD");
                        } else {
                            g_last_rx_tick = HAL_GetTick();
                            cmd_note("WD %dms", tc.i0);
                            ack("WD:%d\r\n", tc.i0);
                        }
                        break;

                    /* ---- IMU 工具链 (解析域校验在 txt_cmd, 数值域在此; 执行端 = imu_bridge) ---- */
                    case TXTCMD_ITEL:
                        if (app_control_itel_set((uint8_t)tc.i0) != BRIDGE_OK) {
                            ack("ITEL BAD\r\n"); cmd_note("ITEL BAD");
                        } else {
                            cmd_note("ITEL %d", tc.i0);
                            ack("ITEL:%d\r\n", tc.i0);
                        }
                        break;
                    case TXTCMD_IGAIN:
                        /* Mahony 增益 ×100 (RAM 生效, 断电失); 允许 0 0 = 纯陀螺积分 (E2 实验) */
                        if (tc.i0 < 0 || tc.i1 < 0) { ack("IGAIN BAD\r\n"); cmd_note("IGAIN BAD"); break; }
                        (void)imu_bridge_set_mahony_gains(0, tc.i0 / 100.0f, tc.i1 / 100.0f);
                        cmd_note("IGAIN %d %d", tc.i0, tc.i1);
                        ack("IGAIN:%d %d\r\n", tc.i0, tc.i1);
                        break;
                    case TXTCMD_IDRIFT:
                        if (tc.i0 < 0 || tc.i0 > 1) { ack("IDRIFT BAD\r\n"); cmd_note("IDRIFT BAD"); break; }
                        (void)imu_bridge_set_drift_enable(0, (uint8_t)tc.i0);
                        cmd_note("IDRIFT %d", tc.i0);
                        ack("IDRIFT:%d\r\n", tc.i0);
                        break;
                    case TXTCMD_IRATE:
                        if (app_control_imu_rate_set((uint16_t)tc.i0) != BRIDGE_OK) {
                            ack("IRATE BAD\r\n"); cmd_note("IRATE BAD");
                        } else {
                            cmd_note("IRATE %d", tc.i0);
                            ack("IRATE:%d\r\n", tc.i0);
                        }
                        break;
                    case TXTCMD_ICAL:
                        /* 前置: 车体静止水平; 之后 50 拍(默认5s)校准, 完成后 yaw 归零 */
                        if (imu_bridge_recalibrate(0) != BRIDGE_OK) { ack("ICAL BAD\r\n"); cmd_note("ICAL BAD"); break; }
                        cmd_note("ICAL GO");
                        ack("ICAL:GO\r\n");
                        break;
                    case TXTCMD_LINK:
                        /* [P2 双链路仲裁] 查询/切换 — 任意链路可发 (安全例外, 设计文档 §2.2);
                         * 让出/接管/抢回三语义合一: 目标=对方即切换, 目标=自己即无操作 */
                        if (tc.i0 < 0) {
                            cmd_note("OWNER:%s", link_arb_owner() == LINK_ARB_USB ? "USB" : "UART");
                            ack("OWNER:%s\r\n", (link_arb_owner() == LINK_ARB_USB) ? "USB" : "UART");
                        } else {
#if LINK_BOTH_ENABLED
                            LinkArbRet r = link_arb_request(tc.i0 == 1 ? LINK_ARB_UART : LINK_ARB_USB);
                            if (r == LINK_ARB_OK) {
                                cmd_note("LINK:%s", tc.i0 == 1 ? "UART" : "USB");
                                ack("LINK:%s OK\r\n", (tc.i0 == 1) ? "UART" : "USB");
                            } else {       /* ERR_SESSION — 会话锁 */
                                cmd_note("BUSY SESSION");
                                ack("BUSY:SESSION\r\n");
                            }
#else
                            /* [P3 开关收口] 单链路模式: 目标链路编译期不存在 → 明确回 LINK:OFF
                             * (既不当成功也不无声无操作, 上位机必须能分辨"切不了") */
                            if (((tc.i0 == 1) ? LINK_UART : LINK_USB) == LINK_FALLBACK) {
                                ack("LINK:%s OK\r\n", tc.i0 == 1 ? "UART" : "USB");
                            } else {
                                cmd_note("LINK:OFF");
                                ack("LINK:OFF\r\n");
                            }
#endif
                        }
                        break;
                    default:
                        break;
                    }
                } else {
                    /* 识别失败的文本回 '?', 无线调试时确认"收到了但没看懂" */
                    cmd_note("ERR TXT");
                    ack("?\r\n");
                }
            }
            txt_len[li] = 0;
          }
        }

        /* 无线丢包保护(按链路): 二进制帧不完整超过 100ms 丢弃, 重新同步
         * (无线链路可能吞掉帧尾 0xFF 0xFF, 不复位会卡住后续解析) */
        for (uint8_t li = 0; li < LINK_COUNT; li++)
            if (in_frame[li] && (HAL_GetTick() - t_frame[li]) > 100) { in_frame[li] = 0; f_len[li] = 0; }

        /* ② + ③b → app_control (2026-09-20 拆分): 50ms 控制体（判活喂入/WD/STEP/
         * 闭环搬运/odom 组帧/TEL/REC）与 IMU 独立节拍（update_filter/ITEL）已整体
         * 迁往 Core/Src/app/app_control.c, 节拍门内聚在模块内, 此处只喂时间 */
        uint32_t now = HAL_GetTick();
        app_control_task(now);

        /* ③ 每 100ms 刷新显示 + 传感器
         * [OLED 分页轮转] 每 100ms 只刷 1 页 (8 页 800ms 轮完) — I2C2 异常时每页
         * ~10ms 超时, 8 页连刷曾阻塞主循环 ~850ms/圈 → 控制环掉到 1Hz (2026-09-13
         * 调参实验实测踩坑, 见调试总结 §14 优化方向); 分页后最坏阻塞 ≤1 页 */
        /* ③ → app_display (2026-09-20 拆分): 8 页组版/健康呈现/熔断呈现已迁往
         * Core/Src/app/app_display.c (节拍门内聚)。组装层仅保留"IMU 校准完成
         * 自动启动巡线"——这是接管权仲裁 (脑干职责), 不进显示模块 */
        if (now - t_auto >= CAR_DISP_PERIOD_MS) {
            t_auto = now;
            uint8_t prog = imu_bridge_cal_progress(0);
            line_follower_try_auto_start(prog >= 100, now);
        }
        app_display_task(now);
    }   /* while(1) 收尾 (原与显示门同行的右括号, 拆分后显式独立) */
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USB;
  PeriphClkInit.UsbClockSelection = RCC_USBCLKSOURCE_PLL_DIV1_5;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
