/**
 * ============================================================================
 *  app_display.c — 100ms 显示节拍宿主（组装层）
 * ============================================================================
 *
 *  层位: 组装层 app_* 宿主。零 HAL（灰度/编码器/健康数据经 io 注入, 保 PC 桩）。
 *  职责: OLED 8 页单页轮转组版（定宽整行写, 禁裸清屏）+ 熔断呈现 + 页4/页6 喂数。
 *  搬迁自: main.c 100ms 显示块（页组版）+ cmd_note/ack 的 OLED 记录部分。
 *  换平台: 不动; 换屏: 显示链下层本来就是 oled_bridge 注入, 本模块无感。
 * ============================================================================
 */

#include "app/app_display.h"
#include "driver/oled_bridge.h"
#include "driver/imu_bridge.h"
#include "driver/speed_loop.h"
#include <string.h>
#include <stdio.h>
#include <stdarg.h>

static app_display_cfg_t s_cfg;
static app_display_io_t  s_io;
static uint8_t           s_inited = 0;

static uint32_t t_disp      = 0;
static uint8_t  s_disp_page = 0;

/* 页4/页6 数据源 (原 s_last_cmd/s_last_resp 收编) */
static char s_last_cmd[20]  = "NONE";
static char s_last_resp[20] = "-";

/* 带符号浮点 → 四舍五入 int (显示用; 与 app_control 同实现, 归拢待卫生批) */
static int disp_spd_to_int(float v)
{
    return (int)((v >= 0.0f) ? (v + 0.5f) : (v - 0.5f));
}

/* OLED 整行写入: 补齐/截断到 21 字符, 定宽覆盖无残留, 无需先清屏 (原 main.c oled_line) */
static void page_line(uint8_t page, const char *s)
{
    char buf[32];   /* 32B: 精度上限 21+1 恒 fits — 消除 -Wformat-truncation 误报 */
    snprintf(buf, sizeof(buf), "%-21.21s", s);
    oled_bridge_show_line_small(page, buf);
}

/* ---- 生命周期 ---- */

bridge_ret_t app_display_init(const app_display_cfg_t *cfg, const app_display_io_t *io)
{
    if (cfg == NULL || io == NULL) return BRIDGE_ERR_BAD_ARG;
    if (cfg->disp_period_ms == 0)  return BRIDGE_ERR_BAD_CFG;
    if (io->owner_str == NULL || io->usb_on == NULL || io->rx_overflow == NULL ||
        io->uart_silence_s == NULL || io->enc_count == NULL || io->gray_read == NULL)
        return BRIDGE_ERR_BAD_ARG;
    s_cfg = *cfg;
    s_io  = *io;
    s_inited = 1;
    return BRIDGE_OK;
}

void app_display_task(uint32_t now)
{
    if (!s_inited) return;
    if (now - t_disp < s_cfg.disp_period_ms) return;
    t_disp = now;

    /* 读传感器 (IMU 已在 app_control 独立节拍更新, 此处只取缓存值刷新显示) */
    float roll  = imu_bridge_get_roll(0);
    float pitch = imu_bridge_get_pitch(0);
    float yaw   = imu_bridge_get_yaw(0);
    int32_t enc0 = s_io.enc_count(0, s_io.ctx);
    int32_t enc1 = s_io.enc_count(1, s_io.ctx);

    int ri=(int)roll,  rd=(int)((roll -ri)*10.0f); if(rd<0)rd=-rd;
    int pi=(int)pitch, pd=(int)((pitch-pi)*10.0f); if(pd<0)pd=-pd;
    int yi=(int)yaw,   yd=(int)((yaw  -yi)*10.0f); if(yd<0)yd=-yd;
    uint8_t prog = imu_bridge_cal_progress(0);

    /* OLED 分页轮转: 单页单事务写入, 定宽补齐无残留, 免清屏 */
    char b[32];   /* 32B: 页7 链路行最长 ~29B (E/O 计数为多位时), 留足避免截断告警 */
    switch (s_disp_page) {
    case 0: /* IMU 欧拉角 */
        snprintf(b,26,"R:%d.%d P:%d.%d Y:%d.%d",ri,rd,pi,pd,yi,yd);
        page_line(0,b); break;
    case 1: { /* 速度环 实际->目标 (带符号, 由执行组件状态读出) */
        speed_loop_state_t s0, s1;
        speed_loop_get_state(0, &s0); speed_loop_get_state(1, &s1);
        snprintf(b,26,"M0:%d->%d  M1:%d->%d",
                 disp_spd_to_int(s0.actual_rpm), (int)s0.target_rpm,
                 disp_spd_to_int(s1.actual_rpm), (int)s1.target_rpm);
        page_line(1,b); break;
    }
    case 2: /* 编码器计数（标定转弯/直行距离用） */
        snprintf(b,26,"ENC0:%d  ENC1:%d", (int)enc0, (int)enc1);
        page_line(2,b); break;
    case 3: { /* 灰度 + IMU 状态
               * [P2-4 收敛点] 灰度读经 io 注入 (此前 main.c 与 line_follower 各自直调 HAL) */
        uint8_t g1=s_io.gray_read(0, s_io.ctx), g2=s_io.gray_read(1, s_io.ctx);
        uint8_t g3=s_io.gray_read(2, s_io.ctx), g4=s_io.gray_read(3, s_io.ctx);
        uint8_t g5=s_io.gray_read(4, s_io.ctx);
        snprintf(b,26,"G:%d%d%d%d%d  %s",
                 g1?1:0,g2?1:0,g3?1:0,g4?1:0,g5?1:0, (prog<100)?"CAL":"OK");
        page_line(3,b); break;
    }
    case 4: /* 最近应答 (指令接收后的回复, 无线调试确认) */
        snprintf(b,26,"RSP:%-17s", s_last_resp);
        page_line(4,b); break;
    case 5: { /* PID 参数 M0/M1 (合并一行); 真值从组件读, ×100 显示 */
        float kp[2], ki[2], kd[2];
        speed_loop_get_gains(0, &kp[0], &ki[0], &kd[0]);
        speed_loop_get_gains(1, &kp[1], &ki[1], &kd[1]);
        snprintf(b,26,"P%d,%d I%d,%d D%d,%d",
                 disp_spd_to_int(kp[0]*100.0f), disp_spd_to_int(kp[1]*100.0f),
                 disp_spd_to_int(ki[0]*100.0f), disp_spd_to_int(ki[1]*100.0f),
                 disp_spd_to_int(kd[0]*100.0f), disp_spd_to_int(kd[1]*100.0f));
        page_line(5,b); break;
    }
    case 6: /* 最近执行的命令 (无线命令执行确认) */
        snprintf(b,26,"CMD:%-16s", s_last_cmd);
        page_line(6,b); break;
    default: /* 页7: 链路状态 + 健康计数 (2026-09-19 节拍拉长专案: **失败必须可见**)
              * 字段: L=owner / USB:ON|OFF 或 E=OLED写失败累计 O=ringbuf溢出累计
              *       / 末列 = UART 静默秒数 或 FZ=OLED 已熔断停刷 */
    {
        /* USB: 枚举+配置完成即 ON; [P3] USB 链路下线时不宣称在线 (恒 OFF) */
        uint8_t usb_on = s_cfg.usb_enabled ? s_io.usb_on(s_io.ctx) : 0u;
        /* UART: MCU 无法感知 SPP 连接态, 显示距最近收字节的秒数 (从未收过显 "-") */
        char us[10];
        uint32_t sil = s_io.uart_silence_s(s_io.ctx);
        if (sil != 0xFFFFFFFFu)
            snprintf(us, sizeof(us), "%lus", (unsigned long)sil);
        else
            snprintf(us, sizeof(us), "-");

        uint16_t oled_e = oled_bridge_tx_fail_count();
        uint16_t rb_ovf = s_io.rx_overflow(s_io.ctx);
        if (oled_e || rb_ovf)     /* 有异常才挤掉 USB 状态字段, 正常态保持原版式 */
            snprintf(b,sizeof(b),"L:%s E%d O%d %s",
                     s_io.owner_str(s_io.ctx),
                     (int)oled_e, (int)rb_ovf,
                     oled_bridge_fused() ? "FZ" : us);
        else
            snprintf(b,sizeof(b),"L:%s USB:%s U:%s",
                     s_io.owner_str(s_io.ctx),
                     usb_on ? "ON" : "OFF", us);
        page_line(7,b); break;
    }
    }
    s_disp_page = (s_disp_page + 1) & 7;
}

/* ---- 页4/页6 喂数 (仅供 app_wiring 填进 app_page_note_t, 业务勿直调) ----
 * 截断到 19 字符 + NUL (显示宽度约定, 与原 main.c 手动截断等价) */

void app_display_note_cmd(const char *s, void *ctx)
{
    (void)ctx;
    if (s == NULL) return;
    size_t n = strlen(s);
    if (n > sizeof(s_last_cmd) - 1) n = sizeof(s_last_cmd) - 1;
    memcpy(s_last_cmd, s, n);
    s_last_cmd[n] = '\0';
}

void app_display_note_resp(const char *s, void *ctx)
{
    (void)ctx;
    if (s == NULL) return;
    size_t n = strlen(s);
    if (n > sizeof(s_last_resp) - 1) n = sizeof(s_last_resp) - 1;
    memcpy(s_last_resp, s, n);
    s_last_resp[n] = '\0';
}
