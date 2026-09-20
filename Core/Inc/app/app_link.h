/**
 * ============================================================================
 *  app_link.h — 命令链宿主（组装层）
 * ============================================================================
 *
 *  层位: 组装层 app_* 宿主。零 HAL（保 PC 桩）。
 *  职责: 双 ringbuf 消费(时间片封顶) + 指挥权门控 + 命令表执行 + 应答。
 *  换平台: 不动; 换协议: 改命令表。
 *
 *  吞掉的 main.c 现块: HAL_UART_RxCpltCallback 的业务部分 + 主循环 ① 命令解析段。
 *  关联: 指挥权语义权威 = doc/双链路仲裁设计文档.md §2; 本模块顺带兑现
 *        C4(接管权收敛为命令表 manual 属性) 与 P1-8(USB 收包注入式)。
 * ============================================================================
 */

#ifndef APP_LINK_H
#define APP_LINK_H

#include <stdint.h>
#include "driver/bridge_ret.h"
#include "app/app_iface.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ---- 配置（值由 app_wiring 从 car_config.h 翻译注入） ---- */
typedef struct {
    uint8_t  uart_enabled;        /* 1=链路接线存在 (CAR_FEATURE_UART) */
    uint8_t  usb_enabled;         /* 1=链路接线存在 (CAR_FEATURE_USB)  */
    uint8_t  byte_budget;         /* 每链路每轮 task 最多消费字节数 (现 64, §4.4) */
    uint32_t frame_timeout_ms;    /* 二进制帧不完整丢弃阈值 (现 100ms) */
} app_link_cfg_t;

/* ---- 依赖注入（全部由 app_wiring 填充; 指针不得为 NULL） ---- */
typedef struct {
    app_tx_sink_t    sink;            /* 应答/遥测发送出口 (app_tx) */
    app_page_note_t  page_note;       /* 页6/页4 喂数 (app_display) */
    void (*set_active)(uint8_t link, void *ctx);   /* 命令来源链路登记 (app_tx) */
    uint8_t (*owner_link)(void *ctx);              /* 当前指挥权(换算后) (app_tx) */
    void (*override_manual)(void *ctx);            /* 接管权声明 (wiring 绑 line_follower_enable(0)) */
    void (*wd_feed)(void *ctx);                    /* owner 链路字节喂狗 (app_control) */
    uint8_t (*session_active)(void *ctx);          /* 调参会话判据 (app_control) */
    void *ctx;
} app_link_io_t;

/* ---- 生命周期 ----
 * init: 校验 cfg/io 全部指针非空、budget>0, 否则 BAD_ARG; 双链路同 0 → BAD_CFG
 *       (CMake configure 期已拒, 此为运行期兜底)。 */
bridge_ret_t app_link_init(const app_link_cfg_t *cfg, const app_link_io_t *io);

/* 节拍入口: 消费双 ringbuf + 门控 + 命令表执行。
 * now = ms 时间基准(调用方传入, 义务 5)。裸机与 RTOS 同形。 */
void app_link_task(uint32_t now);

/* ---- ISR 注入（中断上下文安全: 只写 ringbuf + 溢出计数, 不解析） ---- */
void app_link_uart_rx_isr(uint8_t byte);   /* 装载层 HAL_UART_RxCpltCallback 调用 */
void app_link_usb_rx_isr(uint8_t byte);    /* usbd_cdc_if.c 经注册 sink 调用 (P1-8) */

/* ---- 只读查询（OLED 页7 数据源; 未 init 返回安全默认值 0） ---- */
uint16_t app_link_rx_overflow(void);       /* 双 ringbuf 溢出计数之和 */

#ifdef __cplusplus
}
#endif

#endif /* APP_LINK_H */
