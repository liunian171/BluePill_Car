/**
 * ============================================================================
 *  app_tx.c — 发送出口与路由（组装层, HAL 直调唯一收口）
 * ============================================================================
 *
 *  层位: 组装层 app_* 宿主（本模块是组装层 HAL 直调的三个合法点之一）。
 *  搬迁自: main.c usb_send_raw / uart_send_raw / link_tx / link_owner_tx /
 *          owner_main_link（P1 双链路 + P2 仲裁 + P3 开关收口成果，语义不变）。
 *  换平台: 只替换本文件内的两个原始出口实现。
 *
 *  ▸ P3 链路开关语义 ◂
 *    uart_enabled / usb_enabled 来自 car_config.h（组装层 init 注入）。
 *    =0 的出口为运行时 no-op —— 与 CMake 的 stub 空壳替换、组装层消费门控
 *    共同构成"两层都关才算链路下线"。
 * ============================================================================
 */

#include "app/app_tx.h"
#include "driver/link_arbiter.h"
#include "usart.h"              /* huart2（CubeMX 生成） */
#include "usb_device.h"
#include "usbd_cdc_if.h"        /* CDC_Transmit_FS */
#include <string.h>

extern USBD_HandleTypeDef hUsbDeviceFS;   /* 定义在 usb_device.c */

static app_tx_cfg_t s_cfg;
static uint8_t      s_active_link = APP_LINK_USB;   /* 应答路由: 最近命令来源 */
static uint8_t      s_inited = 0;

/* ---- 两个原始出口（语义与原 main.c 静态函数逐行一致） ---- */

static void tx_usb(const char *buf, int n)
{
    if (!s_cfg.usb_enabled) return;                       /* [P3] 链路下线 no-op */
    if (hUsbDeviceFS.dev_state != USBD_STATE_CONFIGURED ||
        hUsbDeviceFS.pClassData == NULL)
        return;                                           /* 未枚举: 防空指针硬错误 */
    uint32_t t0 = HAL_GetTick();
    while (CDC_Transmit_FS((uint8_t *)buf, (uint16_t)n) == USBD_BUSY) {
        if ((HAL_GetTick() - t0) > s_cfg.usb_busy_timeout_ms) return;  /* 限时弃 */
    }
}

static void tx_uart(const char *buf, int n)
{
    if (!s_cfg.uart_enabled) return;                      /* [P3] 链路下线 no-op */
    (void)HAL_UART_Transmit(&huart2, (uint8_t *)buf, (uint16_t)n,
                            s_cfg.uart_block_timeout_ms);
}

/* ---- 公开接口 ---- */

bridge_ret_t app_tx_init(const app_tx_cfg_t *cfg)
{
    if (cfg == NULL) return BRIDGE_ERR_BAD_ARG;
    if (!cfg->uart_enabled && !cfg->usb_enabled) return BRIDGE_ERR_BAD_CFG; /* 双 0 无意义 */
    s_cfg = *cfg;
    s_active_link = cfg->usb_enabled ? APP_LINK_USB : APP_LINK_UART;
    s_inited = 1;
    return BRIDGE_OK;
}

void app_tx_send_by_link(uint8_t link, const char *buf, int n)
{
    if (!s_inited || buf == NULL || n <= 0) return;
    if (link == APP_LINK_USB) tx_usb(buf, n);
    else                      tx_uart(buf, n);
}

void app_tx_send_to_owner(const char *buf, int n)
{
    /* 仲裁广播必须到达指挥权方 —— 与应答路由(send_by_link)不同 */
    app_tx_send_by_link(app_tx_owner_main_link(), buf, n);
}

void app_tx_set_active(uint8_t link)
{
    s_active_link = (link == APP_LINK_USB) ? APP_LINK_USB : APP_LINK_UART;
}

uint8_t app_tx_owner_main_link(void)
{
    /* [P3 编号换算] main 消费循环 0=UART/1=USB, 仲裁枚举 0=USB/1=UART —
     * 两套编号相反, 换算在此唯一收口（2026-09-19 真机 bug 教训）。
     * 单链路模式直接返回唯一存活链路, 不依赖仲裁状态机（cfg 为编译期常量,
     * 分支可被优化; 门控本体仍在 CMake stub 取舍 + 组装层消费门控两处）。 */
    if (!s_cfg.usb_enabled)  return APP_LINK_UART;
    if (!s_cfg.uart_enabled) return APP_LINK_USB;
    return (link_arb_owner() == LINK_ARB_USB) ? APP_LINK_USB : APP_LINK_UART;
}

const char *app_tx_owner_str(void)
{
    return (link_arb_owner() == LINK_ARB_USB) ? "USB" : "UART";
}

uint8_t app_tx_usb_on(void)
{
    return (hUsbDeviceFS.dev_state == USBD_STATE_CONFIGURED &&
            hUsbDeviceFS.pClassData != NULL) ? 1u : 0u;
}
