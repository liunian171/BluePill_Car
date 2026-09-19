/**
 * @file    link_arbiter.h
 * @brief   双链路指挥权仲裁 — 纯逻辑组件, 不依赖 HAL/硬件 (PC 测试桩可编译)
 *
 * 设计权威: doc/双链路仲裁设计文档.md §2 (状态机) §4 (判活)
 * 组件纪律: 依赖白名单仅 <stdint.h> (义务 7); 硬件知识全部经 cfg 函数指针注入。
 *
 * 状态机 (OWNER_USB 默认 / OWNER_UART 接管):
 *   - 显式切换: 任一链路发 LINK <target> (接管/让出/抢回三语义合一)
 *   - 自动 failover: usb_alive 1→0 边沿且 owner=USB → 切 UART + 广播 USB LOST
 *   - 恢复不抢回: usb_alive 0→1 边沿仅广播 USB READY, 需显式 LINK USB
 *   - 未知态宽限: 上电 usb_alive 未知(-1), 宽限期内不 failover, 超时仍不活 → 切 UART
 *   - 会话锁: 调参会话激活时拒绝切换 (STOP 安全例外在分发层, 不在本组件)
 */

#ifndef __LINK_ARBITER_H__
#define __LINK_ARBITER_H__

#include <stdint.h>

typedef enum {
    LINK_ARB_USB  = 0,      /* 与 main.c LINK_USB/LINK_UART 编号一致 */
    LINK_ARB_UART = 1,
} LinkArbOwner;

typedef enum {
    LINK_ARB_OK = 0,        /* 切换成功 (含目标==当前的无操作) */
    LINK_ARB_ERR_SESSION,   /* 调参会话激活, 切换被会话锁拒绝 */
    LINK_ARB_ERR_BAD_ARG,   /* 非法参数 */
} LinkArbRet;

typedef struct {
    /* 调参会话激活判据 (STEP/TEL/REC/ODOM/ITEL 任一激活 = 1) — 注入, 不可为 NULL */
    uint8_t (*session_active)(void);
    /* owner 链路广播 (USB LOST/USB READY 等) — 注入, 可为 NULL (静默) */
    void    (*broadcast)(const char *msg);
    uint32_t boot_grace_ms;     /* 上电 usb_alive 未知态宽限 (建议 3000) */
} LinkArbCfg;

/**
 * @brief  初始化仲裁器
 * @param  cfg            配置 + IO 注入
 * @param  usb_alive_init 上电瞬间 usb_alive: 1=活 0=死 -1=未知(枚举未完成)
 * @param  now            当前毫秒 tick (宽限期基准)
 */
void         link_arb_init(const LinkArbCfg *cfg, int8_t usb_alive_init, uint32_t now);

/**
 * @brief  显式切换指挥权 (LINK 命令执行端)
 * @param  target    目标链路
 * @return LINK_ARB_OK / LINK_ARB_ERR_SESSION
 */
LinkArbRet   link_arb_request(LinkArbOwner target);

/**
 * @brief  喂入 USB 判活状态 (每控制节拍一次; 内部做边沿检测)
 * @param  alive 1=活 0=死 -1=未知
 * @param  now   当前毫秒 tick
 * @return 1=本拍发生了 owner 切换 (failover), 0=无变化
 */
uint8_t      link_arb_notify(int8_t alive, uint32_t now);

/** @brief 当前 owner (分发层判 BUSY / OLED 显示 / WD 喂狗过滤) */
LinkArbOwner link_arb_owner(void);

#endif /* __LINK_ARBITER_H__ */
