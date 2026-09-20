/**
 * ============================================================================
 *  app_iface.h — 组装层宿主模块的跨模块接口类型
 * ============================================================================
 *
 *  层位: 组装层（app_* 宿主模块之间唯一的类型共享点）。
 *  换平台/换器件: 不动（纯接口类型，零硬件知识）。
 *
 *  ▸ 规则 ◂
 *    - 这里只放"被 ≥2 个 app 模块使用"的函数指针结构体（接口带）。
 *      单模块私有的类型放各自 .h / .c。
 *    - 结构体 = C 版接口类: 由 app_wiring 在装配期填充并注入（交界不拥有）。
 *    - 全部函数指针约定: 不得为 NULL 后调用（init 契约校验; 参见桥接家族契约 §2.5）。
 * ============================================================================
 */

#ifndef APP_IFACE_H
#define APP_IFACE_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* 发送出口 — 应答/遥测/广播的唯一下行通道（app_tx 实现, app_link 消费）
 * 语义: 尽力而为; USB 非阻塞(BUSY 限时), UART 阻塞(见 app_tx.h 注释)。
 * buf 需 NUL 结尾与否由实现约定: 本工程按"发送 n 字节, 不要求 NUL"。 */
typedef struct {
    void (*send)(const char *buf, int n, void *ctx);
    void *ctx;                       /* 实现侧上下文, app_link 不解释 */
} app_tx_sink_t;

/* 显示喂数 — "最近命令 / 最近应答" (app_link 产生, app_display 呈现于页6/页4)
 * 语义: s 为 NUL 结尾字符串; 实现侧负责截断到显示宽度(≤19 字符 + NUL)。 */
typedef struct {
    void (*note_cmd)(const char *s, void *ctx);
    void (*note_resp)(const char *s, void *ctx);
    void *ctx;
} app_page_note_t;

#ifdef __cplusplus
}
#endif

#endif /* APP_IFACE_H */
