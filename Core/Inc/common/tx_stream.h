/**
 * ============================================================================
 *  tx_stream — 发送流核心（纯逻辑，零 HAL）【R3 新增 2026-09-21】
 * ============================================================================
 *
 *  层位: 公共层 common/。零硬件依赖（时间基准/收发全部注入），保 PC 桩可测
 *        （对齐 cmd_exec / txt_cmd / ringbuf 模式）。
 *
 *  职责: UART 与 USB 两条链路统一的分段发送状态机 ——
 *        ▸ 字节队（复用 RingBuffer，SPSC：任务产 / 本模块消）
 *        ▸ 分段预取：把 ≤TX_SEG_MAX 字节 peek 进 seg，`start()` 返回
 *          TX_START_OK 才 commit（弹出）；RETRY/ERR 不消费 → 数据不丢。
 *        ▸ 推进链：`send()` 入队即 kick；任务周期调 `service()`（可 while-drain）。
 *
 *  注入面（平台只实现两个原子原语，见 app_tx.c）：
 *        free(ctx) : 链路是否可收新段。UART=`huart2.gState==READY`；USB=`已枚举&&CDC_IsTxFree`。
 *        start(seg,len,ctx) : 发起一段异步发送，返回 TX_START_OK/RETRY/ERR。
 *
 *  ▸ 非阻塞语义 ◂
 *        - UART：`HAL_Transmit_IT` 置 gState=BUSY_TX → 下轮 free()=false 自限速续推。
 *        - USB：`CDC_Transmit_FS` 置 TxState=1 → 同理，IN 完成后续推，不落地丢弃。
 *        - 队满才丢弃（dropped 计数可见）——"沉默失败必可见"铁律。
 *
 *  统计: dropped（入队丢弃字节）/ txerr（start 失败）/ sent（累计字节），饱和计数。
 * ============================================================================
 */

#ifndef TX_STREAM_H
#define TX_STREAM_H

#include <stdint.h>
#include <stddef.h>          /* NULL */
#include "common/ringbuf.h"

#ifdef __cplusplus
extern "C" {
#endif

/* 单段最大字节（scratch 大小）。UART IT 一段 ≤ ~50ms@9600; USB 一个 chunk。 */
#define TX_SEG_MAX  48

/* start() 返回码 */
#define TX_START_OK     0   /* 已接受：段数据交平台，队列可 commit */
#define TX_START_RETRY  1   /* 链路忙：段未消费，留待下次 service() */
#define TX_START_ERR    2   /* 发送失败：段未消费，计数 txerr，留待下次 */

/* 平台注入面 */
typedef struct {
    int  (*start)(const uint8_t *seg, uint16_t len, void *ctx);
    int  (*free)(void *ctx);       /* 1=可收新段 */
    void *ctx;
} tx_stream_io_t;

typedef struct {
    RingBuffer   q;                  /* 字节队（RINGBUF_SIZE 容量） */
    uint8_t      seg[TX_SEG_MAX];    /* 分段 scratch（free()=1 前不被覆写） */
    uint16_t     dropped;            /* 入队丢弃字节（饱和） */
    uint16_t     txerr;              /* start 失败计数（饱和） */
    uint32_t     sent;               /* 已启动发送字节累计 */
    tx_stream_io_t io;
} tx_stream_t;

/* 初始化：清零 + 队初始化。 */
void     tx_stream_init(tx_stream_t *s);
/* 注入平台两原语（app_wiring/app_tx 装配期调用，须在首次 send 前） */
void     tx_stream_bind(tx_stream_t *s, const tx_stream_io_t *io);

/* 入队 n 字节 → 返回实际接受字节数（队满丢弃并计 dropped）；内部 kick 一段。 */
uint16_t tx_stream_send(tx_stream_t *s, const uint8_t *buf, int n);

/* 尝试推进一段：free()=可收 && 队非空 才发生。返回 1=已消耗一段（可 while-drain）。 */
uint8_t  tx_stream_service(tx_stream_t *s);

/* 统计 / 状态 */
uint16_t tx_stream_pending(tx_stream_t *s);   /* 队内未发字节 */
uint16_t tx_stream_dropped(tx_stream_t *s);
uint16_t tx_stream_txerr(tx_stream_t *s);
uint32_t tx_stream_sent(tx_stream_t *s);

#ifdef __cplusplus
}
#endif

#endif /* TX_STREAM_H */