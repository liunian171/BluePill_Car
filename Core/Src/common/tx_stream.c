/**
 * ============================================================================
 *  tx_stream — 发送流核心实现（纯逻辑，零 HAL）
 * ============================================================================
 *  实现 exact 见同名 .h 设计。核心不变量：段数据仅在 start()=OK 后才消费；
 *  RETRY/ERR 一律不弹出，保证数据不丢（"重试不重传"= 队内原样保留）。
 * ============================================================================
 */

#include "common/tx_stream.h"

void tx_stream_init(tx_stream_t *s)
{
    if (s == NULL) return;
    ringbuf_init(&s->q);
    s->dropped = 0;
    s->txerr   = 0;
    s->sent    = 0;
}

void tx_stream_bind(tx_stream_t *s, const tx_stream_io_t *io)
{
    if (s == NULL || io == NULL) return;
    s->io = *io;
}

/* 尝试推进一段。仅在 free()=可收 且 队非空 时预取并 start。 */
uint8_t tx_stream_service(tx_stream_t *s)
{
    if (s == NULL)                          return 0;
    if (s->io.start == NULL || s->io.free == NULL) return 0;
    if (!s->io.free(s->io.ctx))             return 0;   /* 链路忙：本次不推进 */
    uint16_t avail = (uint16_t)ringbuf_num_available(&s->q);
    if (avail == 0)                         return 0;   /* 队空 */

    uint16_t len = (avail > TX_SEG_MAX) ? TX_SEG_MAX : avail;
    ringbuf_peek(&s->q, s->seg, len);                    /* 拷到 scratch，不消费 */
    int r = s->io.start(s->seg, len, s->io.ctx);
    if (r == TX_START_OK) {
        ringbuf_commit(&s->q, len);                      /* 校验通过：弹出该段 */
        if (s->sent < 0x0FFFFFFFu) s->sent += len;
        return 1;
    }
    if (r == TX_START_ERR && s->txerr < 0xFFFFu) s->txerr++;
    return 0;   /* RETRY/ERR：段未消费，留待下次 */
}

uint16_t tx_stream_send(tx_stream_t *s, const uint8_t *buf, int n)
{
    if (s == NULL || buf == NULL || n <= 0) return 0;
    uint16_t accepted = 0;
    for (int i = 0; i < n; i++) {
        uint16_t before = (uint16_t)ringbuf_num_available(&s->q);
        ringbuf_write(&s->q, buf[i]);                    /* 队满 new 弃 + overflow++ */
        uint16_t after  = (uint16_t)ringbuf_num_available(&s->q);
        if (after > before) accepted++;
        else if (s->dropped < 0xFFFFu) s->dropped++;
    }
    (void)tx_stream_service(s);                          /* kick 第一段（尽力） */
    return accepted;
}

uint16_t tx_stream_pending(tx_stream_t *s) { return (s == NULL) ? 0u : (uint16_t)ringbuf_num_available(&s->q); }
uint16_t tx_stream_dropped(tx_stream_t *s){ return (s == NULL) ? 0u : s->dropped; }
uint16_t tx_stream_txerr  (tx_stream_t *s){ return (s == NULL) ? 0u : s->txerr; }
uint32_t tx_stream_sent    (tx_stream_t *s){ return (s == NULL) ? 0u : s->sent; }