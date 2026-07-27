/**
 * ============================================================================
 *  RingBuffer — 环形缓冲区实现
 * ============================================================================
 *
 *  本文件实现 ringbuf.h 中声明的 4 个函数。
 *
 *  ▸ 核心算法 ◂
 *    · 写入前检查满：(head + 1) % SIZE == tail  → 丢弃
 *    · 读取前检查空： head == tail                → 返回 -1
 *    · 索引自回绕：  (idx ± 1) % SIZE 自动处理 head/tail 绕回
 *
 *  参考: UART_Serial_Design.md 第四章
 * ============================================================================
 */

#include "ringbuf.h"
#include <string.h>   /* memset */


/*==============================================================================
 *  ringbuf_init — 初始化，清零整个结构体
 *==============================================================================*/
void ringbuf_init(RingBuffer *rb)
{
    memset(rb, 0, sizeof(RingBuffer));
}


/*==============================================================================
 *  ringbuf_write — 写入 1 字节（中断中调用）
 *
 *  ① 算出 head 的下一步位置
 *  ② 下一步 == tail  → 满，丢弃新数据（return）
 *  ③ 否则写入 buf[head]，head 前进一步
 *
 *  复杂度：O(1)，< 1μs（在 180MHz STM32F4 上）
 *==============================================================================*/
void ringbuf_write(RingBuffer *rb, uint8_t byte)
{
    uint8_t next_head = (rb->head + 1) % RINGBUF_SIZE;

    if (next_head == rb->tail)               /* 满 — 丢弃新数据 */
        return;

    rb->buf[rb->head] = byte;                /* 写入 */
    rb->head           = next_head;          /* head 进位 */
}


/*==============================================================================
 *  ringbuf_read — 读取 1 字节（主循环中调用）
 *
 *  ① head == tail  → 空，返回 -1
 *  ② 否则取出 buf[tail]，tail 前进一步，返回 0
 *
 *  复杂度：O(1)
 *==============================================================================*/
int8_t ringbuf_read(RingBuffer *rb, uint8_t *byte)
{
    if (rb->head == rb->tail)                /* 空 */
        return -1;

    *byte    = rb->buf[rb->tail];            /* 读出 */
    rb->tail = (rb->tail + 1) % RINGBUF_SIZE; /* tail 进位 */
    return 0;
}


/*==============================================================================
 *  ringbuf_num_available — 获取未读字节数
 *
 *  公式：(head - tail + SIZE) % SIZE
 *        uint8_t 的无符号减法自动处理 head < tail 的情况
 *==============================================================================*/
uint16_t ringbuf_num_available(RingBuffer *rb)
{
    return (rb->head - rb->tail + RINGBUF_SIZE) % RINGBUF_SIZE;
}
