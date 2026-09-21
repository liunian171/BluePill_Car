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
 *  ② 下一步 == tail  → 满，**记一次溢出**并丢弃新数据（return）
 *  ③ 否则写入 buf[head]，head 前进一步
 *
 *  复杂度：O(1)，< 1μs（在 180MHz STM32F4 上）
 *  ⚠️ overflow 计数在中断中写、主循环中读：uint8_t 单字节读写原子，无需临界区；
 *     饱和在 255（诊断用途，只需回答"有没有丢、丢过多少量级"）
 *==============================================================================*/
void ringbuf_write(RingBuffer *rb, uint8_t byte)
{
    uint8_t next_head = (rb->head + 1) % RINGBUF_SIZE;

    if (next_head == rb->tail) {              /* 满 — 丢弃新数据 */
        if (rb->overflow < 255u) rb->overflow++;   /* 丢过必须可见 */
        return;
    }

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


/*==============================================================================
 *  ringbuf_overflow — 累计溢出丢弃字节数（饱和 255）
 *==============================================================================*/
uint8_t ringbuf_overflow(RingBuffer *rb)
{
    return rb->overflow;
}


/*==============================================================================
 *  ringbuf_peek — 窥视拷贝（不推进 tail）
 *  从当前 tail 起拷贝最多 max 个未读字节到 dst；可跨越缓冲末尾回绕。
 *  ⚠️ 调用方视角是"读"tail 区间：须保证本函数执行期间 tail 不被其它消费者推进
 *     （tx_stream 内与 ringbuf_commit 配对使用，均在任务上下文，安全）。
 *==============================================================================*/
uint16_t ringbuf_peek(RingBuffer *rb, uint8_t *dst, uint16_t max)
{
    uint16_t avail = (uint16_t)ringbuf_num_available(rb);
    if (max > avail) max = avail;
    uint16_t tail = rb->tail;
    for (uint16_t i = 0; i < max; i++) {
        dst[i] = rb->buf[tail];
        tail = (tail + 1) % RINGBUF_SIZE;
    }
    return max;
}


/*==============================================================================
 *  ringbuf_commit — 按 n 字节推进 tail（消费已窥视的数据）
 *  调用方必须保证 n ≤ ringbuf_num_available（否则 tail 越过 head，缓冲损坏）。
 *==============================================================================*/
void ringbuf_commit(RingBuffer *rb, uint16_t n)
{
    uint16_t avail = (uint16_t)ringbuf_num_available(rb);
    if (n > avail) n = avail;
    rb->tail = (rb->tail + n) % RINGBUF_SIZE;
}
