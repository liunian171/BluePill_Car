/**
 * ============================================================================
 *  RingBuffer — 环形缓冲区
 * ============================================================================
 *
 *  用途：中断（生产者）与主循环（消费者）之间的字节暂存区。
 *        解决"不知道数据何时来"的事件驱动问题和"主循环不能阻塞等待"的矛盾。
 *
 *  ▸ 设计要点 ◂
 *
 *    1. 数组实现 — 编译时固定大小，无 malloc，无内存碎片
 *    2. 单生产者单消费者 — head 只归中断写，tail 只归主循环读
 *       → 无需关中断、无需 spinlock，天然无锁安全
 *    3. volatile uint8_t head — 防编译器优化掉中断中的写入
 *    4. uint8_t 索引 — RINGBUF_SIZE ≤ 255 时省内存，单字节操作原子性好
 *    5. 满策略：丢弃新数据（保护已收到的旧数据不丢）
 *    6. 空/满区分：牺牲一个元素，head+1==tail 即满
 *
 *  ▸ 读写配合 ◂
 *
 *                   中断（生产者）                主循环（消费者）
 *              ┌──────────────────┐        ┌────────────────────┐
 *              │  字节到达        │        │   tick() 轮询      │
 *              │  ringbuf_write() │  ┌───┐ │  ringbuf_read()    │
 *              │  每次 < 1μs      │─→│buf│→│  有才取，慢点拿    │
 *              │  写完就走         │  └───┘ │  拼帧 → 分发       │
 *              └──────────────────┘        └────────────────────┘
 *
 *  参考: UART_Serial_Design.md 第四章
 * ============================================================================
 */

#ifndef __RINGBUF_H__
#define __RINGBUF_H__

#include <stdint.h>

/** @brief 环形缓冲区容量。
 *          uint8_t 索引下最大 255，当前 128 对 9600/115200 波特率够用。 */
#define RINGBUF_SIZE  128

/** @brief 环形缓冲区结构体
 *
 *  内存布局（RINGBUF_SIZE=128, 共 132 字节）：
 *    byte 0~127 : buf[128]      — 数据本体
 *    byte 128   : head (volatile)— 写索引（中断更新）
 *    byte 129   : tail           — 读索引（主循环更新）
 *
 *  状态判断：
 *    空：head == tail
 *    满：(head + 1) % RINGBUF_SIZE == tail
 *    可读字节数：(head - tail + RINGBUF_SIZE) % RINGBUF_SIZE
 */
typedef struct
{
    uint8_t          buf[RINGBUF_SIZE];   /* 数据缓冲区 */
    volatile uint8_t head;                /* 写索引 — 中断上下文更新 */
    uint8_t          tail;                /* 读索引 — 主循环上下文更新 */
} RingBuffer;


/*==============================================================================
 *  API
 *==============================================================================*/

/** @brief 初始化 — 将整个结构体清零 */
void     ringbuf_init(RingBuffer *rb);

/** @brief 写入 1 字节 — 仅中断中调用；满则丢弃新数据 */
void     ringbuf_write(RingBuffer *rb, uint8_t byte);

/** @brief 读取 1 字节 — 仅主循环调用；空返回 -1，有数据返回 0 */
int8_t   ringbuf_read(RingBuffer *rb, uint8_t *byte);

/** @brief 返回缓冲区中未读字节数 */
uint16_t ringbuf_num_available(RingBuffer *rb);

#endif /* __RINGBUF_H__ */
