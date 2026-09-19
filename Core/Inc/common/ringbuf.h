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
 *
 *  ⚠️ **上限 255**：head/tail 为 uint8_t（单字节读写原子性好，中断/主循环天然无锁），
 *     故 RINGBUF_SIZE 不得取 256（回绕与"满"判据会失效）。
 *
 *  取值依据（2026-09-19 实测订正，原值 128 = 容量 127B 偏小）：
 *    · USB 侧：突发命令 ≥30 条（150B）即溢出丢应答，丢失量 ≈ (N−127)B 定量吻合；
 *              突发 60 条丢 31 条（一半）——实测见 doc/节拍拉长与I2C总线专案.md §5
 *    · UART 侧：9600 → 127B 仅 132ms 缓冲（原文档宣称 266ms）→ 240 给回 239ms
 */
#define RINGBUF_SIZE  240

/** @brief 环形缓冲区结构体
 *
 *  内存布局（RINGBUF_SIZE=240, 共 244 字节）：
 *    byte 0~239 : buf[240]       — 数据本体
 *    byte 240   : head (volatile)— 写索引（中断更新）
 *    byte 241   : tail           — 读索引（主循环更新）
 *    byte 242   : overflow       — 累计溢出丢字节数（饱和 255，中断写/主循环读）
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
    volatile uint8_t overflow;            /* 满丢弃计数 — 饱和 255（"丢过数据必须可见"） */
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

/** @brief 返回累计溢出丢弃字节数（饱和 255）—— 0 表示未丢过
 *  ▸ 用途：丢过数据必须可见（设计文档 §3.3 契约）；显示于 OLED 页 7 */
uint8_t  ringbuf_overflow(RingBuffer *rb);

#endif /* __RINGBUF_H__ */
