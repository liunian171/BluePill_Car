#ifndef TOOL_H
#define TOOL_H

#include <stdint.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

int map(int x, int in_min, int in_max, int out_min, int out_max);

#ifdef __cplusplus
}
#endif
//定点小数
#define DIV2(a)    ((a) >> 1)
#define DIV4(a)    ((a) >> 2)
#define DIV8(a)    ((a) >> 3)
#define DIV16(a)   ((a) >> 4)
#define DIV32(a)   ((a) >> 5)
#define DIV64(a)   ((a) >> 6)

#define MULT2(a)   ((a) << 1)
#define MULT4(a)   ((a) << 2)
#define MULT8(a)   ((a) << 3)
#define MULT16(a)  ((a) << 4)
#define MULT32(a)  ((a) << 5)
#define MULT64(a)  ((a) << 6)

/*==============================================================================
 *  handle_to_id — 通用句柄到序号映射
 *
 *  利用 .bss 段连续布局特性，通过地址差计算句柄数组中的序号。
 *  适用于 HAL 库中同类句柄连续排列的场景，O(1) 无分支。
 *
 *  用法:
 *    // 给定 huart1/huart6/huart7 在 .bss 中连续排列
 *    uint8_t id = handle_to_id(&huart1, sizeof(UART_HandleTypeDef), hal_huart);
 *
 *  注意:
 *    - 各 HAL 句柄必须在 .bss 中连续定义，无其他类型变量穿插
 *    - CubeMX 按外设添加顺序声明，通常满足此条件
 *    - 如布局不连续，返回的索引无意义
 *==============================================================================*/
static inline uint8_t handle_to_id(const void *base, size_t elem_size, const void *handle)
{
    if (!base || !handle || !elem_size) return 0;
    return (uint8_t)(((uintptr_t)handle - (uintptr_t)base) / elem_size);
}


#endif
