/**
 * @file    encoder.c
 * @brief   编码器策略层 — 纯计数器读写
 */

#include "encoder.h"

void encoder_start(Encoder_Handle *henc)
{
    henc->ops->start(henc->htim);
}

int32_t encoder_get_count(Encoder_Handle *henc)
{
    henc->position = henc->ops->get_counter(henc->htim);
    return henc->position;
}

/* 2026-09-20 P1-6: encoder_stop / encoder_set_count / encoder_reset 判死删除
 * （§7 全仓零调用）。平台原语 ops->stop / ops->set_counter 保留（原子集合完整）。 */
