/**
 * @file    encoder.c
 * @brief   编码器策略层 — 纯计数器读写
 */

#include "encoder.h"

void encoder_start(Encoder_Handle *henc)
{
    henc->ops->start(henc->htim);
}

void encoder_stop(Encoder_Handle *henc)
{
    henc->ops->stop(henc->htim);
}

int32_t encoder_get_count(Encoder_Handle *henc)
{
    henc->position = henc->ops->get_counter(henc->htim);
    return henc->position;
}

void encoder_set_count(Encoder_Handle *henc, int32_t count)
{
    henc->ops->set_counter(henc->htim, count);
}

void encoder_reset(Encoder_Handle *henc)
{
    henc->ops->set_counter(henc->htim, 0);
}
