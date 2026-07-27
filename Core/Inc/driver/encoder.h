#ifndef ENCODER_H
#define ENCODER_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct Encoder_PlatformOps_t
{
    void (*start)(void *htim);
    void (*stop)(void *htim);
    int32_t (*get_counter)(void *htim);
    void    (*set_counter)(void *htim, int32_t cnt);
} Encoder_PlatformOps_t;



typedef struct Encoder_Handle
{
    void *htim;  
        // 定时器句柄（隐藏具体芯片类型）
    Encoder_PlatformOps_t *ops;
    // Encoder_type_t type;           // 编码器类型（增量式/绝对式）,先不考虑其他编码器的
    uint32_t CH_A;               // 定时器通道号（1~4）
    uint32_t CH_B;               // 定时器通道号（1~4）
    uint16_t ppr;                // 每转脉冲数（上层 RPM 换算用）
    int32_t  position;           // 当前编码器位置（计数值）

} Encoder_Handle;


// void encoder_init(Encoder_Handle *hencoder, void *htim); //编写时创建对象

void encoder_start(Encoder_Handle *hencoder);

void encoder_stop(Encoder_Handle *hencoder);

void encoder_reset(Encoder_Handle *hencoder);

void encoder_set_count(Encoder_Handle *hencoder, int32_t count);

int32_t encoder_get_count(Encoder_Handle *hencoder);

#ifdef __cplusplus
}
#endif

Encoder_PlatformOps_t *encoder_platform_get_ops(void);

#endif // ENCODER