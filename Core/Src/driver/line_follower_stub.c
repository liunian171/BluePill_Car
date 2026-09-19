/**
 * ============================================================================
 *  line_follower_stub.c — 巡线功能的编译期空壳（功能开关的"另一侧"）
 * ============================================================================
 *
 *  层位：组件层占位实现（非业务组件）。与 line_follower.c 二选一参与编译，
 *  由 CMakeLists.txt 依据 Core/Inc/car_config.h 的 CAR_FEATURE_LINE_FOLLOWER
 *  宏取舍（空壳替换方案，详 doc/开发跟踪.md §设计决议 2026-09-19）。
 *
 *  ▸ 为什么存在 ◂
 *    巡线暂不使用（ROS 下位机模式主用），但组装层 main.c 有 12+ 处
 *    line_follower_enable(0)（接管权安全语义：手动命令必须退巡线接管）。
 *    若直接剔除源文件，这些调用点符号悬空编译不过；#ifdef 包裹则污染组装层。
 *    空壳方案：函数名全保留、全部 no-op → main.c 零修改，enable(0) 自然
 *    变无效操作，巡线永不接管，安全语义不变。
 *
 *  ▸ 语义承诺 ◂
 *    1) enabled() 恒返回 0 → 50ms 节拍永不把意图缓冲推给速度环
 *    2) update() 不写 spd_target/spd_dir（保持调用方缓冲原样）
 *    3) 全部 setter 为 no-op；全部 getter 返回安全零值
 *    4) 已知副作用：L/LA 命令应答仍会回显 "LINE:ON"（应答文本由组装层
 *       按入参拼装，不经组件）——真伪以 OLED 页 7 "LINE:OFF" 为准；
 *       如需命令级彻底禁用，属组装层 case 收窄，另行决策
 *
 *  ▸ 换平台/换器件时动不动这个文件 ◂
 *    不动。它是纯占位，无任何硬件知识；巡线功能恢复 = car_config.h 宏改 1
 *    + 重新 cmake configure，本文件自动退出编译。
 * ============================================================================
 */

#include "driver/line_follower.h"

/* ---- 生命周期：no-op（参数显式消费，避免 -Wunused-parameter）---- */

void line_follower_init(float base_spd, float kp)
{
    (void)base_spd;
    (void)kp;
}

void line_follower_update(uint32_t now_ms, float *spd_target, int8_t *spd_dir,
                          int32_t enc0, int32_t enc1)
{
    /* 空壳核心纪律：不写 spd_target / spd_dir，保持调用方缓冲原样 */
    (void)now_ms;
    (void)spd_target;
    (void)spd_dir;
    (void)enc0;
    (void)enc1;
}

void line_follower_try_auto_start(uint8_t cal_ok, uint32_t now_ms)
{
    (void)cal_ok;
    (void)now_ms;
}

/* ---- 运行时控制：全部 no-op ---- */

void line_follower_enable(uint8_t en)        { (void)en; }
void line_follower_set_auto(uint8_t en)      { (void)en; }
void line_follower_set_speed(float rpm)      { (void)rpm; }
void line_follower_set_turn_speed(float rpm) { (void)rpm; }
void line_follower_set_kp(float kp)          { (void)kp; }
void line_follower_set_kd(float kd)          { (void)kd; }
void line_follower_invert(void)              { }
void line_follower_set_straight_cnt(int32_t cnt) { (void)cnt; }
void line_follower_set_turn_cnt(int32_t cnt)     { (void)cnt; }
void line_follower_set_event_cb(line_event_cb_t cb) { (void)cb; }

/* ---- 查询接口：安全零值（enabled 恒 0 是空壳的语义核心）---- */

uint8_t line_follower_enabled(void)     { return 0; }
uint8_t line_follower_auto(void)        { return 0; }
float   line_follower_base_spd(void)    { return 0.0f; }
float   line_follower_turn_spd(void)    { return 0.0f; }
float   line_follower_kp(void)          { return 0.0f; }
float   line_follower_kd(void)          { return 0.0f; }
int8_t  line_follower_inverted(void)    { return 0; }
uint8_t line_follower_state(void)       { return 0; }
int8_t  line_follower_turn_dir(void)    { return 0; }
int32_t line_follower_straight_cnt(void){ return 0; }
int32_t line_follower_turn_cnt(void)    { return 0; }
