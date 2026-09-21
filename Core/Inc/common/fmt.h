/**
 * ============================================================================
 *  fmt.h — 带符号浮点 → 整数格式化助手（串口/显示/遥测共用, 纯逻辑零 HAL）
 * ============================================================================
 *
 *  归拢自: app/cmd_exec.c（spd_to_int/dec1）、app/app_control.c（fmt_f2）、
 *          app/app_display.c（disp_spd_to_int）—— 原四处本地 static 重复实现，
 *          此卫生批统一收拢到 common 层（仿 math_fast.h/.c 形态）。
 *
 *  语义承诺: 三个函数与搬迁前**逐行为等价**（实现原样搬移, 零行为变更）。
 *  ⚠️ 禁 %f: nano.specs 不含浮点 printf, 一律整数拆分 %d.%d（AGENTS §5.2-2）。
 * ============================================================================
 */

#ifndef FMT_H
#define FMT_H

#ifdef __cplusplus
extern "C" {
#endif

/* 带符号浮点 → 四舍五入 int（原 cmd_exec/app_control 的 spd_to_int）:
 *   (int)((v >= 0.0f) ? (v + 0.5f) : (v - 0.5f)) */
int fmt_spd_to_int(float v);

/* float → 十分位个位数（负数先取绝对值; 进位饱和 9）—— 原 cmd_exec 的 dec1 */
int fmt_dec1(float v);

/* float → "%s%d.%02d"（负号/整数/两位小数进位; 返回 snprintf 返回值）—— 原 app_control 的 fmt_f2 */
int fmt_f2(char *out, int n, float v);

#ifdef __cplusplus
}
#endif

#endif /* FMT_H */