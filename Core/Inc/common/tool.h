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

#endif
