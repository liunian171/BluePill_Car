#include "tool.h"

int map(int x, int in_min, int in_max, int out_min, int out_max)
    {
        if (x < in_min)
        {
            return out_min;
        }
        else if (x <= in_max && x >= in_min)
        {
            return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
        }
        else
        {
            return out_max;
        }
    }

// handle_to_id 已在 tool.h 中定义为 static inline