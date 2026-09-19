#!/bin/bash
# P3 开关收口：car_config.h 链路开关编译矩阵（4 组合）
# 用法: bash .workbuddy/mx_matrix.sh
# 收尾强制恢复交付态 11，避免把头文件留在非默认状态。
set -u
cd "D:/666WorkSpace/STM32Cube/BluePill_Car" || exit 1
CFG="Core/Inc/car_config.h"
MAP="build/Release/BluePill_Car.map"
ELF="build/Release/BluePill_Car.elf"

set_macros() {   # $1=USB $2=UART
  sed -i -E "s/^#define CAR_FEATURE_USB +[01]/#define CAR_FEATURE_USB        $1/" "$CFG"
  sed -i -E "s/^#define CAR_FEATURE_UART +[01]/#define CAR_FEATURE_UART       $2/" "$CFG"
}

run_combo() {    # $1=USB $2=UART $3=标签
  set_macros "$1" "$2"
  echo "================ 组合 $1$2  ($3) ================"
  if ! cmake --preset Release > /tmp/cfg_$1$2.log 2>&1; then
      echo "  [configure 失败/被拒] —— 预期为双 0 组合"
      grep -E "CMake Error|car_config:" /tmp/cfg_$1$2.log | head -8
      return
  fi
  grep -E "car_config:" /tmp/cfg_$1$2.log | sed 's/^/  /'
  cmake --build build/Release > /tmp/bld_$1$2.log 2>&1
  local warn; warn=$(grep -c "warning:" /tmp/bld_$1$2.log)
  grep -E "RAM:|FLASH:" /tmp/bld_$1$2.log | sed 's/^/  /'
  echo "  告警数=$warn"
  # 证据：map 里实际参与链接的取舍文件
  echo -n "  map: line_follower.c.obj="; grep -c "line_follower\.c\.obj" "$MAP"
  echo -n "  map: uart_platform_ops.c.obj="; grep -c "uart_platform_ops\.c\.obj" "$MAP"
  echo -n "  map: uart_platform_ops_stub.c.obj="; grep -c "uart_platform_ops_stub\.c\.obj" "$MAP"
  echo -n "  elf 含 'LINK:OFF' 串(单链路专属)="
  if strings "$ELF" | grep -q "LINK:OFF"; then echo "YES"; else echo "NO"; fi
}

run_combo 1 1 "双链路并存 (交付态)"
run_combo 1 0 "纯 USB (不接 BT04)"
run_combo 0 1 "纯 UART/BT04"
run_combo 0 0 "预期被 CMake 拒绝"

echo "================ 恢复交付态 1 1 ================"
set_macros 1 1
cmake --preset Release > /dev/null 2>&1 && cmake --build build/Release > /tmp/bld_restore.log 2>&1
grep -E "RAM:|FLASH:" /tmp/bld_restore.log | sed 's/^/  /'
grep -nE "^#define CAR_FEATURE_(USB|UART)" "$CFG"
