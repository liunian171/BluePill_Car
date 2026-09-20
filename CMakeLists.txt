cmake_minimum_required(VERSION 3.22)

#
# This file is generated only once,
# and is not re-generated if converter is called multiple times.
#
# User is free to modify the file as much as necessary
#

# Setup compiler settings
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS ON)


# Define the build type
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Debug")
endif()

# Set the project name
set(CMAKE_PROJECT_NAME BluePill_Car)

# Enable compile command to ease indexing with e.g. clangd
set(CMAKE_EXPORT_COMPILE_COMMANDS TRUE)

# Core project settings
project(${CMAKE_PROJECT_NAME})
message("Build type: " ${CMAKE_BUILD_TYPE})

# Enable CMake support for ASM, C and C++ languages
enable_language(C CXX ASM)

# Create an executable object type
add_executable(${CMAKE_PROJECT_NAME})

# Add STM32CubeMX generated sources
add_subdirectory(cmake/stm32cubemx)

# Link directories setup
target_link_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined library search paths
)

# reconfigure: add all driver sources
file(GLOB USER_SOURCES
    Core/Src/driver/*.c
    Core/Src/driver/*.cpp
    Core/Src/common/*.c
    Core/Src/common/*.cpp
    Core/Src/app/*.c
)

# ---- 功能编译开关（单一真值源 = Core/Inc/car_config.h，详开发跟踪 2026-09-19 设计决议）----
# 空壳替换方案：按宏取舍 *_stub.c 实现，组装层 main.c 尽量零修改。
# 注意：改宏后必须重新 cmake configure（本段在 configure 期执行），只 build 不生效。
set(CAR_CFG_FILE "${CMAKE_CURRENT_SOURCE_DIR}/Core/Inc/car_config.h")

# car_cfg_read(<MACRO> <fallback>) — 从 car_config.h 读 <MACRO> 的 1/0 值
# 兜底默认一律取 1（= 保留功能/保留链路，安全侧）；解析失败仅告警不静默
function(car_cfg_read _name _fallback)
    set(_val ${_fallback})
    if(EXISTS ${CAR_CFG_FILE})
        file(STRINGS ${CAR_CFG_FILE} _line REGEX "^[ \t]*#define[ \t]+${_name}[ \t]+[01]")
        if(_line)
            string(REGEX REPLACE ".*${_name}[ \t]+([01]).*" "\\1" _val "${_line}")
        else()
            message(WARNING "car_config: ${_name} 未在 car_config.h 中定义 → 用兜底值 ${_fallback}")
        endif()
    endif()
    set(${_name} ${_val} PARENT_SCOPE)
    message(STATUS "car_config: ${_name} = ${_val}")
endfunction()

car_cfg_read(CAR_FEATURE_LINE_FOLLOWER 1)
car_cfg_read(CAR_FEATURE_USB           1)
car_cfg_read(CAR_FEATURE_UART          1)

# 链路双 0 无意义（车彻底失联）→ configure 期直接拒绝，不留到运行期才发现
if(NOT CAR_FEATURE_USB AND NOT CAR_FEATURE_UART)
    message(FATAL_ERROR
        "car_config.h: CAR_FEATURE_USB 与 CAR_FEATURE_UART 不能同时为 0 —— 车将没有可用控制链路。"
        " 至少保留一条（ROS 下位机模式留 USB；无线独立模式留 UART）。")
endif()

# 巡线：空壳替换 line_follower.c / line_follower_stub.c
list(FILTER USER_SOURCES EXCLUDE REGEX "line_follower(_stub)?\\.c$")
if(CAR_FEATURE_LINE_FOLLOWER)
    list(APPEND USER_SOURCES "${CMAKE_CURRENT_SOURCE_DIR}/Core/Src/driver/line_follower.c")
    message(STATUS "car_config: line follower -> FULL implementation (line_follower.c)")
else()
    list(APPEND USER_SOURCES "${CMAKE_CURRENT_SOURCE_DIR}/Core/Src/driver/line_follower_stub.c")
    message(STATUS "car_config: line follower -> STUB (feature compiled out)")
endif()

# UART 链路：空壳替换 uart_platform_ops.c / uart_platform_ops_stub.c
# （=0 时平台 ops 三个函数变 no-op，符号名不变 → main.c 零修改）
list(FILTER USER_SOURCES EXCLUDE REGEX "uart_platform_ops(_stub)?\\.c$")
if(CAR_FEATURE_UART)
    list(APPEND USER_SOURCES "${CMAKE_CURRENT_SOURCE_DIR}/Core/Src/driver/uart_platform_ops.c")
    message(STATUS "car_config: UART link -> FULL (uart_platform_ops.c)")
else()
    list(APPEND USER_SOURCES "${CMAKE_CURRENT_SOURCE_DIR}/Core/Src/driver/uart_platform_ops_stub.c")
    message(STATUS "car_config: UART link -> STUB (uart platform ops no-op)")
endif()

target_sources(${CMAKE_PROJECT_NAME} PRIVATE ${USER_SOURCES})

# Add include paths
target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
    Core/Inc/driver
    Core/Inc/common
)

# Add project symbols (macros)
target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined symbols
)

# ---- USB 独立链路测试模式 (临时, 链路验证通过后移除) ----
# 用法: cmake --preset Release -DUSB_ECHO_TEST=1 (改宏后必须重新 configure)
# 效果: main.c 只 init USB + 空转, 车功能全不 init; CDC 收到 PING 回 PONG, 其余原样回显
if(USB_ECHO_TEST)
    target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE USB_ECHO_TEST)
    message(STATUS "USB_ECHO_TEST: ON (main.c 仅跑 USB 回显测试, 车功能不 init)")
endif()

# Remove wrong libob.a library dependency when using cpp files
list(REMOVE_ITEM CMAKE_C_IMPLICIT_LINK_LIBRARIES ob)

# Add linked libraries
target_link_libraries(${CMAKE_PROJECT_NAME}
    stm32cubemx
    stdc++          # C++ standard library

    # Add user defined libraries
)
