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
)

# ---- 功能编译开关（单一真值源 = Core/Inc/car_config.h，详开发跟踪 2026-09-19 设计决议）----
# 空壳替换方案：按宏取舍 line_follower.c / line_follower_stub.c，组装层 main.c 零修改。
# 注意：改宏后必须重新 cmake configure（本段在 configure 期执行），只 build 不生效。
set(CAR_CFG_FILE "${CMAKE_CURRENT_SOURCE_DIR}/Core/Inc/car_config.h")
set(CAR_FEATURE_LINE_FOLLOWER 1)   # 解析失败时的兜底默认（= 保留功能，安全侧）
if(EXISTS ${CAR_CFG_FILE})
    file(STRINGS ${CAR_CFG_FILE} _car_lf_line REGEX "^[ \t]*#define[ \t]+CAR_FEATURE_LINE_FOLLOWER[ \t]+[01]")
    if(_car_lf_line)
        string(REGEX REPLACE ".*CAR_FEATURE_LINE_FOLLOWER[ \t]+([01]).*" "\\1" _car_lf_val "${_car_lf_line}")
        set(CAR_FEATURE_LINE_FOLLOWER ${_car_lf_val})
    endif()
endif()
message(STATUS "car_config: CAR_FEATURE_LINE_FOLLOWER = ${CAR_FEATURE_LINE_FOLLOWER}")

list(FILTER USER_SOURCES EXCLUDE REGEX "line_follower(_stub)?\\.c$")
if(CAR_FEATURE_LINE_FOLLOWER)
    list(APPEND USER_SOURCES "${CMAKE_CURRENT_SOURCE_DIR}/Core/Src/driver/line_follower.c")
    message(STATUS "car_config: line follower -> FULL implementation (line_follower.c)")
else()
    list(APPEND USER_SOURCES "${CMAKE_CURRENT_SOURCE_DIR}/Core/Src/driver/line_follower_stub.c")
    message(STATUS "car_config: line follower -> STUB (feature compiled out)")
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
