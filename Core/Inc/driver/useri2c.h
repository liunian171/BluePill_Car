/**
 * ============================================================================
 *  I2C 驱动 — 抽象层（非阻塞版，支持软件模拟与硬件外设）
 * ============================================================================
 *
 *  架构对标 pwm.h / uart.h，采用 ops 表模式：
 *
 *   ┌─ useri2c.c ──────────────────────────────────────────────┐
 *   │  策略层（跨平台通用）                                       │
 *   │  职责：组装事务参数 → 调 ops->start_transfer → 等回调     │
 *   │  不碰 GPIO、不碰定时器                                     │
 *   └──────────────────────┬────────────────────────────────────┘
 *                          │ hI2C->ops->start_transfer() / is_busy()
 *                          ▼
 *   ┌─ i2c_software_ops.c ────────────────────────────────────┐
 *   │  软件 I2C 平台实现（双层 FSM + 定时器 ISR）               │
 *   │  职责：start_transfer 填入参数 → 开定时器 → CPU 立即返回 │
 *   │        ISR 逐 tick 翻 GPIO → 传完调 complete_callback   │
 *   │  GPIO 基于 usergpio，换平台只换 usergpio_ops             │
 *   └──────────────────────────────────────────────────────────┘
 *
 *  ▸ ops 是事务级接口（start_transfer + is_busy），不暴露 GPIO ◂
 *  ▸ 阻塞版 send_byte / recv_byte 变为 FSM 内部步骤 ◂
 */

#ifndef __USERI2C_H
#define __USERI2C_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include "usergpio.h"

// ================================================================
//  回调类型
// ================================================================

/**
 * @brief 传输完成回调
 *
 * 在 ISR 末尾（关定时器之后、回到主循环之前）调用。
 *
 * @param i2c_context  I2C_SoftwareContext 指针
 * @param result       0=成功, -1=NACK, -2=总线错误……
 */
typedef void (*I2C_TransferCallback)(void *i2c_context, int8_t result);

// ================================================================
//  FSM 状态枚举
// ================================================================

/**
 * @brief 微状态 —— 决定每次 ISR 走哪个分支
 *
 * 每个数据位 = PREPARE（准备数据）+ SAMPLE（采样/锁存），反复交替。
 */
typedef enum {
    I2C_MICRO_PREPARE = 0,   ///< 准备阶段：时钟拉低，设数据到信号线
    I2C_MICRO_SAMPLE  = 1,   ///< 采样阶段：时钟拉高，从机读取（发）/ 主机读取（收）
    I2C_MICRO_IDLE    = 2    ///< 空闲，定时器未启动
} I2C_MicroState;

/**
 * @brief 宏状态 —— 事务进行到哪个阶段
 */
typedef enum {
    I2C_MACRO_IDLE  = 0,     ///< 无传输，定时器已关
    I2C_MACRO_START = 1,     ///< 发出 START 信号（走 special_step 子 FSM）
    I2C_MACRO_SEND  = 2,     ///< 发送数据字节（微/宏 FSM 协作）
    I2C_MACRO_RECV  = 3,     ///< 接收数据字节（微/宏 FSM 协作）
    I2C_MACRO_STOP  = 4,     ///< 发出 STOP 信号（走 special_step 子 FSM）
    I2C_MACRO_DONE  = 5      ///< 传输逻辑结束，等待 complete_callback 后回到 IDLE
} I2C_MacroState;

// ================================================================
//  软件 I2C 状态机上下文（对标 TIM_HandleTypeDef）
// ================================================================

/**
 * @brief 软件 I2C 状态机上下文
 *
 * 同时承载硬件资源（GPIO + 定时器）和 FSM 运行时状态，
 * 挂在 I2C_Handle.i2c_context 上（仅软件实现使用）。
 */
typedef struct I2C_SoftwareContext {
    /* ---- 硬件资源（start_transfer 时填入，运行时只读） ---- */
    UserGPIO_Handle   scl_pin;        ///< SCL 引脚句柄，原子层 usergpio_write 操作
    UserGPIO_Handle   sda_pin;        ///< SDA 引脚句柄，原子层 usergpio_write/read 操作
    void             *timer_handle;   ///< 定时器句柄（STM32: TIM_HandleTypeDef*），timer_start/stop 使用

    /* ---- FSM 状态（运行时持续变化，决定每次 ISR 走哪个分支） ---- */
    I2C_MicroState    micro_state;    ///< fsm_bit 读写：PREPARE(0)↔SAMPLE(1) 交替，每 4μs 切换
    I2C_MacroState    macro_state;    ///< fsm_byte/fsm_edge 写，timer_isr 读：IDLE→START→SEND→STOP→DONE→IDLE
    uint8_t           edge_step;      ///< fsm_edge 自管：START/STOP 各 3 tick，0→1→2 推进

    /* ---- 事务参数（start_transfer 填入，fsm_byte 读取做决策） ---- */
    uint8_t           device_address; ///< 从设备 7 位地址（不含 R/W），fsm_byte 用 <<1|0 或 <<1|1 拼接
    uint8_t           register_address;///< 寄存器地址，地址字节后的下一个 send_byte
    uint8_t           is_read;        ///< 0=写事务, 1=读事务。读事务时 fsm_byte 发完寄存器地址后自动切 RECV
    uint8_t          *data_buffer;    ///< 数据缓冲区（写：只读 / 读：fsm_byte 写回）
    uint16_t          data_length;    ///< 数据字节数，byte_index>=data_length+2 时全部发完
    uint16_t          byte_index;     ///< 当前字节序号（fsm_byte 管理）：0=设备地址, 1=寄存器地址, 2+=数据字节

    /* ---- 当前字节上下文（fsm_bit 和 fsm_byte 协作的共享"黑板"） ---- */
    uint8_t           send_byte;      ///< 待发字节（fsm_byte 装入，fsm_bit 取 bit_counter 位推到 SDA）
    uint8_t           receive_byte;   ///< 正在接收的字节（fsm_bit 左移|存入，fsm_byte 写回 data_buffer）
    int8_t            bit_counter;    ///< 位计数（fsm_byte 唯一写）：7→0 数据位，0→-1 进 ACK，-1→7 装下一字节

    /* ---- 异步通知 ---- */
    I2C_TransferCallback complete_callback;  ///< fsm_edge 在 STOP 完成后调用：result=0 成功，负数失败
    int8_t            transfer_result;       ///< 传输结果：start_transfer 初始化为 0，fsm_byte 发现 NACK 时置 -1，fsm_edge 读取并回传
} I2C_SoftwareContext;

// ================================================================
//  平台操作表（事务级接口）
// ================================================================

/**
 * @brief I2C 平台操作表
 *
 * 对标 PWM_PlatformOps_t / UART_PlatformOps_t 的 void* 模式：
 * 所有函数第一个参数是 I2C_Handle.i2c_context（即 I2C_SoftwareContext* 或 I2C_HandleTypeDef*）。
 *
 * 软件实现：start_transfer 填参数 + 开定时器 ISR，全程非阻塞
 * 硬件实现：start_transfer 调 HAL_I2C_Mem_Write/Read_IT
 */
typedef struct I2C_PlatformOps {
    /**
     * @brief 启动一次非阻塞 I2C 传输
     *
     * @param i2c_context    平台资源指针（I2C_SoftwareContext* 或 I2C_HandleTypeDef*）
     * @param device_address 从设备 7 位地址
     * @param register_addr  寄存器地址
     * @param is_read        0=写, 1=读
     * @param buffer         数据缓冲区
     * @param length         字节数
     * @param callback       传输完成回调（ISR 末尾调用）
     * @retval 0  启动成功
     * @retval -1 句柄忙
     */
    int8_t (*start_transfer)(void *i2c_context,
                             uint8_t device_address, uint8_t register_addr,
                             uint8_t is_read,
                             uint8_t *buffer, uint16_t length,
                             I2C_TransferCallback callback);

    /**
     * @brief 查询是否正在传输
     * @param i2c_context  平台资源指针
     * @retval 0  空闲
     * @retval 1  传输中
     */
    uint8_t (*is_busy)(void *i2c_context);

} I2C_PlatformOps_t;

// ================================================================
//  I2C 句柄（对标 PWM_Handle / UART_Handle）
// ================================================================

typedef struct I2C_Handle {
    void                     *i2c_context;  ///< 平台资源指针
                                            ///<   软件 I2C → I2C_SoftwareContext*
                                            ///<   硬件 I2C → I2C_HandleTypeDef*
    const I2C_PlatformOps_t  *ops;          ///< 平台操作表指针
} I2C_Handle;

// ================================================================
//  策略层 API（非阻塞）
// ================================================================

/**
 * @brief 异步写 I2C 设备寄存器
 *
 * 内部调用 ops->start_transfer 启动传输后立即返回，
 * 写完成后自动调用 complete_callback 通知调用者。
 *
 * 传输中再次调用返回 -1。
 *
 * @param handle          I2C 句柄
 * @param device_address  从设备 7 位地址
 * @param register_addr   寄存器地址
 * @param data            待写数据缓冲区（调用期间必须保持有效！）
 * @param length          数据字节数
 * @param callback        完成回调（ISR 末尾调用，result=0 成功/负数失败）
 * @retval 0  启动成功
 * @retval -1 句柄忙
 */
int8_t i2c_write_reg_async(I2C_Handle *handle, uint8_t device_address,
                           uint8_t register_addr,
                           uint8_t *data, uint16_t length,
                           I2C_TransferCallback callback);

/**
 * @brief 异步读 I2C 设备寄存器
 *
 * 先写寄存器地址，再重 START 并切换为读模式。
 * 读完成后自动调用 complete_callback 通知调用者。
 *
 * @param handle          I2C 句柄
 * @param device_address  从设备 7 位地址
 * @param register_addr   寄存器地址
 * @param buffer          接收缓冲区（调用期间必须保持有效！）
 * @param length          数据字节数
 * @param callback        完成回调
 * @retval 0  启动成功
 * @retval -1 句柄忙
 */
int8_t i2c_read_reg_async(I2C_Handle *handle, uint8_t device_address,
                          uint8_t register_addr,
                          uint8_t *buffer, uint16_t length,
                          I2C_TransferCallback callback);

/**
 * @brief 查询 I2C 句柄是否正在传输
 * @retval 0  空闲
 * @retval 1  传输中
 */
uint8_t i2c_is_busy(I2C_Handle *handle);

#ifdef __cplusplus
}
#endif

#endif /* __USERI2C_H */
