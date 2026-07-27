/**
 * ============================================================================
 *  软件 I2C 平台操作表 — STM32 平台实现（三层 FSM + 定时器 ISR）
 * ============================================================================
 *
 *  本文件实现 useri2c_ops.h 中声明的各个函数，对标 uart_platform_ops.c。
 *
 *  硬件资源：
 *    - SCL: PF0（开漏 + 上拉）
 *    - SDA: PF1（开漏 + 上拉）
 *    - 定时器: TIM6（PSC=0, ARR=899, 周期 10μs, SCL ~50kHz）
 *
 *  ISR 流程：
 *    TIM6_DAC_IRQHandler → i2c_software_timer_isr(ctx)
 *      ├─ START/STOP → i2c_sw_fsm_edge()     边沿信号（3 tick）
 *      └─ SEND/RECV  → i2c_sw_fsm_bit()      逐 bit 微状态机
 *           └─ SAMPLE 末尾 → i2c_sw_fsm_byte() 逐字节宏状态机
 */

#include "useri2c_ops.h"
#include "usergpio.h"
#include "tim.h"


/* ==========================================================================
 *  原子操作 —— GPIO 读写 & 定时器启停
 *  所有上层 FSM 只调用这 5 个 helper，不直接碰硬寄存器。
 * ========================================================================== */

static void i2c_sw_scl_set(I2C_SoftwareContext *ctx, uint8_t level)
{
    usergpio_write(&ctx->scl_pin, level);
}

static void i2c_sw_sda_set(I2C_SoftwareContext *ctx, uint8_t level)
{
    usergpio_write(&ctx->sda_pin, level);
}

static uint8_t i2c_sw_sda_read(I2C_SoftwareContext *ctx)
{
    return usergpio_read(&ctx->sda_pin);
}

static void i2c_sw_timer_start(I2C_SoftwareContext *ctx)
{
    HAL_TIM_Base_Start_IT((TIM_HandleTypeDef *)ctx->timer_handle);
}

static void i2c_sw_timer_stop(I2C_SoftwareContext *ctx)
{
    HAL_TIM_Base_Stop_IT((TIM_HandleTypeDef *)ctx->timer_handle);
}


/* 前向声明（fsm_bit & fsm_byte 互相依赖） */
static void i2c_sw_fsm_byte(I2C_SoftwareContext *ctx);


/* ==========================================================================
 *  边沿信号子 FSM（fsm_edge）—— 处理 START / STOP
 *
 *  START: 3 tick（SCL=1 期间 SDA↓）
 *  STOP:  3 tick（SCL=1 期间 SDA↑）
 *  不走 2-tick 模型，因为 START/STOP 要求 SCL=1 时改变 SDA。
 * ========================================================================== */

static void i2c_sw_fsm_edge(I2C_SoftwareContext *ctx)
{
    /* ════════════ START ════════════ */
    if (ctx->macro_state == I2C_MACRO_START)
    {
        /* RESTART 时先拉 SCL 低（避免 SDA 跳变被误判 STOP），首次 START 直接跳 */
        int step_offset = (ctx->byte_index > 0) ? 0 : 1;  // byte_index>0 → RESTART
        int step = ctx->edge_step + step_offset;

        if (step == 0) {
            i2c_sw_scl_set(ctx, 0);              // ★ RESTART: 确保 SCL 低
        }
        else if (step == 1) {
            i2c_sw_scl_set(ctx, 1);
            i2c_sw_sda_set(ctx, 1);              // 总线空闲
        }
        else if (step == 2) {
            i2c_sw_sda_set(ctx, 0);              // ★ SCL=1, SDA↓ → START
        }
        else { // step >= 3
            i2c_sw_scl_set(ctx, 0);              // 拉低 SCL，准备发数据
            ctx->micro_state = I2C_MICRO_PREPARE;
            ctx->macro_state = I2C_MACRO_SEND;
        }
        ctx->edge_step++;
        return;
    }

    /* ════════════ STOP ════════════ */
    if (ctx->macro_state == I2C_MACRO_STOP)
    {
        if (ctx->edge_step == 0) {
            i2c_sw_scl_set(ctx, 0);
            i2c_sw_sda_set(ctx, 0);              // 先确保低电平
        }
        else if (ctx->edge_step == 1) {
            i2c_sw_scl_set(ctx, 1);              // 先拉高 SCL
        }
        else { // edge_step == 2
            i2c_sw_sda_set(ctx, 1);              // ★ SCL=1, SDA↑ → STOP
            i2c_sw_timer_stop(ctx);
            ctx->macro_state = I2C_MACRO_DONE;
            if (ctx->complete_callback)
                ctx->complete_callback(ctx, ctx->transfer_result);
            ctx->macro_state = I2C_MACRO_IDLE;
            return;
        }
        ctx->edge_step++;
        return;
    }
}


/* ==========================================================================
 *  逐 bit 微状态机（fsm_bit）—— PREPARE ↔ SAMPLE 交替
 *
 *  [调用前提] timer_isr 分派，仅 macro_state==SEND 或 RECV 时调用。
 *
 *  [处理时序] PREPARE / SAMPLE 并列，每 10μs 交替一次 → 末尾调 fsm_byte
 *
 *  [PREPARE: SCL=0，安全改变 SDA]
 *    SEND: 数据位→SDA=send_byte[bit_counter],  ACK位→SDA=1(释放)
 *    RECV: 数据位→SDA=1(释放),                 ACK位→SDA=0(ACK)/1(NACK)
 *
 *  [SAMPLE: SCL=1，从机采样 / 主机读取]
 *    SEND: 从机自动采样，ACK位由 fsm_byte 检查
 *    RECV: 数据位→读SDA左移存入receive_byte
 * ========================================================================== */

static void i2c_sw_fsm_bit(I2C_SoftwareContext *ctx)
{
    /* ════════════ PREPARE：SCL=0，主机安全改变 SDA ════════════ */
    if (ctx->micro_state == I2C_MICRO_PREPARE)
    {
        i2c_sw_scl_set(ctx, 0);

        /* --- SEND：主机往 SDA 放数据位 --- */
        if (ctx->macro_state == I2C_MACRO_SEND)
        {
            if (ctx->bit_counter == -1)
                i2c_sw_sda_set(ctx, 1);          // ACK 位：释放 SDA
            else
                i2c_sw_sda_set(ctx, (ctx->send_byte >> ctx->bit_counter) & 1);
        }
        /* --- RECV：主机释放总线 或 发 ACK/NACK --- */
        else if (ctx->macro_state == I2C_MACRO_RECV)
        {
            if (ctx->bit_counter == -1)           // ACK 位：主机驱动 SDA
            {
                if ((ctx->byte_index - 2) + 1 >= ctx->data_length)
                    i2c_sw_sda_set(ctx, 1);       // NACK：最后一字节
                else
                    i2c_sw_sda_set(ctx, 0);       // ACK：继续收
            }
            else                                  // 数据位：释放 SDA
                i2c_sw_sda_set(ctx, 1);
        }
    }
    /* ════════════ SAMPLE：SCL=1，从机采样 / 主机读取 ════════════ */
    else // micro_state == I2C_MICRO_SAMPLE
    {
        i2c_sw_scl_set(ctx, 1);

        /* --- RECV：主机读 SDA --- */
        if (ctx->macro_state == I2C_MACRO_RECV)
        {
            if (ctx->bit_counter >= 0)            // 数据位：左移存入
                ctx->receive_byte = (ctx->receive_byte << 1)
                                  | i2c_sw_sda_read(ctx);
        }
        // SEND：从机自动采样，ACK 由 fsm_byte 检查

        i2c_sw_fsm_byte(ctx);                     // 通知宏状态机推进
    }

    /* 翻转 micro_state：0↔1，下次 ISR 走对面分支 */
    ctx->micro_state = (ctx->micro_state == I2C_MICRO_PREPARE)
                       ? I2C_MICRO_SAMPLE : I2C_MICRO_PREPARE;
}


/* ==========================================================================
 *  逐字节宏状态机（fsm_byte）—— 字节调度 & 事务推进
 *
 *  [调用前提] fsm_bit 在 SAMPLE 末尾调用，只做纯逻辑决策。
 *
 *  [处理时序 —— 三层递进，每层 return 拦截]
 *    ① bit_counter > 0  → bit_counter-- (继续当前字节)
 *    ② bit_counter == 0 → bit_counter = -1 (进 ACK 位)
 *    ③ bit_counter == -1 → ACK 完成，字节级决策（装下一字节 or STOP）
 *
 *  [③ 内部 —— SEND / RECV 并列]
 *    byte_index: 0=设备地址, 1=寄存器地址, 2+=数据字节
 * ========================================================================== */

static void i2c_sw_fsm_byte(I2C_SoftwareContext *ctx)
{
    /* ① 字节进行中：递减 bit_counter */
    if (ctx->bit_counter > 0) {
        ctx->bit_counter--;
        return;
    }

    /* ② 8 bit 完成：进 ACK */
    if (ctx->bit_counter == 0) {
        ctx->bit_counter = -1;
        return;
    }

    /* ③ ACK 完成（bit_counter == -1）：字节级决策 */

    /* ════════════ SEND：主机发送 ════════════ */
    if (ctx->macro_state == I2C_MACRO_SEND)
    {
        /* --- 检查从机 ACK --- */
        if (i2c_sw_sda_read(ctx) != 0) {
            ctx->transfer_result = -1;
            ctx->macro_state = I2C_MACRO_STOP;
            ctx->edge_step   = 0;
            return;
        }

        /* --- 根据 byte_index 装下一字节 --- */
        if (ctx->byte_index == 0)
        {
            ctx->send_byte = ctx->register_address;         // 设备地址→寄存器地址
        }
        else if (ctx->byte_index == 1)
        {
            if (ctx->is_read)                               // 读事务 → RESTART
            {
                ctx->send_byte   = (ctx->device_address << 1) | 1;
                ctx->macro_state = I2C_MACRO_START;
                ctx->edge_step   = 0;
                ctx->byte_index++;
                ctx->bit_counter = 7;
                return;
            }
            else                                            // 写事务 → data[0]
            {
                ctx->send_byte = ctx->data_buffer[0];
            }
        }
        else                                                // byte_index >= 2
        {
            if (ctx->is_read)                               // RESTART 后→切 RECV
            {
                ctx->macro_state = I2C_MACRO_RECV;
                ctx->bit_counter = 7;
                return;                                     // byte_index 保持 2
            }

            uint16_t next_idx = ctx->byte_index - 1;        // 下一数据下标
            if (next_idx >= ctx->data_length) {             // 全部发完
                ctx->macro_state = I2C_MACRO_STOP;
                ctx->edge_step   = 0;
                return;
            }
            ctx->send_byte = ctx->data_buffer[next_idx];
        }
        ctx->byte_index++;
        ctx->bit_counter = 7;
    }
    /* ════════════ RECV：主机接收 ════════════ */
    else if (ctx->macro_state == I2C_MACRO_RECV)
    {
        ctx->data_buffer[ctx->byte_index - 2] = ctx->receive_byte;

        if ((ctx->byte_index - 2) + 1 >= ctx->data_length) {
            ctx->macro_state = I2C_MACRO_STOP;
            ctx->edge_step   = 0;
        } else {
            ctx->byte_index++;
            ctx->bit_counter = 7;
        }
    }
}


/* ==========================================================================
 *  is_busy — 查询句柄是否在传输中
 *  供策略层 i2c_is_busy 调用，填入 I2C_PlatformOps_t。
 * ========================================================================== */

uint8_t i2c_software_is_busy(void *i2c_context)
{
    I2C_SoftwareContext *ctx = (I2C_SoftwareContext *)i2c_context;
    return (ctx->macro_state != I2C_MACRO_IDLE) ? 1 : 0;
}


/* ==========================================================================
 *  平台操作表实例
 * ========================================================================== */

const I2C_PlatformOps_t i2c_software_platform_ops_stm32 = {
    .start_transfer = i2c_software_start_transfer,
    .is_busy        = i2c_software_is_busy,
};


/* ==========================================================================
 *  start_transfer — 填入事务参数 + 启动定时器 ISR
 *
 *  硬件资源（scl_pin/sda_pin/timer_handle）在 I2C_SoftwareContext 定义时初始化。
 *  本函数只填每次传输可能变化的事务参数。
 * ========================================================================== */

int8_t i2c_software_start_transfer(void *i2c_context,
    uint8_t device_address, uint8_t register_addr,
    uint8_t is_read, uint8_t *buffer, uint16_t length,
    I2C_TransferCallback callback)
{
    I2C_SoftwareContext *ctx = (I2C_SoftwareContext *)i2c_context;

    /* 忙检查 */
    if (ctx->macro_state != I2C_MACRO_IDLE) return -1;

    /* 填入事务参数 */
    ctx->device_address    = device_address;
    ctx->register_address  = register_addr;
    ctx->is_read           = is_read;
    ctx->data_buffer       = buffer;
    ctx->data_length       = length;
    ctx->byte_index        = 0;
    ctx->complete_callback = callback;
    ctx->transfer_result   = 0;

    /* 初始化 FSM 状态 + 首发球 + 启动定时器 */
    ctx->send_byte   = (ctx->device_address << 1) | 0;
    ctx->bit_counter = 7;
    ctx->micro_state = I2C_MICRO_PREPARE;
    ctx->edge_step   = 0;
    ctx->macro_state = I2C_MACRO_START;
    i2c_sw_timer_start(ctx);

    return 0;
}


/* ==========================================================================
 *  定时器 ISR 入口 —— 三层 FSM 调度器
 *
 *  由 stm32f1xx_it.c 的 TIM4_IRQHandler 每 10μs 调用一次。
 *  读 macro_state 分派到对应 FSM。
 * ========================================================================== */

void i2c_software_timer_isr(I2C_SoftwareContext *ctx)
{
    /* START / STOP：边沿信号（3 tick） */
    if (ctx->macro_state == I2C_MACRO_START ||
        ctx->macro_state == I2C_MACRO_STOP) {
        i2c_sw_fsm_edge(ctx);
        return;
    }

    /* SEND / RECV：逐 bit 数据传输 */
    if (ctx->macro_state == I2C_MACRO_SEND ||
        ctx->macro_state == I2C_MACRO_RECV) {
        i2c_sw_fsm_bit(ctx);
        return;
    }

    /* DONE / IDLE：不应到达，兜底关定时器 */
    if (ctx->macro_state == I2C_MACRO_DONE ||
        ctx->macro_state == I2C_MACRO_IDLE) {
        i2c_sw_timer_stop(ctx);
    }
}


/* ==========================================================================
 *  集成备忘（待使用者手动添加）
 *
 *  stm32f4xx_it.c:
 *    void TIM6_DAC_IRQHandler(void) {
 *        HAL_TIM_IRQHandler(&htim6);
 *        if (__HAL_TIM_GET_FLAG(&htim6, TIM_FLAG_UPDATE)) {
 *            __HAL_TIM_CLEAR_FLAG(&htim6, TIM_FLAG_UPDATE);
 *            i2c_software_timer_isr(&g_software_i2c);
 *        }
 *    }
 *
 *  main.c / 初始化文件:
 *    I2C_SoftwareContext g_software_i2c = {
 *        .scl_pin = { .hgpio_port=GPIOF, .gpio_pin=0, .ops=&usergpio_platform_ops_stm32 },
 *        .sda_pin = { .hgpio_port=GPIOF, .gpio_pin=1, .ops=&usergpio_platform_ops_stm32 },
 *        .timer_handle = &htim6,
 *        .macro_state = I2C_MACRO_IDLE,
 *        .micro_state = I2C_MICRO_IDLE,
 *    };
 * ========================================================================== */
