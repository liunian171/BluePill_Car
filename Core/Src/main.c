/* USER CODE BEGIN Header */
/**
  * @file           : main.c
  * @brief          : 装载层 — CubeMX init + 组装层装载 + 主循环时间喂入
  ******************************************************************************
  * [2026-09-20 组装层拆分] 本文件已瘦身为"装载器"：全部装配知识 (4 张桥配置表 /
  * io 绑定 / init 链) 收口到 Core/Src/app/app_wiring.c (唯一 include car_config.h
  * 的 app 模块)；命令链/控制/显示宿主分居 app_link/app_control/app_display。
  * 本文件保留：CubeMX 骨架、UART 硬件通道 bring-up (baud 重配/首挂/横幅)、
  * UART ISR 壳 (投递 app_link + 重挂)、主循环时间喂入。
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "dma.h"
#include "i2c.h"
#include "tim.h"
#include "usart.h"
#include "usb_device.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "car_config.h"              /* 链路编译开关 (UART 通道 bring-up / ISR 门控用) */
#include "driver/uart.h"             /* uart_debug 句柄 (ISR 重挂经策略层) */
#include "driver/uart_platform_ops.h"
#include "app/app_wiring.h"          /* 装配入口 app_wiring_load() */
#include "app/app_link.h"            /* ISR 收包投递 */
#include "app/app_control.h"         /* WD 闩锁解除 */
#include "app/app_tx.h"              /* 发送流队列推进 (R3) */
#include "app/app_display.h"
#include "driver/line_follower.h"    /* 巡线自动启动仲裁 (脑干职责, 留装载层) */
#include "driver/imu_bridge.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* UART 通道 bring-up 门控 (真值源 = car_config.h; 其余链路开关随装配进 app_wiring) */
#define LINK_UART_ENABLED  CAR_FEATURE_UART
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* UART 硬件通道句柄 (ISR 重挂 + rx_byte 落脚点; 装配知识在 app_wiring) */
UART_Handle    uart_debug = {
    .huart      = &huart2,
    .ops        = &uart_platform_ops_stm32,
};
/* [R3] UART 接收硬件错误累计 (ORE/framing, PA3 悬空噪声; 中断上下文写, 饱和) */
volatile uint16_t g_uart_rx_err = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
/* UART ISR 壳 (装载层/HAL 通道职责): 投递 app_link + 经策略层重挂。
 * [P1-1] 重挂走 uart_receive_IT() → ops → HAL; UART=0 时经 stub 自然 no-op。
 * [app_link 拆分] 收包只投递不解析 (ISR 铁律)。 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *hal_huart)
{
    if (hal_huart->Instance != USART2) return;
#if LINK_UART_ENABLED
    app_link_uart_rx_isr(uart_debug.rx_byte);
#endif
    (void)uart_receive_IT(&uart_debug, &uart_debug.rx_byte);
}

/* [R3 发送路径专项 3a] UART 硬件错误重挂 (PA3 悬空/噪声 → ORE/framing 会中止接收,
 * 此前无 ErrorCallback → 静默停摆)。重挂中断接收 + 计数（诊断用, 饱和）。
 * 中断上下文只计数不解析（ISR 铁律）。 */
void HAL_UART_ErrorCallback(UART_HandleTypeDef *hal_huart)
{
    if (hal_huart->Instance != USART2) return;
#if LINK_UART_ENABLED
    if (g_uart_rx_err < 0xFFFFu) g_uart_rx_err++;
    (void)uart_receive_IT(&uart_debug, &uart_debug.rx_byte);   /* 重挂续收 */
#endif
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */
  /* [免拔插 USB 重枚举 — 试验结论 2026-09-19 ❌ 本板无效，已回退]
   * 固件侧无法制造断开事件 (详调试总结 §27)；对策 = flash_flow.py 流程自动化。 */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_USART2_UART_Init();
  MX_TIM4_Init();
  MX_I2C2_Init();
  MX_USB_DEVICE_Init();
  /* USER CODE BEGIN 2 */
#ifdef USB_ECHO_TEST
    /* ---- USB 独立链路测试模式 (临时): 不 init 任何车功能, 收发全在 CDC_Receive_FS。
     * 回显模式不注册收包 sink → app 链路不参与 (usbd_cdc_if.c 未注册即丢弃)。 */
    for (;;) { HAL_Delay(1); }
#endif
    /* ---- UART 硬件通道 bring-up (装载层/HAL 通道职责) ----
     * PA2/PA3 接 BT04, 9600 覆盖 CubeMX 默认 115200;
     * [P3] CAR_FEATURE_UART=0 → 不重配/不首挂/不发横幅 (通道整体不存在) */
#if LINK_UART_ENABLED
    huart2.Init.BaudRate = 9600;
    if (HAL_UART_Init(&huart2) != HAL_OK) Error_Handler();
    /* 首挂中断接收 (经策略层, P1-1) + 开机横幅 */
    (void)uart_receive_IT(&uart_debug, &uart_debug.rx_byte);
    (void)uart_send(&uart_debug, (const uint8_t *)"UART2 Ready\r\n", 13);
#endif

    /* ---- 装配: 配置表 + io 绑定 + 全部组件/桥/app init (唯一装配点) ----
     * 任一失败 → Error_Handler 停机 ("init 失败即停", 真机已验 2026-09-20 §22);
     * 失败定位可用 app_wiring_failed_module() 查看 */
    if (app_wiring_load() != BRIDGE_OK)
        Error_Handler();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  /* 装载层主循环: 只喂时间, 不含任何业务 (业务节拍门内聚在各 app 模块) */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
        uint32_t now = HAL_GetTick();

        /* ① 命令链 (app_link): 双 ringbuf 消费/门控/命令执行 */
        app_link_task(now);

        /* ②+③b 控制节拍 (app_control): 判活/WD/闭环搬运/感知 (50ms 门内聚) */
        app_control_task(now);

        /* ②' 发送流推进 (app_tx, R3): 每轮摊销推进 UART IT / USB 段队列 (非阻塞自限速) */
        app_tx_service();

        /* ③ 显示节拍 (app_display): 8 页轮转 (100ms 门内聚) */
        app_display_task(now);

        /* 仲裁: IMU 校准完成自动启动巡线 (接管权仲裁 = 脑干职责, 留装载层) */
        static uint32_t t_auto = 0;
        if (now - t_auto >= CAR_DISP_PERIOD_MS) {
            t_auto = now;
            uint8_t prog = imu_bridge_cal_progress(0);
            line_follower_try_auto_start(prog >= 100, now);
        }
  /* USER CODE END 3 */
  }
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USB;
  PeriphClkInit.UsbClockSelection = RCC_USBCLKSOURCE_PLL_DIV1_5;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
    /* init 失败定位: app_wiring_failed_module() (SWD 调试器可读 s_failed 静态) */
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/* USER CODE BEGIN PRIVATE_FUNCTIONS_IMPLEMENTATION */

/* USER CODE END PRIVATE_FUNCTIONS_IMPLEMENTATION */
