/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "dma.h"
#include "fdcan.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include "drv_adc_sampling.h"
#include "drv_spi_as5048a.h"
#include "drv_spi_as5048a_debug.h"
#include "drv_tim_pwm.h"
#include "ctl_foc_core.h"
#include "ctl_foc_debug.h"
#include "ctl_foc_openloop.h"
#include "ctl_foc_current.h"
#include "ctl_foc_speed.h"
#include "ctl_foc_position.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/** @brief DRV8313 nFAULT 功能开关: 1=启用, 0=关闭 */
#define NFAULT_ENABLE  0

/** @brief 位置环自动步进: 1=每500ms转60°, 0=手动拧到新位置保持3秒即锁定 */
#define POS_AUTO_STEP  1

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

#if NFAULT_ENABLE
/** @brief DRV8313 nFAULT 标志（ISR 置 1，主循环检测并处理） */
static volatile uint8_t nfault_triggered = 0;
#endif

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

#if NFAULT_ENABLE
/**
 * @brief   GPIO EXTI 回调（覆盖 HAL 弱定义）
 * @note    由 EXTI15_10_IRQHandler → HAL_GPIO_EXTI_IRQHandler 链调用。
 *          PB11 (nFAULT) 下降沿触发 → DRV8313 报告故障 → 紧急停机。
 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    if (GPIO_Pin == GPIO_EXTI11_nFAULT_Pin) {
        nfault_triggered = 1;
        if (g_foc.state >= FOC_STATE_READY) {
            drv_tim_pwm_moe_off();
            HAL_GPIO_WritePin(GPIOC, GPIO_Output_EN_Pin, GPIO_PIN_RESET);
        }
    }
}
#endif

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  uint32_t tick_led = 0U;
  uint32_t tick_prn = 0U;
  uint32_t tick_now = 0U;
#if POS_AUTO_STEP
  uint32_t tick_pos = 0U;
  float    pos_cmd = 0.0f;
  int      pos_step = 0;
#else
  uint32_t tick_hold = 0U;         /* 手动拧偏后持续偏离的起始时刻 */
  uint8_t  hold_active = 0;        /* 1=正在计时, 0=未偏离或已复位 */
#endif

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_ADC1_Init();
  MX_ADC2_Init();
  MX_FDCAN1_Init();
  MX_SPI1_Init();
  MX_TIM1_Init();
  MX_USART1_UART_Init();
  MX_TIM6_Init();
  MX_TIM7_Init();
  MX_TIM4_Init();
  /* USER CODE BEGIN 2 */

  /* 位置闭环 */
  FOC_SystemInit();
  FOC_Debug_Init();
#if POS_AUTO_STEP
  /* 自动步进: 每 0.5s 转 60° */
  FOC_Position_SetRef(&g_foc, 0U);
  printf("\r\n=== Position Loop (auto step 60deg/0.5s) ===\r\n\r\n");
  tick_pos = HAL_GetTick() + 500U;
#else
  /* 手动示教: 锁在当前角度, 拧偏保持6秒即更新目标 */
  FOC_Position_SetRef(&g_foc, g_foc.raw_angle);
  printf("\r\n=== Position Loop (hold-6s-to-relock) ===\r\n\r\n");
#endif

  tick_prn = HAL_GetTick() + 1000U;

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* ---- LED 心跳 ---- */
    tick_now = HAL_GetTick();
    if (tick_now >= tick_led) {
      HAL_GPIO_TogglePin(GPIOC, GPIO_Output_LED_Pin);
      tick_led = tick_now + 1000U;
    }

    /* USER CODE BEGIN 3 */

    tick_now = HAL_GetTick();
    if (tick_now >= tick_prn) {
        FOC_Debug_Print_Compact(&g_foc);
        tick_prn = tick_now + 20U;
    }

#if POS_AUTO_STEP
    /* 自动步进: 每 0.5s 转 60° (2731 LSB), 持续同向旋转 */
    tick_now = HAL_GetTick();
    if (tick_now >= tick_pos) {
        pos_step++;
        pos_cmd += 2731.0f;
        CTL_PID_SetSetpoint(&g_foc.pid_pos, pos_cmd);
        printf("\r\n--- Step %d: pos=%.0f LSB (%.0f deg) ---\r\n\r\n",
               pos_step, (double)pos_cmd, (double)(pos_step * 60.0f));
        tick_pos = tick_now + 500U;
    }
#else
    /* 手动示教: 拧偏超过 500 LSB (~11°) 并保持 6 秒 → 锁定新位置 */
    tick_now = HAL_GetTick();
    {
        float sp  = g_foc.pid_pos.setpoint;
        float fb  = g_foc.unwrapped_pos;
        float err = sp - fb;
        if (err < 0.0f) err = -err;
        if (err > 500.0f) {
            if (!hold_active) {
                hold_active = 1;
                tick_hold   = tick_now;
            } else if (tick_now - tick_hold >= 6000U) {
                CTL_PID_SetSetpoint(&g_foc.pid_pos, fb);
                printf("\r\n--- Hold 6s: locked at %.0f LSB ---\r\n\r\n", (double)fb);
                hold_active = 0;
            }
        } else {
            hold_active = 0;
        }
    }
#endif

  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1_BOOST);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV2;
  RCC_OscInitStruct.PLL.PLLN = 85;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }

  /** Enables the Clock Security System
  */
  HAL_RCC_EnableCSS();
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
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
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
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
