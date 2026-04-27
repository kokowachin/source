/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    stm32f4xx_it.c
  * @brief   Interrupt Service Routines.
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
#include "stm32f4xx_it.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "PID.h"
#include "encoder.h"
#include "kalman.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN TD */

/* USER CODE END TD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */
    float final_duty = 0.0f;
		volatile float raw_iout = 0.0f;
		volatile float raw_vout = 0.0f;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/
extern DMA_HandleTypeDef hdma_adc1;
extern TIM_HandleTypeDef htim3;
/* USER CODE BEGIN EV */
volatile uint8_t mode_switch_flag = 0;
volatile uint8_t new_mode = 0;   // 1:Buck, 2:Boost, 3:Mix
/* USER CODE END EV */

/******************************************************************************/
/*           Cortex-M4 Processor Interruption and Exception Handlers          */
/******************************************************************************/
/**
  * @brief This function handles Non maskable interrupt.
  */
void NMI_Handler(void)
{
  /* USER CODE BEGIN NonMaskableInt_IRQn 0 */

  /* USER CODE END NonMaskableInt_IRQn 0 */
  /* USER CODE BEGIN NonMaskableInt_IRQn 1 */
  while (1)
  {
  }
  /* USER CODE END NonMaskableInt_IRQn 1 */
}

/**
  * @brief This function handles Hard fault interrupt.
  */
void HardFault_Handler(void)
{
  /* USER CODE BEGIN HardFault_IRQn 0 */

  /* USER CODE END HardFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_HardFault_IRQn 0 */
    /* USER CODE END W1_HardFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Memory management fault.
  */
void MemManage_Handler(void)
{
  /* USER CODE BEGIN MemoryManagement_IRQn 0 */

  /* USER CODE END MemoryManagement_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_MemoryManagement_IRQn 0 */
    /* USER CODE END W1_MemoryManagement_IRQn 0 */
  }
}

/**
  * @brief This function handles Pre-fetch fault, memory access fault.
  */
void BusFault_Handler(void)
{
  /* USER CODE BEGIN BusFault_IRQn 0 */

  /* USER CODE END BusFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_BusFault_IRQn 0 */
    /* USER CODE END W1_BusFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Undefined instruction or illegal state.
  */
void UsageFault_Handler(void)
{
  /* USER CODE BEGIN UsageFault_IRQn 0 */

  /* USER CODE END UsageFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_UsageFault_IRQn 0 */
    /* USER CODE END W1_UsageFault_IRQn 0 */
  }
}

/**
  * @brief This function handles System service call via SWI instruction.
  */
void SVC_Handler(void)
{
  /* USER CODE BEGIN SVCall_IRQn 0 */

  /* USER CODE END SVCall_IRQn 0 */
  /* USER CODE BEGIN SVCall_IRQn 1 */

  /* USER CODE END SVCall_IRQn 1 */
}

/**
  * @brief This function handles Debug monitor.
  */
void DebugMon_Handler(void)
{
  /* USER CODE BEGIN DebugMonitor_IRQn 0 */

  /* USER CODE END DebugMonitor_IRQn 0 */
  /* USER CODE BEGIN DebugMonitor_IRQn 1 */

  /* USER CODE END DebugMonitor_IRQn 1 */
}

/**
  * @brief This function handles Pendable request for system service.
  */
void PendSV_Handler(void)
{
  /* USER CODE BEGIN PendSV_IRQn 0 */

  /* USER CODE END PendSV_IRQn 0 */
  /* USER CODE BEGIN PendSV_IRQn 1 */

  /* USER CODE END PendSV_IRQn 1 */
}

/**
  * @brief This function handles System tick timer.
  */
void SysTick_Handler(void)
{
  /* USER CODE BEGIN SysTick_IRQn 0 */

  /* USER CODE END SysTick_IRQn 0 */
  HAL_IncTick();
  /* USER CODE BEGIN SysTick_IRQn 1 */

  /* USER CODE END SysTick_IRQn 1 */
}

/******************************************************************************/
/* STM32F4xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32f4xx.s).                    */
/******************************************************************************/

/**
  * @brief This function handles TIM3 global interrupt.
  */
void TIM3_IRQHandler(void)
{
  /* USER CODE BEGIN TIM3_IRQn 0 */

  /* USER CODE END TIM3_IRQn 0 */
  HAL_TIM_IRQHandler(&htim3);
  /* USER CODE BEGIN TIM3_IRQn 1 */

		// =================================================================
  // 1. 获取基础硬件参数
  // =================================================================
  uint32_t current_arr = TIM1->ARR;           // 定时器自动重载值
  float vin_val = 24.0f;                      // 输入电压（固定值，可改为实时采样）

  // =================================================================
  // 2. 传感器 ADC 采集与物理量还原（反馈链路）
  // =================================================================
  float raw_v_pin = ((float)ADC1_RESULT[2] / 4095.0f) * 3.3f;
  float raw_vout1 = raw_v_pin * (75.0f / 4.7f);      // 输出电压 (V)
	raw_vout = Kalman_Update(&kal_v, raw_vout1);

  float raw_i_pinv = ((float)ADC1_RESULT[3] / 4095.0f) * 3.3f;
  float raw_iout1 = (raw_i_pinv / 50.0f) / 0.01f;    // 输出电流 (A)，根据实际运放倍数调整
	raw_iout = Kalman_Update(&kal_i, raw_iout1);
  // =================================================================
  // 3. 拓扑模式跨越：无缝切换与积分重置（原逻辑保留）
  // =================================================================
  if (DF.BBModeChange == 1) {
      float new_duty = 0.0f;
      if (DF.BBFlag == Buck)
          new_duty = (SET_Value.Vout / vin_val) * 100.0f;
      else if (DF.BBFlag == Boost)
          new_duty = ((SET_Value.Vout - vin_val) / SET_Value.Vout) * 100.0f;
      else if (DF.BBFlag == Mix)
          new_duty = (SET_Value.Vout / (vin_val + SET_Value.Vout)) * 100.0f;

      if (new_duty < 10.0f) new_duty = 10.0f;
      // 将两个环的积分器同时对齐到新占空比，防止切换瞬间突变
      pid_v.integral = new_duty;
      pid_i.integral = new_duty;
      // 通知主循环发送模式变化
      new_mode = (uint8_t)DF.BBFlag;
      mode_switch_flag = 1;
      DF.BBModeChange = 0;
  }

  // =================================================================
  // 4. 双环 PID 计算（输入为物理量）
  // =================================================================
  float duty_v = PID_Calc(&pid_v, SET_Value.Vout, raw_vout);
  float duty_i = PID_Calc(&pid_i, SET_Value.Iout, raw_iout);

  // =================================================================
  // 5. 模式切换时的积分预置（平滑过渡）
  // =================================================================
  static uint8_t last_mode = 0;       // 0:CV, 1:CC
  if (last_mode != VorI) {
      if (VorI == 1) {                // 刚进入恒流模式
          // 将电流环积分器设置为当前正在使用的占空比
          pid_i.integral = (last_mode == 0) ? duty_v : duty_i;
          if (pid_i.integral > pid_i.output_max) pid_i.integral = pid_i.output_max;
          if (pid_i.integral < pid_i.output_min) pid_i.integral = pid_i.output_min;
      } else {                        // 刚切回恒压模式
          pid_v.integral = (last_mode == 1) ? duty_i : duty_v;
          if (pid_v.integral > pid_v.output_max) pid_v.integral = pid_v.output_max;
          if (pid_v.integral < pid_v.output_min) pid_v.integral = pid_v.output_min;
      }
      last_mode = VorI;
  }

  // =================================================================
  // 6. 恒流模式下的安全保护（自动退回到恒压）
  // =================================================================
  float final_duty_tmp;
  if (!VorI) {
      final_duty_tmp = duty_v;               // 恒压模式：电压环主导
  } else {
      final_duty_tmp = duty_i;               // 恒流模式：电流环完全主导

      // 6.1 输出电压过高保护（负载开路或异常）
      if (raw_vout > 45.0f) {
          VorI = 0;                          // 强制切回恒压
          last_mode = 0;
          pid_v.integral = final_duty_tmp;   // 将当前占空比赋给电压环积分器
          final_duty_tmp = duty_v;           // 立即使用电压环输出
          mode_switch_flag = 1;
          new_mode = 0;                      // 0 表示恒压模式
      }
      // 6.2 无负载检测（电流极小且持续一段时间）
      else {
          static uint16_t no_load_cnt = 0;
          if (raw_iout < 0.05f) {            // 电流 < 50mA 视为无负载
              no_load_cnt++;
              // 假设中断频率 10kHz，计数 5000 次 = 500ms
              if (no_load_cnt > 5000) {
                  VorI = 0;
                  last_mode = 0;
                  pid_v.integral = final_duty_tmp;
                  final_duty_tmp = duty_v;
                  mode_switch_flag = 1;
                  new_mode = 0;
                  no_load_cnt = 0;
              }
          } else {
              no_load_cnt = 0;               // 有负载时复位计数器
          }
      }
  }

  float final_duty = final_duty_tmp;

  // =================================================================
  // 7. 拓扑强制降级（防短路时 Boost 过度升压）
  // =================================================================
  uint8_t current_hw_mode = DF.BBFlag;
  if (VorI && raw_vout < (vin_val * 0.95f)) {
      current_hw_mode = Buck;
  }

  // =================================================================
  // 8. 各模式独立安全限幅
  // =================================================================
  if (current_hw_mode == Buck) {
      if (final_duty > 95.0f) final_duty = 95.0f;
  } else if (current_hw_mode == Boost) {
      if (final_duty > 50.0f) final_duty = 50.0f;
  } else if (current_hw_mode == Mix) {
      if (final_duty > 60.0f) final_duty = 60.0f;
  }
  if (final_duty < 0.0f) final_duty = 0.0f;

  // =================================================================
  // 9. 转换为定时器比较值并更新 PWM
  // =================================================================
  uint32_t ccr_val = (uint32_t)((final_duty / 100.0f) * current_arr);
  switch (current_hw_mode) {
      case Buck:
          TIM1->CCR1 = ccr_val;
          TIM1->CCR2 = current_arr - 6;       // Boost 上管常通
          break;
      case Boost:
          TIM1->CCR1 = current_arr - 6;       // Buck 上管常通
          TIM1->CCR2 = current_arr - ccr_val;  // Boost 反相输出
          break;
      case Mix:
          TIM1->CCR1 = ccr_val;
          TIM1->CCR2 = current_arr - ccr_val;
          break;
      default:
          TIM1->CCR1 = 0;
          TIM1->CCR2 = 0;
          break;
  }
		
		

  /* USER CODE END TIM3_IRQn 1 */
}

/**
  * @brief This function handles DMA2 stream0 global interrupt.
  */
void DMA2_Stream0_IRQHandler(void)
{
  /* USER CODE BEGIN DMA2_Stream0_IRQn 0 */

  /* USER CODE END DMA2_Stream0_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_adc1);
  /* USER CODE BEGIN DMA2_Stream0_IRQn 1 */

  /* USER CODE END DMA2_Stream0_IRQn 1 */
}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */
