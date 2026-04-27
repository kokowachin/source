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
	本程序由吉林大学仪电学院电赛小组开发而来，未经允许禁止采用
  ADC采样：PA0 PA1 PA2 PA3
	PWM互补输出：PE9与PA7(TIM1CH1) PE11 PB0(TIM1CH2)
	PWM普通输出：PA5(TIM2CH1) PB3(TIM2CH2)
	OLED(I2C): PB7 PB6
	串口：PA10(RX) PA9(TX) 
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "dma.h"
#include "i2c.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <string.h>
#include "function.h"
#include "state.h"
#include "encoder.h"
#include "PID.h"
#include "stm32f4xx_it.h"
#include "oled.h"
#include "kalman.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
//毫秒计时变量
volatile uint16_t ms_cnt_1 = 0; // 计时变量1
volatile uint16_t ms_cnt_2 = 0; // 计时变量2
volatile uint16_t ms_cnt_3 = 0;
extern volatile float raw_vout;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */
  char message1[] = "Buck\n";
	char message2[] = "Boost\n";
	char message3[] = "Mix\n";
	char message4[] = "others\n";
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */
	DF.SMFlag = Init;		//初始化状态机

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_ADC1_Init();
  MX_I2C1_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_USART1_UART_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  /* USER CODE BEGIN 2 */
	HAL_ADC_Start_DMA(&hadc1, (uint32_t*)ADC1_RESULT, 4);//启动ADC的DMA采样
	HAL_TIM_Base_Start_IT(&htim4);  // 启动TIM4中断，用于1ms计时
	ValInit();         // 初始化全局变量和结构体
	DF.SMFlag = Init;  // 设定状态机初始状态
	Encoder_Init();	//编码器初始化
	PID_Init();	//PID初始化
	PWM_Start();	//启动PWM
	Kalman_Init() ;
	HAL_TIM_Base_Start_IT(&htim3);  // 启动TIM3中断，用于PID控制
			OLED_Init();
	 OLED_Clear();	
	//Test_PWM_Output();
	HAL_GPIO_WritePin(GPIOC,GPIO_PIN_13,GPIO_PIN_RESET);

char uart_buf[64];
uint32_t last_oled_tick = HAL_GetTick();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
		float v_display = raw_vout ;
		float v_decide = SET_Value.Vout ;
		 // 1. 处理按键（独立于旋转）
    // 1. 处理按键（扫描长短按状态机）
    uint8_t btn_event = Scan_Encoder_Button();
    
    if (btn_event == 1) {
        // === 触发了短按 (VorI已经在内部切换完毕) ===
        if (VorI) {
            HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);   // CC模式亮灯
        } else {
            HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);  // CV模式灭灯
        }
    } 

    // 2. 处理编码器旋转（修改设定值）
    int8_t dir = Get_Encoder_Dir();      
    if (dir != 0) {
        App_Menu_Action(dir);   // 注意：App_Menu_Action 内部不要再处理按键了
    }			
		
			ShortOff();  // 短路保护
			OVP();       // 输出过压保护
			OCP();       // 输出过流保护
			StateM();    // 电源状态机函数
			BBMode();    // 运行模式判断
        
			Check_NoLoad_And_ClearCC();
		if((HAL_GetTick() - last_oled_tick) >= 200){
			last_oled_tick = HAL_GetTick();
				// ==========================================================
// 1. 第一行：显示当前状态
// ==========================================================
// 清空第一行（可选，避免残留）
OLED_ShowChinese(0, 0, 0); // 当
OLED_ShowChinese(0, 1, 1); // 前
OLED_ShowChinese(0, 2, 2); // 状
OLED_ShowChinese(0, 3, 3); // 态
OLED_ShowChinese(0, 4, 4); // 恒
if (VorI == 0) {
    OLED_ShowChinese(0, 5, 5); // 压
    // 清除可能残留的字符
    OLED_ShowString(1, 12, "  "); // 注意这里要用正确的行号
} else {
    OLED_ShowChinese(0, 5, 8); // 流
}

// ==========================================================
// 2. 第二行：显示当前电压或电流值
// ==========================================================
OLED_ShowChinese(1, 0, 0); // 当
OLED_ShowChinese(1, 1, 1); // 前

if (VorI == 0) {    
    // 显示值前先清除该区域（显示5个空格）
    OLED_ShowString(2, 5, "     "); 
    OLED_ShowFloatNum(2, 5, v_display, 2, 1);
    OLED_ShowString(2, 10, "V ");
} else {  
    // 清除区域
    OLED_ShowString(2, 5, "     ");   
    OLED_ShowFloatNum(2, 5, raw_iout, 2, 1);
    OLED_ShowString(2, 10, "A ");
}

// ==========================================================
// 3. 第三/四行：显示设定值
// ==========================================================
    OLED_ShowChinese(2, 0, 10); // 目
    OLED_ShowChinese(2, 1, 11); // 标
if (VorI == 0) {
    // 第三行显示设定电压
    // 清除显示区域
    OLED_ShowString(3, 5, "     ");
    
    OLED_ShowFloatNum(3, 5, v_decide, 2, 1);
    OLED_ShowString(3, 10, "V ");
} else {
    // 第四行显示设定电流
    // 清除显示区域
    OLED_ShowString(3, 5, "     ");
    
    OLED_ShowFloatNum(3,5, SET_Value.Iout, 2, 1);
    OLED_ShowString(3, 10, "A ");
}
}
     // 看看状态机进去了没有？(0=NA, 1=Buck, 2=Boost, 3=Mix)
	/*	 
		 float test = (float)ADC1_RESULT[2] ;
OLED_ShowFloatNum(1, 10, test, 4,2); 

// 看看电流环和电压环到底算出了多少占空比？(如果是0，说明被限流或保护按死了)
OLED_ShowFloatNum(2, 10, raw_vout, 2, 2); 

// 看看定时器的 ARR 读到了没有？(如果是0，说明 TIM1 没启动)
OLED_ShowSignedNum(3, 10, ms_cnt_3,4); 

   */   
										
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

/*
    const char* msg = "";
    switch(new_mode) {
        case 1: msg = "Mode -> Buck\r\n"; break;
        case 2: msg = "Mode -> Boost\r\n"; break;
        case 3: msg = "Mode -> Mix\r\n"; break;
        default: msg = "Mode -> Unknown\r\n"; break;
    }
		
    HAL_UART_Transmit(&huart1, (uint8_t*)msg, strlen(msg), 100);
	*/	 


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
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/**
 * @brief HAL_TIM_PeriodElapsedCallback函数,中断回调函数
 *
 * 当定时器周期结束时，该函数将被调用。
 *
 * @param htim TIM句柄指针
 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  if (htim->Instance == TIM4) // 定时器TIM4，中断时间1ms
  {
    ms_cnt_1++;
    ms_cnt_2++;
  }
}
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
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
