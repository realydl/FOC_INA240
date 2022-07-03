/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2022 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "dma.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
extern int16_t adc_buff[ADC_NUM_MAX];    // ADC电压采集缓冲区
//extern float adc_buff[ADC_NUM_MAX];    // ADC电压采集缓冲区
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
extern float adc_u;
extern float adc_v;
uint16_t IU,IV;

//extern __IO uint16_t ChannelPulse;
__IO uint16_t ChannelPulse = 5600 - 1;//100%占空比

float targetId;
float targetIq;
float velocity_target;	//待设定目标速度
float position_target;	//待设定目标位置

int  pole_pairs	= 7;//电机极对数

long sensor_direction;
float voltage_power_supply;
float voltage_limit;
float voltage_sensor_align;

float velocity_limit;
float current_limit;

uint32_t upposition;
uint32_t upvelocity;
uint32_t upcurrentq;
uint32_t upcurrentd;

float now_velocity;
float now_position;
float Iq_target;

PhaseCurrent_s current_test;
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

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
	voltage_limit = 6;          //V，最大值需小于 供电电压/sqrt(3) 12/1.732=6.9 24/sqrt(3)=13.8568
	voltage_power_supply = 12; 	//电源电压
	voltage_sensor_align = 1;   //V alignSensor() use it，大功率电机设置的值小一点比如0.5-1，小电机设置的大一点比如2-3
	current_limit = 50; 				//电流限制，速度模式和位置模式起作用
	velocity_limit=20;         	//位置模式速度限制
	velocity_target = 3;        //待设定目标速度 3 rad/s
	position_target = 3;
	Iq_target = 0;
	targetId = 0;
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
  MX_TIM8_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
	protocol_init();
	
	__HAL_TIM_SetCounter(&htim4, 0);
  __HAL_TIM_ENABLE(&htim4);
	
	IIC_Init();
	my_delay_ms(1000);
		
	HAL_ADC_Start(&hadc1);
	//HAL_ADC_Start_DMA(&hadc3,(uint32_t *)&ADC_ConvertedValue,3);
	HAL_ADC_Start_DMA(&hadc1,(uint32_t*)&adc_buff,ADC_NUM_MAX);//ADC采样点数
	
	LPF_init();
	PID_init();
	
	InlineCurrentSense(0.01,50);    //SimpleMotor//采样电阻阻值，运放倍数
	
	Motor_init();
	my_delay_ms(1000);
	Motor_initFOC(0,UNKNOWN);
	my_delay_ms(1000);
	Current_calibrateOffsets();//电流偏置测量(空载情况下的电流)
	my_delay_ms(1000);
	
	set_computer_value(SEND_STOP_CMD, CURVES_CH1, NULL, 0);                // 同步上位机的启动按钮状态
	set_computer_value(SEND_STOP_CMD, CURVES_CH2, NULL, 0);                // 同步上位机的启动按钮状态
	set_computer_value(SEND_STOP_CMD, CURVES_CH3, NULL, 0);                // 同步上位机的启动按钮状态
	set_computer_value(SEND_STOP_CMD, CURVES_CH4, NULL, 0);                // 同步上位机的启动按钮状态
	
	set_computer_value(SEND_TARGET_CMD, CURVES_CH1, &upcurrentq, 1);     // 给通道 3发送目标值
	set_computer_value(SEND_TARGET_CMD, CURVES_CH2, &upcurrentd, 1);     // 给通道 3发送目标值
	set_computer_value(SEND_TARGET_CMD, CURVES_CH3, &velocity_target, 1);     // 给通道 3发送目标值
	set_computer_value(SEND_TARGET_CMD, CURVES_CH4, &position_target, 1);     // 给通道 3发送目标值
	
	set_computer_value(SEND_START_CMD, CURVES_CH1, NULL, 0);               // 同步上位机的启动按钮状态
	set_computer_value(SEND_START_CMD, CURVES_CH2, NULL, 0);               // 同步上位机的启动按钮状态
	set_computer_value(SEND_START_CMD, CURVES_CH3, NULL, 0);               // 同步上位机的启动按钮状态
	set_computer_value(SEND_START_CMD, CURVES_CH4, NULL, 0);               // 同步上位机的启动按钮状态
	
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
		receiving_process();
		
		//电机对相
		#if 0
//		__HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_1,0);
//		__HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_2,0);
//		__HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_3,0.09*ChannelPulse);
		
//		__HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_1,0);
//		__HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_2,0.09*ChannelPulse);
//		__HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_3,0);
		
		__HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_1,0.09*ChannelPulse);
		__HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_2,0);
		__HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_3,0);
		
		current_test = getPhaseCurrents();
		#endif
		
//		setPhaseVoltage(0, 1, _2PI * 1.0f/ 12.0f);
//		setPhaseVoltage(0, 1, _2PI * 3.0f/ 12.0f);
//		setPhaseVoltage(0, 1, _2PI * 5.0f/ 12.0f);
//		setPhaseVoltage(0, 1, _2PI * 7.0f/ 12.0f);
//		setPhaseVoltage(0, 1, _2PI * 9.0f/ 12.0f);
//		setPhaseVoltage(0, 1, _2PI * 11.0f/ 12.0f);
		
//		shaft_angle = shaftAngle();// shaft angle
//		electrical_angle = electricalAngle();// electrical angle - need shaftAngle to be called first
//		current = getFOCCurrents(electrical_angle);
		
		
//		move(Iq_target);
//Id
#if 0
		shaft_angle = shaftAngle();// shaft angle
		electrical_angle = electricalAngle();// electrical angle - need shaftAngle to be called first
		
		// read dq currents
		current = getFOCCurrents(electrical_angle);
		// filter values
//		current.q = LPFoperator(&LPF_current_q,current.q);
		current.d = LPFoperator(&LPF_current_d,current.d);
		
		voltage.d = PIDoperator(&PID_current_d, (targetId - current.d));
		setPhaseVoltage(0, voltage.d, electrical_angle);
#endif		

//Iq
#if 0
		shaft_angle = shaftAngle();// shaft angle
		electrical_angle = electricalAngle();// electrical angle - need shaftAngle to be called first
		
		// read dq currents
		current = getFOCCurrents(electrical_angle);
		// filter values
		current.q = LPFoperator(&LPF_current_q,current.q);
//		current.d = LPFoperator(&LPF_current_d,current.d);
		
//		voltage.d = PIDoperator(&PID_current_d, (targetId - current.d));
		voltage.q = PIDoperator(&PID_current_q, (targetIq - current.q));
		setPhaseVoltage(voltage.q , 0, electrical_angle);
#endif
#if 0
		move(0.05);
		loopFOCtest();
		shaft_velocity = shaftVelocity();//测速
		upvelocity = shaft_velocity * 1000;
#endif
//		//速度模式
#if 1
		now_velocity = move_velocity(velocity_target);
		loopFOCtest();
//		upposition = getAngle();
		
//		upvelocity = now_velocity * 1000;
//		upcurrentq = current.q * 1000 + 1000;//扩大1000，偏置1000
//		upcurrentd = current.d * 1000 + 1000;//扩大1000,偏置1000
		
//		set_computer_value(SEND_FACT_CMD, CURVES_CH1, &upcurrentq, 1);     // 给通道1发Iq
//		set_computer_value(SEND_FACT_CMD, CURVES_CH2, &upcurrentd, 1);     // 给通道2发Id
//		set_computer_value(SEND_FACT_CMD, CURVES_CH3, &upvelocity, 1);     // 给通道3发送速度值
////		set_computer_value(SEND_FACT_CMD, CURVES_CH4, &upposition, 1);     // 给通道4发送位置值
#endif

		//位置环模式
#if 0
		now_position = move_position(position_target);
		loopFOCtest();//电流环PID
		
		now_velocity = shaftVelocity();
		
//		upvelocity = now_velocity * 1000;//扩大1000
//		upcurrentq = current.q * 1000 + 1000;//扩大1000，偏置1000
//		upcurrentd = current.d * 1000 + 1000;//扩大1000,偏置1000
#endif

//		now_velocity= move_velocity(velocity_target);
//		loopFOCtest();

		upposition = now_position * 1000;//扩大1000
		upvelocity = now_velocity * 1000;//扩大1000
		upcurrentq = current.q * 1000 + 1000;//扩大1000，偏置1000
		upcurrentd = current.d * 1000 + 1000;//扩大1000,偏置1000
		set_computer_value(SEND_FACT_CMD, CURVES_CH1, &upcurrentq, 1);     // 给通道1发Iq
		set_computer_value(SEND_FACT_CMD, CURVES_CH2, &upcurrentd, 1);     // 给通道2发Id
		set_computer_value(SEND_FACT_CMD, CURVES_CH3, &upvelocity, 1);     // 给通道3发送速度值
		set_computer_value(SEND_FACT_CMD, CURVES_CH4, &upposition, 1);     // 给通道4发送位置值
		
		
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
//		my_delay_ms(100);
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
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
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

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */

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
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
