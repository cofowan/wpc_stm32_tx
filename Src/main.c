/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
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
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

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
TIM_HandleTypeDef htim1;

UART_HandleTypeDef huart2;
UART_HandleTypeDef huart3;

osThreadId defaultTaskHandle;
/* USER CODE BEGIN PV */
#include "inc_pid_controller.h"
typedef enum
{
	DOT_NOTHING,
	UP,
	DOWN,
	UP_CONTINU,
	DOWN_CONTINU
}KEY_ACTION;
KEY_ACTION myKeyActiion = DOT_NOTHING;
typedef enum
{
		KEY_NOT,
		KEY_PRESSED,
		KEY_CONTINU,
		KEY_RELEASED
}KEY_STATUS_t;	
typedef void (*func)(void);
typedef struct{
	KEY_STATUS_t 	status;
	func 			key_pressed_handle;
	func 			key_relased_handle;
	func 			key_contuinu_handle;
	uint8_t 		key_value;
	uint32_t 		key_into_continu;
	uint16_t		key_into_continu_trigger_count;
	uint16_t		key_continu_interval;
}key_t;
void key_prosser_handle(key_t *key)
{
	if( (key->key_value != 0) && (key->status == KEY_NOT ) )
	{
		return;
	}
	switch(key->status)
	{
		case KEY_NOT:
			if(!key->key_value)
			{
				key->status = KEY_PRESSED;
			}				
			else
			{
				key->status = KEY_NOT;
				key->key_into_continu = 0;
			}
			break;
			
		case KEY_PRESSED:
			if(!key->key_value)
			{	
				key->key_into_continu++;
				if( (key->key_pressed_handle != NULL) && (key->key_into_continu < 2) ) 
				{
					key->key_pressed_handle();
				}
				
				if(key->key_into_continu > key->key_into_continu_trigger_count) key->status = KEY_CONTINU;
				
			}
			else
			{
				key->status = KEY_RELEASED;
			}				
			break;
			
		case KEY_CONTINU:
			if(!key->key_value)
			{	
				key->key_into_continu++;
				if( !(key->key_into_continu % key->key_continu_interval) )
				{
					if(key->key_contuinu_handle != NULL) key->key_contuinu_handle();
				}
			}
			else
			{	
				key->key_into_continu = 0;
				key->status = KEY_NOT;
			}
			break;
			
		case KEY_RELEASED:
			if(key->key_value)
			{
				if(key->key_relased_handle != NULL) key->key_relased_handle();
			}
			key->status = KEY_NOT;
			key->key_into_continu = 0;
			break;
			
		default:
			key->status = KEY_NOT;
			key->key_into_continu = 0;
			break;
	}
}


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM1_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_USART2_UART_Init(void);
void StartDefaultTask(void const * argument);

/* USER CODE BEGIN PFP */
//printf 打印函数，重新定向！定向指向pb2 swo这条gpio口，就可以利用ST-Link Utility来查看打印内容
#include <stdio.h>
#define ITM_Port8(n) 	( *( (volatile unsigned char *)( 0xE0000000 + 4*n ) ) )
#define ITM_Port16(n) 	( *( (volatile unsigned short *)( 0xE0000000 + 4*n ) ) )
#define ITM_Port32(n) 	( *( (volatile unsigned long *)( 0xE0000000 + 4*n ) ) )

#define DEMCR 			( *( (volatile unsigned long *)( 0xE000EDFC ) ) )
#define TRCENA 			0x01000000

struct __FILE { int handle; /* Add whatever needed */ };
FILE __stdout;
FILE __stdin;

int fputc(int ch, FILE *f) {
	/*
		if (DEMCR & TRCENA) {
			while (ITM_Port32(0) == 0);
			ITM_Port8(0) = ch;
		}
	*/
	//HAL_UART_Transmit(&huart1,(uint8_t*)ch,1,0xFFFF);
	//HAL_UART_Transmit_IT(&huart1,(uint8_t *)ch,1);
	/*上面二句都不能正常显示，下面三行能正常显示！*/
	while((USART2->SR&0X40)==0){}
    USART2->DR = (uint8_t) ch;      
	return(ch);
}
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
uint32_t myADC[2]={0};
static uint16_t ble_connect_flag = 0;
uint8_t RxData[100]={0};
uint8_t Index = 0;
uint8_t arr = 0;
uint8_t uart2_arr = 0;
inc_pid_controller_t pMyPID = NULL;
#include "FreeRTOS_CLI.h"
#include "serial.h"
vol_cur_t *pData = NULL;

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

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_TIM1_Init();
  MX_USART3_UART_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */

  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
  HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_1);
  HAL_UART_Receive_IT(&huart3,(uint8_t*)&arr,1);
  HAL_UART_Receive_IT(&huart2,(uint8_t*)&uart2_arr,1);
	//初始化pid
	//pMyPID = inc_pid_controller_create( 0.6f, 0.1f, 0, 20 );
	pMyPID = inc_pid_controller_create( MYP, MYI, MYD, 30 );
	pMyPID->controller.target = TARGETVOL;
	pMyPID->maximum = TIM1_PERIOD_MIN;//TIM1_PERIOD_MIN (768UL) --> 83.3KHz 640/0.833 = 768,作用：加大功率！
	pMyPID->minimum = TIM1_PERIOD_MAX;//TIM1_PERIOD_MAX (727UL) //88KHz 640/0.88 = 727，作用，减小功率！
	pMyPID->controller.enable = 1;
	
	
  /* USER CODE END 2 */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */

  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of defaultTask */
  osThreadDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 128);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

//vUARTCommandConsoleStart( 1024, 5 );

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* Start scheduler */
  osKernelStart();
  
  /* We should never get here as control is now taken by the scheduler */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
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

  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI_DIV2;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL16;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 727;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 16;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_ENABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED_BLUE_GPIO_Port, LED_BLUE_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin : PB0 */
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : LED_BLUE_Pin */
  GPIO_InitStruct.Pin = LED_BLUE_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(LED_BLUE_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_IRQn, 3, 0);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);

}

/* USER CODE BEGIN 4 */
void HAL_UART_TxCpltCallback  ( UART_HandleTypeDef *  huart ) 
{	//发送完成回调函数
	//HAL_GPIO_TogglePin(GPIOB,GPIO_PIN_0);
}

void HAL_UART_RxCpltCallback  ( UART_HandleTypeDef *  huart ) 
{	
	BaseType_t xHigherPriorityTaskWoken = pdFALSE;
	
	//接收完成中断回调函数
	if(huart == &huart3)
	{ //实测接收快时３０ｍｓ，慢时６５ｍｓ
		//HAL_GPIO_TogglePin(LED_BLUE_GPIO_Port,LED_BLUE_Pin);
		RxData[Index]=arr; //接收中断发生，存储接收值
		Index++;
		
		if( RxData[Index -1] == '\r' || RxData[Index -1] == '\n' || Index >=20 )
		{
		
			if( Index == sizeof(vol_cur_t) ) //从机发过来的数据格式
			{
				/* 
				特别注意：RxData在接收机串口发送到ble的数据流还正确，而给BLE rx端的串口接收到后，
				就变成最尾的一个字节跑到最前面来了，原因有待进一步查明，
				为了能正确读取数据流，这里特意加1！数据即能正确读取！！
				*/
				pData = (vol_cur_t *)(RxData + 1);
				
				vTaskNotifyGiveFromISR( defaultTaskHandle, &xHigherPriorityTaskWoken );
				portYIELD_FROM_ISR( xHigherPriorityTaskWoken );
			
			}	
			Index = 0;//完成接收后，重新清0，开启下次的接收
		}
		ble_connect_flag = 1; //串口接收正常代表蓝牙连接正常
		HAL_UART_Receive_IT(huart, &arr, 1);//再次开启usart1接收中断
		
		
	}
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	static uint16_t cnt = 0;
	if( GPIO_Pin == GPIO_PIN_0 ) //pb0产生中断信号，即tx端过压过载了
	{
		//立即将pwm 的频率调节到88khz
		pwm_set_standby();
		cnt++;
		if(cnt >= 10) //若最低功率还继续发生，则需要关闭pwm
		{
			cnt = 0;
			pwm_set_shutdown(); //关闭pwm
		}
			
	}
}
/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used 
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void const * argument)
{
    
    
    

  /* USER CODE BEGIN 5 */
	uint32_t val = 0;
	const TickType_t xPeriod = pdMS_TO_TICKS( 75 );
	float f = 0.0;
	vTaskDelay(500); //延时500ms才打开pwm
	pwm_set_start();
	
  /* Infinite loop */
  for(;;)
  {
	
	  val = ulTaskNotifyTake( pdFALSE, xPeriod );
	  if(val == 1) //正确原因的解除block;
	  {
		  if( pData != NULL )
		  {
				f = pData->data.vol / ADC_REF;
				if( pMyPID->controller.update(pMyPID,f) == RT_EOK ) //成功读取调用更新
				{
					pwm_set_update(pMyPID->controller.output);
				}
				pData = NULL;
		 }
	  }
	  else //时间溢出
	  {
		  if(ble_connect_flag != 1) //若蓝牙连接不正常
		  {
			  HAL_GPIO_WritePin(LED_BLUE_GPIO_Port,LED_BLUE_Pin,GPIO_PIN_SET); //关闭LED
			  pwm_set_standby(); //蓝牙若连接不正常，PWM设为88KHz
		  }
		  else //若蓝牙连接正常
		  {
			  ble_connect_flag = 0;
			  HAL_GPIO_WritePin(LED_BLUE_GPIO_Port,LED_BLUE_Pin,GPIO_PIN_RESET); //亮LED
			  
		  }
	  }
	  
	 
  }
  /* USER CODE END 5 */ 
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM3 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM3) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

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
