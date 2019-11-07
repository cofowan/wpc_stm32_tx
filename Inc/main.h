/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f1xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include"stdlib.h"
typedef struct
{
	uint16_t voltage;
	uint16_t current;
	uint8_t over_load_rx;
	uint8_t keep_live;
}wireless_charge_data_t;
/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */
#include"string.h"
#define TIM1_PERIOD_MIN (768UL) //83.3KHz 640/0.833 = 768
#define TIM1_PERIOD_MAX (727UL) //88KHz 640/0.88 = 727
#define TIM1_PERIOD_STEP (1UL)
#define TIM1_PERIOD_85KHz (753UL) //85KHz 640/0.85 = 753
#define TIM1_PERIOD_84KHz (761UL) //84KHz 640/0.84 = 761

#define RES_DWON 	( 30.0f )
#define RES_UP 		( 1000.0f )
#define RES_RATE	( ( RES_DWON / ( RES_DWON + RES_UP ) ) )
#define REF_VOL		( 3.30f )
#define ADC_REF		( ( ( RES_RATE * 4096 ) / REF_VOL ) )

typedef enum
{
	CUR_NULL,
	CUR_LIGHT_LOAD,
	CUR_UNDER_4A,
	CUR_UNDER_5A,
	CUR_ERROR
}CURRENT_STATUS;
typedef enum
{
	VOL_NULL,
	VOL_UNDER_43V,
	VOL_43V_46V,
	VOL_47V_61V,
	VOL_62V_68V,
	VOL_69V_90V,
	VOL_ABOVE_90V,
	VOL_ERROR
}VOLTAGE_STATUS;
typedef struct 
{

	uint8_t overload 	: 1;
	uint8_t bleonline 	: 1;
	uint8_t voltage		: 3;
	uint8_t current		: 3;
	
}SignalData;
typedef struct
{
	uint16_t vol;
	uint16_t cur;
	uint16_t overload;
}data_t;
typedef struct
{
	data_t data;
	uint16_t crc;
	char	 ch;
}vol_cur_t;
/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */
extern TIM_HandleTypeDef htim1;
__STATIC_INLINE void pwm_set_up(void);
__STATIC_INLINE void pwm_set_down(void);
__STATIC_INLINE void pwm_set_5x_down(void);
__STATIC_INLINE void pwm_set_standby(void);
__STATIC_INLINE void pwm_set_85KHz(void);
__STATIC_INLINE void pwm_set_84KHz(void);
__STATIC_INLINE void pwm_set_83KHz(void);
__STATIC_INLINE void pwm_set_88KHz(void);
__STATIC_INLINE void pwm_set_shutdown(void);
__STATIC_INLINE  uint32_t pwm_get_peroid(void);
__STATIC_INLINE  float pwm_get_freq(void);
__STATIC_INLINE void pwm_set_update(uint16_t data);
__STATIC_INLINE void pwm_set_up(void)
{
	if(htim1.State == HAL_TIM_STATE_READY)
	 {
		if(htim1.Instance->ARR > TIM1_PERIOD_MAX) //大于581
		{	
			htim1.Instance->ARR -= TIM1_PERIOD_STEP;
			htim1.Instance->CCR1 = htim1.Instance->ARR / 2;
		}
		else{
			htim1.Instance->ARR = TIM1_PERIOD_MAX;
			htim1.Instance->CCR1 = TIM1_PERIOD_MAX / 2;
		}
	}
	 
}
__STATIC_INLINE void pwm_set_down(void)
{
	 
	 if(htim1.State == HAL_TIM_STATE_READY)
	 {
		 if(htim1.Instance->ARR < TIM1_PERIOD_MIN) //小于639
		{	
			htim1.Instance->ARR += TIM1_PERIOD_STEP;
			htim1.Instance->CCR1 = htim1.Instance->ARR / 2;
		}
		else{
			htim1.Instance->ARR = TIM1_PERIOD_MIN;
			htim1.Instance->CCR1 = TIM1_PERIOD_MIN / 2;
		}
	 }
}
__STATIC_INLINE void pwm_set_5x_down(void)
{
	 
	 if(htim1.State == HAL_TIM_STATE_READY)
	 {
		 if(htim1.Instance->ARR < TIM1_PERIOD_MIN) //小于639
		{	
			htim1.Instance->ARR += TIM1_PERIOD_STEP * 5;
			if(htim1.Instance->ARR > TIM1_PERIOD_MIN) htim1.Instance->ARR = TIM1_PERIOD_MIN;
			htim1.Instance->CCR1 = htim1.Instance->ARR / 2;
		}
		else{
			htim1.Instance->ARR = TIM1_PERIOD_MIN;
			htim1.Instance->CCR1 = TIM1_PERIOD_MIN / 2;
		}
	 }
}
__STATIC_INLINE void pwm_set_standby(void)
{
	if(htim1.State == HAL_TIM_STATE_READY)
	 {
		
		htim1.Instance->ARR = TIM1_PERIOD_MAX;
		htim1.Instance->CCR1 = htim1.Instance->ARR / 2;
		
		
	}
}
__STATIC_INLINE void pwm_set_85KHz(void)
{
	if(htim1.State == HAL_TIM_STATE_READY)
	 {
		
		htim1.Instance->ARR = TIM1_PERIOD_85KHz;
		htim1.Instance->CCR1 = htim1.Instance->ARR / 2;
		
		
	}
}
__STATIC_INLINE void pwm_set_84KHz(void)
{
	if(htim1.State == HAL_TIM_STATE_READY)
	 {
		
		htim1.Instance->ARR = TIM1_PERIOD_84KHz;
		htim1.Instance->CCR1 = htim1.Instance->ARR / 2;
		
		
	}
}
__STATIC_INLINE void pwm_set_83KHz(void)
{
	if(htim1.State == HAL_TIM_STATE_READY)
	 {
		
		htim1.Instance->ARR = TIM1_PERIOD_MIN;
		htim1.Instance->CCR1 = htim1.Instance->ARR / 2;
		
		
	}
}
__STATIC_INLINE void pwm_set_update(uint16_t data)
{
	if(htim1.State == HAL_TIM_STATE_READY)
	 {
		
		htim1.Instance->ARR = data;
		htim1.Instance->CCR1 = htim1.Instance->ARR / 2;

	}
}
__STATIC_INLINE void pwm_set_88KHz(void)
{
	if(htim1.State == HAL_TIM_STATE_READY)
	 {
		
		htim1.Instance->ARR = TIM1_PERIOD_MAX;
		htim1.Instance->CCR1 = htim1.Instance->ARR / 2;
		
		
	}
}
__STATIC_INLINE void pwm_set_shutdown(void)
{
	
	HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_1);
    HAL_TIMEx_PWMN_Stop(&htim1, TIM_CHANNEL_1);
}
__STATIC_INLINE void pwm_set_start(void)
{
	
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
    HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_1);
}
__STATIC_INLINE  uint32_t pwm_get_peroid(void)
{
	uint32_t temp = htim1.Instance->ARR ;
	return temp;

}

__STATIC_INLINE  float pwm_get_freq(void)
{
	float temp = htim1.Instance->ARR;
	return ( 64000.0f / temp );
}
/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define LED_BLUE_Pin GPIO_PIN_1
#define LED_BLUE_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */
//#define MYP			( 0.30f )
//#define MYI			( 0.20f )
//#define MYD			( 0.00f )
/*** 63V @3.5A ***/
//#define MYP			( 0.235f )
//#define MYI			( 0.240f )
//#define MYD			( 0.00f )
//#define TARGETVOL   ( 63.0f )
/*****/
/*** 70V @4A ***/
//#define MYP			( 0.135f )
//#define MYI			( 0.140f )
//#define MYD			( 0.00f )
//#define TARGETVOL   ( 70.0f )
/*****/
/*** 80V @4.5A ***/
#define MYP			( 0.045f )
#define MYI			( 0.025f )
#define MYD			( 0.00f )
#define TARGETVOL   ( 80.0f )
/*****/
/*	4.5A以上切换时肯定会产生中断信号，当接收到中断信号时，
	1：	首先将频率设定为84K,然后保持2秒钟，
		之后检测其接收到的电流，电压，中断信号：
		若没有发生中断信号了，表明足够供应：
											然后查电流：
											电流<2.5A,电流<3.5A,电流<5A依次对准二维数组表格，
											里面分别对应着对应的pid与设定电压参数
		若依然产生中断信号，则执行2
		
	2：将频率调节为83K,然后保持2秒钟，之后查电流，电压，中断信号：
		若没有产生中断信号，表明终于够供应：
											然后查电流：
											电流<2.5A,电流<3.5A,电流<5A依次对准二维数组表格，
											里面分别对应着对应的pid与设定电压参数
		若依然产生中断信号，一般性况下不应还发生此事，可能出问题了，比如过载了！
		此时LED灯间烁0.5秒钟，停止工作10秒钟，十秒之后再复位开机。
*/
/* USER CODE END Private defines */
/*** 50V @2.5A 1.84V ***/
//#define MYP			( 0.175 )
//#define MYI			( 0.125f )
//#define MYD			( 0.00f )
//#define TARGETVOL   ( 50.0f )
/*****/
//4.0A = 1.95V
//3.5A = 1.913V
//3.0A = 1.875V
//2.5A = 1.84V
//2.0A = 1.80V
//1.5A = 1.76V
//1.0A = 1.73V
//0.5A = 1.69V
//0.0A = 1.65V
/* USER CODE END Private defines */
#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
