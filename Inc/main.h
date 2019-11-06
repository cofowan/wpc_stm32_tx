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
#define MYP			( 0.35f )
#define MYI			( 0.40f )
#define MYD			( 0.00f )
#define TARGETVOL   ( 62.0f )
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
