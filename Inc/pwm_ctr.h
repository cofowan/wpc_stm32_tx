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


/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __PWM_CTR_H
#define __PWM_CTR_H

#ifdef __cplusplus
extern "C" {
#endif


#include "main.h"
#include "FreeRTOS.h"
#include "task.h"
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

#ifdef __cplusplus
}
#endif

#endif /* __PWM_CTR_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
