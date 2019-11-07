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
/*	4.5A�����л�ʱ�϶�������ж��źţ������յ��ж��ź�ʱ��
	1��	���Ƚ�Ƶ���趨Ϊ84K,Ȼ�󱣳�2���ӣ�
		֮��������յ��ĵ�������ѹ���ж��źţ�
		��û�з����ж��ź��ˣ������㹻��Ӧ��
											Ȼ��������
											����<2.5A,����<3.5A,����<5A���ζ�׼��ά������
											����ֱ��Ӧ�Ŷ�Ӧ��pid���趨��ѹ����
		����Ȼ�����ж��źţ���ִ��2
		
	2����Ƶ�ʵ���Ϊ83K,Ȼ�󱣳�2���ӣ�֮����������ѹ���ж��źţ�
		��û�в����ж��źţ��������ڹ���Ӧ��
											Ȼ��������
											����<2.5A,����<3.5A,����<5A���ζ�׼��ά������
											����ֱ��Ӧ�Ŷ�Ӧ��pid���趨��ѹ����
		����Ȼ�����ж��źţ�һ���Կ��²�Ӧ���������£����ܳ������ˣ���������ˣ�
		��ʱLED�Ƽ�˸0.5���ӣ�ֹͣ����10���ӣ�ʮ��֮���ٸ�λ������
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
