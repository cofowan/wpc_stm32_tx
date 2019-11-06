/*
 * FreeRTOS Kernel V10.2.1
 * Copyright (C) 2019 Amazon.com, Inc. or its affiliates.  All Rights Reserved.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy of
 * this software and associated documentation files (the "Software"), to deal in
 * the Software without restriction, including without limitation the rights to
 * use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of
 * the Software, and to permit persons to whom the Software is furnished to do so,
 * subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS
 * FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR
 * COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER
 * IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
 * CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 *
 * http://www.FreeRTOS.org
 * http://aws.amazon.com/freertos
 *
 * 1 tab == 4 spaces!
 */

/*
	BASIC INTERRUPT DRIVEN SERIAL PORT DRIVER FOR UART0.
*/

/* Scheduler includes. */
#include "FreeRTOS.h"
#include "queue.h"
#include "semphr.h"

/* Library includes. */
//#include "stm32f10x_lib.h"
#include "stm32f1xx_hal.h"
#include "stdio.h"
/* Demo application includes. */
#include "serial.h"
/*-----------------------------------------------------------*/

/* Misc defines. */
#define serINVALID_QUEUE				( ( QueueHandle_t ) 0 )
#define serNO_BLOCK						( ( TickType_t ) 0 )
#define serTX_BLOCK_TIME				( 40 / portTICK_PERIOD_MS )

/*-----------------------------------------------------------*/

/* The queue used to hold received characters. */
static QueueHandle_t xRxedChars;
static QueueHandle_t xCharsForTx;

/*-----------------------------------------------------------*/

/* UART interrupt handler. */
void vUARTInterruptHandler( void );
//extern void USART2_IRQHandler(void);
UART_HandleTypeDef USART_InitStructure;
/*-----------------------------------------------------------*/

/*
 * See the serial2.h header file.
 */
xComPortHandle xSerialPortInitMinimal( unsigned long ulWantedBaud, unsigned portBASE_TYPE uxQueueLength )
{
xComPortHandle xReturn;
//USART_InitTypeDef USART_InitStructure;
	//UART_HandleTypeDef USART_InitStructure;
//NVIC_InitTypeDef NVIC_InitStructure;
GPIO_InitTypeDef GPIO_InitStructure;

	/* Create the queues used to hold Rx/Tx characters. */
	xRxedChars = xQueueCreate( uxQueueLength, ( unsigned portBASE_TYPE ) sizeof( signed char ) );
	xCharsForTx = xQueueCreate( uxQueueLength + 1, ( unsigned portBASE_TYPE ) sizeof( signed char ) );
	
	/* If the queue/semaphore was created correctly then setup the serial port
	hardware. */
	if( ( xRxedChars != serINVALID_QUEUE ) && ( xCharsForTx != serINVALID_QUEUE ) )
	{
		/* Enable USART1 clock */
//		//RCC_APB2PeriphClockCmd( RCC_APB2Periph_USART1 | RCC_APB2Periph_GPIOA, ENABLE );	
//		 __HAL_RCC_GPIOA_CLK_ENABLE();
//		
//		
//		/* Configure USART1 Rx (PA10) as input floating */
//		//GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
//		GPIO_InitStructure.Pin = GPIO_PIN_3;
//		//GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
//		GPIO_InitStructure.Mode = GPIO_MODE_IT_FALLING;
//		//GPIO_Init( GPIOA, &GPIO_InitStructure );
//		HAL_GPIO_Init( GPIOA, &GPIO_InitStructure );
//		
//		
//		__HAL_RCC_USART2_CLK_ENABLE();
//		/* Configure USART1 Tx (PA9) as alternate function push-pull */
//		//GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
//		GPIO_InitStructure.Pin = GPIO_PIN_2;
//		//GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
//		GPIO_InitStructure.Speed = GPIO_SPEED_FREQ_HIGH;
//		//GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
//		GPIO_InitStructure.Mode = GPIO_MODE_AF_PP;
//		//GPIO_Init( GPIOA, &GPIO_InitStructure );
//		HAL_GPIO_Init( GPIOA, &GPIO_InitStructure );
//		
//		USART_InitStructure.Instance = USART2;
//		//USART_InitStructure.USART_BaudRate = ulWantedBaud;
//		USART_InitStructure.Init.BaudRate = ulWantedBaud;
//		//USART_InitStructure.USART_WordLength = USART_WordLength_8b;
//		USART_InitStructure.Init.WordLength = UART_WORDLENGTH_8B;
//		//USART_InitStructure.USART_StopBits = USART_StopBits_1;
//		USART_InitStructure.Init.StopBits = UART_STOPBITS_1;
//		//USART_InitStructure.USART_Parity = USART_Parity_No ;
//		USART_InitStructure.Init.Parity = UART_PARITY_NONE;
//		//USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
//		USART_InitStructure.Init.HwFlowCtl = UART_HWCONTROL_NONE;
//		//USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
//		USART_InitStructure.Init.Mode = UART_MODE_TX_RX;
//		//USART_InitStructure.USART_Clock = USART_Clock_Disable;
//		// ??
//		//USART_InitStructure.USART_CPOL = USART_CPOL_Low;
//		// ??
//		//USART_InitStructure.USART_CPHA = USART_CPHA_2Edge;
//		// ??
//		//USART_InitStructure.USART_LastBit = USART_LastBit_Disable;
//		// ??
//		USART_InitStructure.Init.OverSampling = UART_OVERSAMPLING_16;
//		//USART_Init( USART1, &USART_InitStructure );
//		 HAL_UART_Init( &USART_InitStructure );
//		
//		
//		//USART_ITConfig( USART1, USART_IT_RXNE, ENABLE );
//		__HAL_UART_ENABLE_IT( &USART_InitStructure, UART_IT_RXNE );
//		
//		//NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQChannel;
//		//NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = configLIBRARY_KERNEL_INTERRUPT_PRIORITY;
//		//NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
//		//NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
//		//NVIC_Init( &NVIC_InitStructure );
//		HAL_NVIC_SetPriority(USART2_IRQn, 5, 0);
//		HAL_NVIC_EnableIRQ(USART2_IRQn);
//		
//		//USART_Cmd( USART1, ENABLE );	
//			__HAL_UART_ENABLE( &USART_InitStructure );
		
		
	}
	else
	{
		xReturn = ( xComPortHandle ) 0;
	}

	/* This demo file only supports a single port but we have to return
	something to comply with the standard demo header file. */
	return xReturn;
}
/*-----------------------------------------------------------*/

signed portBASE_TYPE xSerialGetChar( xComPortHandle pxPort, signed char *pcRxedChar, TickType_t xBlockTime )
{
	/* The port handle is not required as this driver only supports one port. */
	( void ) pxPort;

	/* Get the next character from the buffer.  Return false if no characters
	are available, or arrive before xBlockTime expires. */
	if( xQueueReceive( xRxedChars, pcRxedChar, xBlockTime ) )
	{
		return pdTRUE;
	}
	else
	{
		return pdFALSE;
	}
}
/*-----------------------------------------------------------*/

void vSerialPutString( xComPortHandle pxPort, const signed char * const pcString, unsigned short usStringLength )
{
signed char *pxNext;

	/* A couple of parameters that this port does not use. */
	( void ) usStringLength;
	( void ) pxPort;

	/* NOTE: This implementation does not handle the queue being full as no
	block time is used! */

	/* The port handle is not required as this driver only supports UART1. */
	( void ) pxPort;

	/* Send each character in the string, one at a time. */
	pxNext = ( signed char * ) pcString;
	while( *pxNext )
	{
		xSerialPutChar( pxPort, *pxNext, serNO_BLOCK );
		pxNext++;
		
	}
}
/*-----------------------------------------------------------*/
#define  u16 uint16_t
#define  u32 uint32_t
#define  u8 uint8_t
#define USART_IT_Mask             ((u16)0x001F)  /* USART Interrupt Mask */
void USART_ITConfig(USART_TypeDef* USARTx, u16 USART_IT, FunctionalState NewState)
{
  u32 usartreg = 0x00, itpos = 0x00, itmask = 0x00;
  u32 address = 0x00;

  /* Check the parameters */
 // assert(IS_USART_CONFIG_IT(USART_IT));  
 // assert(IS_FUNCTIONAL_STATE(NewState));
  
  /* Get the USART register index */
  usartreg = (((u8)USART_IT) >> 0x05);

  /* Get the interrupt position */
  itpos = USART_IT & USART_IT_Mask;

  itmask = (((u32)0x01) << itpos);
  address = *(u32*)&(USARTx);

  if (usartreg == 0x01) /* The IT  is in CR1 register */
  {
    address += 0x0C;
  }
  else if (usartreg == 0x02) /* The IT  is in CR2 register */
  {
    address += 0x10;
  }
  else /* The IT  is in CR3 register */
  {
    address += 0x14; 
  }
  if (NewState != DISABLE)
  {
    *(u32*)address  |= itmask;
  }
  else
  {
    *(u32*)address &= ~itmask;
  }
}
#define USART_IT_TXE                         ((u16)0x0727)
signed portBASE_TYPE xSerialPutChar( xComPortHandle pxPort, signed char cOutChar, TickType_t xBlockTime )
{	
signed portBASE_TYPE xReturn;

	if( xQueueSend( xCharsForTx, &cOutChar, xBlockTime ) == pdPASS )
	{	printf("QQ\r\n");
		xReturn = pdPASS;
		USART_ITConfig( USART1, USART_IT_TXE, ENABLE );
		//__HAL_UART_ENABLE_IT( (UART_HandleTypeDef*)USART2, UART_IT_TXE );//到了这里跑死了！！！！！！！！
		printf("QQ1\r\n");
	}
	else
	{
		xReturn = pdFAIL;
	}

	return xReturn;
}
/*-----------------------------------------------------------*/

void vSerialClose( xComPortHandle xPort )
{
	/* Not supported as not required by the demo application. */
}
/*-----------------------------------------------------------*/

void vUARTInterruptHandler( void )
//void	USART2_IRQHandler(void)
{
portBASE_TYPE xHigherPriorityTaskWoken = pdFALSE;
char cChar;
	
	//if( USART_GetITStatus( USART1, USART_IT_TXE ) == SET )
	if( __HAL_UART_GET_IT_SOURCE( &USART_InitStructure, UART_IT_TXE ) == SET )
	{ 
		/* The interrupt was caused by the THR becoming empty.  Are there any
		more characters to transmit? */
		if( xQueueReceiveFromISR( xCharsForTx, &cChar, &xHigherPriorityTaskWoken ) == pdTRUE )
		{
			/* A character was retrieved from the queue so can be sent to the
			THR now. */
			//USART_SendData( USART1, cChar );
			HAL_UART_Transmit_IT( &USART_InitStructure, ( uint8_t * )&cChar, 1);
		}
		else
		{
			//USART_ITConfig( USART1, USART_IT_TXE, DISABLE );	
				__HAL_UART_DISABLE_IT( &USART_InitStructure, UART_IT_TXE );
		}		
	}
	
	//if( USART_GetITStatus( USART1, USART_IT_RXNE ) == SET )
	if( __HAL_UART_GET_IT_SOURCE( &USART_InitStructure, UART_IT_RXNE ) == SET )
	{
		//cChar = USART_ReceiveData( USART1 );
		HAL_UART_Receive_IT( &USART_InitStructure, ( uint8_t * )&cChar, 1);
		xQueueSendFromISR( xRxedChars, &cChar, &xHigherPriorityTaskWoken );
	}	
	
	portEND_SWITCHING_ISR( xHigherPriorityTaskWoken );
}





	
