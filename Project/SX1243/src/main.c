/**
  ******************************************************************************
  * @file    Project/Template/main.c
  * @author  MCD Application Team
  * @version V1.3.0
  * @date    07/14/2010
  * @brief   Main program body
  ******************************************************************************
  * @copy
  *
  * THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
  * WITH CODING INFORMATION REGARDIN THEIR PRODUCTS IN ORDER FOR THEM TO SAVE
  * TIME. AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY
  * DIRECT, INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING
  * FROM THE CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE
  * CODING INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
  *
  * <h2><center>&copy; COPYRIGHT 2010 STMicroelectronics</center></h2>
  */ 
	
/* Includes ------------------------------------------------------------------*/
#if defined(STM8L15X_MD)
#include "stm8l15x.h"
//#include "stm8l15x_flash.h"
#elif defined(STM8S003)
#include "stm8s.h"
#endif
#include "task.h"
#include "sx1243.h"
#include "board.h"


/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* This Payload value is used only as an example */
static uint8_t ExamplePayload[16] = { 0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07,
                                 0x08, 0x09, 0x0A, 0x0B, 0x0C, 0x0D, 0x0E, 0x0F };

/* This Sync value is used only as an example */
static uint8_t ExampleSyncWord[4] = { 0x69, 0x81, 0x7E, 0x96 };

/* This parameter allows us to access the sx1243 structure directly from the main */
extern tTxParam TxParam;
/* Private function prototypes -----------------------------------------------*/

/* Private functions ---------------------------------------------------------*/

/**
  * @brief  Main program.
  * @param  None
  * @retval None
  */
void main(void)
{
  disableInterrupts();
  
  Board_Init();
  
  SX1243Init( );
  
  enableInterrupts();
  
  /* Infinite loop */
  while (1)
  {
  	// Send 1 packet at 9600 bps every time the button 1 is pressed using the default configuration
	TxParam.Delay = 0;
	TxParam.Repeat = 0;
	TxParam.Bitrate = 9600;
	SX1243SetTxSyncValue( ExampleSyncWord, (U8)sizeof( ExampleSyncWord ) );
	SX1243SetTxPacketBuffer( ExamplePayload, (U8)sizeof( ExamplePayload ) );
	SX1243Process( );
	StandardWait(250);
  }
}

/**
  * @brief  Inserts a delay time.
  * @param  nCount: specifies the delay time length.
  * @retval None
  */

#ifdef  USE_FULL_ASSERT

/**
  * @brief  Reports the name of the source file and the source line number
  *   where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t* file, uint32_t line)
{ 
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

  /* Infinite loop */
  while (1)
  {
  }
}
#endif

/**
  * @}
  */

/******************* (C) COPYRIGHT 2010 STMicroelectronics *****END OF FILE****/