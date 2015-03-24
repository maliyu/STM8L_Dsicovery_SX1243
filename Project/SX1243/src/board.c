/******************************************************************************
 * Project        : STM8L_Discovery_SX1278_SFM1L
 * File           : board.c
 * Copyright      : 2014 Yosun Singapore Pte Ltd
 ******************************************************************************
  Change History:

    Version 1.0 - Sep 2014
    > Initial revision

******************************************************************************/
#if defined(STM8S003)
#include "stm8s.h"
//#include "stm8s_clk.h"
//#include "stm8s_gpio.h"
//#include "stm8s_uart1.h"
#elif defined(STM8L15X_MD)
#include "stm8l15x.h"
#include "stm8l15x_clk.h"
#include "stm8l15x_gpio.h"
#include "stm8l15x_usart.h"
#endif

#define ISUNSIGNED(a) (a>0 && ~a>0)

void Board_Init(void)
{
#if defined(STM8S003)
  /*Clock configuration fmaster = 16MHz*/
  CLK_SYSCLKConfig(CLK_PRESCALER_HSIDIV1);
  CLK_SYSCLKConfig(CLK_PRESCALER_CPUDIV1);
  
  /* f_ck_cnt = 16Mhz/16 = 1MHz */
  /* count up mode */
  /* 1000/1MHz = 1ms */
  TIM1_TimeBaseInit(15, TIM1_COUNTERMODE_UP, 999, 0);
  TIM1_SetCounter(0);
  TIM1_ARRPreloadConfig(DISABLE);
  TIM1_ITConfig(TIM1_IT_UPDATE, ENABLE);
  TIM1_Cmd(ENABLE);
  
  /* UART init */    
  CLK_PeripheralClockConfig(CLK_PERIPHERAL_UART1, ENABLE);
  GPIO_ExternalPullUpConfig(GPIOD, GPIO_PIN_5, ENABLE);
  GPIO_ExternalPullUpConfig(GPIOD, GPIO_PIN_6, ENABLE);
  UART1->CR2 = 0x24;
  UART1->SR = 0;
  UART1->CR1 = 0;
  UART1->CR3 = 0;   
  // baud rate 115200
  // Actual baud rate may be 125000. So please set it as 128000 in Windows
  UART1->BRR1 = 0x08; 
  UART1->BRR2 = 0x0B;
  // baud rate 9600
  //UART1->BRR1 = 0x68; 
  //UART1->BRR2 = 0x03;
  
  /* Configure PD0 (LED1) as output push-pull low (led switched on) */
  GPIO_Init(GPIOD, GPIO_PIN_0, GPIO_MODE_OUT_PP_HIGH_FAST);
  //GPIO_Init(GPIOD, GPIO_PIN_0, GPIO_MODE_OUT_PP_LOW_FAST);
#elif defined(STM8L15X_MD)
  /* Internal clock */
  CLK_SYSCLKSourceConfig(CLK_SYSCLKSource_HSI);
  CLK_SYSCLKDivConfig(CLK_SYSCLKDiv_1);
  
  /* UART init */
  CLK_PeripheralClockConfig(CLK_Peripheral_USART1,ENABLE);
  GPIO_ExternalPullUpConfig(GPIOC, GPIO_Pin_2, ENABLE);
  GPIO_ExternalPullUpConfig(GPIOC, GPIO_Pin_3, ENABLE);
  /* Enable receiver interrupt */
  USART1->CR2 = 0x24;
  USART1->SR = 0;
  USART1->CR1 = 0;
  USART1->CR3 = 0;   
  // baud rate 115200
  //USART1->BRR1 = 0x08; 
  //USART1->BRR2 = 0x0B;
  // baud rate 9600
  USART1->BRR1 = 0x68; 
  USART1->BRR2 = 0x03;
  
  /* LD4 LED blue */
  GPIO_Init(GPIOC, GPIO_Pin_7, GPIO_Mode_Out_PP_High_Fast);
  /* LD3 LED green */
  GPIO_Init(GPIOE, GPIO_Pin_7, GPIO_Mode_Out_PP_High_Fast);
#endif
}

void LoRaRX_Indicate(void)
{
#if defined(STM8L15X_MD)
  GPIO_ToggleBits(GPIOC, GPIO_Pin_7);
#elif defined(STM8S003)
  GPIO_WriteReverse(GPIOD, GPIO_PIN_0);
#endif
}

void Uart_Prints(uint8_t *p_data, uint16_t length)
{
  if(!ISUNSIGNED(length))
  {
    return;
  }
  
  if(p_data == 0)
  {
    return;
  }
  
  while(length--)
  {
#if defined(STM8L15X_MD)
    USART1->CR2 |= (1<<3);
    while((USART1->SR & USART_FLAG_TXE) == RESET);
    USART1->DR = *(p_data++);
    while((USART1->SR & USART_FLAG_TC) == RESET);
    USART1->CR2 &= ~(1<<3);
#elif defined(STM8S003)
    UART1->CR2 |= (1<<3);
    while((UART1->SR & UART1_FLAG_TXE) == RESET);
    UART1->DR = *(p_data++);
    while((UART1->SR & UART1_FLAG_TC) == RESET);
    UART1->CR2 &= ~(1<<3);
#endif
  }
}

#if 0
void EEPROM_Write(uint16_t address, uint8_t *p_data, uint16_t len)
{
  if(!(ISUNSIGNED(len)))
  {
    return;
  }
  
  if(p_data == 0)
  {
    return;
  }
  
  //FLASH_Unlock(FLASH_MEMTYPE_DATA);
  /* Warning: keys are reversed on data memory !!! */
  FLASH->DUKR = 0xAE; 
  FLASH->DUKR = 0x56;
  //FLASH_ProgramByte(FIRMWARE_VERSION_ADDRESS, 0x55);
  while(len--)
  {
    *(PointerAttr uint8_t*)(address++) = (*p_data++);
  }
  //FLASH_Lock(FLASH_MEMTYPE_DATA);
  FLASH->IAPSR &= (uint8_t)(~(1<<3));
}

void EEPROM_Read(uint16_t address, uint8_t *p_data, uint16_t len)
{
  if(!(ISUNSIGNED(len)))
  {
    return;
  }
  
  if(p_data == 0)
  {
    return;
  }
  
  while(len--)
  {
    (*p_data++) = *(PointerAttr uint8_t*)(address++);
  }
}
#endif
