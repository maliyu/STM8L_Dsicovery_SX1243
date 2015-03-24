/******************************************************************************
 * Project        : STM8L_Discovery_SX1278_SFM1L
 * File           : sx1276.c
 * Copyright      : 2014 Yosun Singapore Pte Ltd
 ******************************************************************************
  Change History:

    Version 1.0 - Sep 2014
    > Initial revision

******************************************************************************/
#include <string.h>
#if defined(STM8S003)
#include "stm8s.h"
//#include "stm8s_spi.h"
#elif defined(STM8L15X_MD)
#include "stm8l15x.h"
#include "stm8l15x_gpio.h"
#include "stm8l15x_spi.h"
#include "stm8l15x_clk.h"
#include "stm8l15x_syscfg.h"
#endif
#include "sx1276.h"

static void SX1276InitIo( void );
static void SX1276Init( void );
static void SX1276Reset( void );
static void SX1276StartRx( void );
static void SX1276GetRxPacket( void *buffer, uint16_t *size );
static void SX1276SetTxPacket( const void *buffer, uint16_t size );
static uint32_t SX1276Process( void );
static void SX1276SetLoRaOn( void );
static uint8_t SX1276ReadDio0( void );
static uint8_t SX1276ReadDio1( void );
static uint8_t SX1276ReadDio2( void );
static uint8_t SX1276ReadDio3( void );
static uint8_t SX1276ReadDio4( void );
static uint8_t SX1276ReadDio5( void );
static uint8_t SX1276LoRaReadRxGain( void );
static void SX1276LoRaInit( void );
static void SX1276LoRaSetLoRaOn( void );
//static void SX1276LoRaSetSpreadingFactor( uint8_t factor );
//static void SX1276LoRaSetErrorCoding( uint8_t value );
//static void SX1276LoRaSetPacketCrcOn( bool enable );
//static void SX1276LoRaSetSignalBandwidth( uint8_t bw );
//static void SX1276LoRaSetImplicitHeaderOn( bool enable );
//static void SX1276LoRaSetSymbTimeout( uint16_t value );
//static void SX1276LoRaSetPayloadLength( uint8_t value );
//static void SX1276LoRaSetLowDatarateOptimize( bool enable );
//static void SX1276LoRaSetPAOutput( uint8_t outputPin );
//static void SX1276LoRaSetPa20dBm( bool enale );
//static void SX1276LoRaSetRFPower( int8_t power );
//static void SX1276LoRaSetNbTrigPeaks( uint8_t value );
static void SX1276LoRaSetRFState( uint8_t state );
static void SX1276LoRaGetRxPacket( void *buffer, uint16_t *size );
static void SX1276LoRaSetTxPacket( const void *buffer, uint16_t size );
static uint32_t SX1276LoRaProcess( void );
static void SX1276LoRaSetRFFrequency( uint32_t freq );
static void SX1276WriteRxTx( uint8_t txEnable );

//#define STM8_SOFT_SPI // Otherwise, SPI driver from STM lib would be used

#define GET_TICK_COUNT( )                           ( TickCounter )

// RXTX pin control
#define RXTX( txEnable )                            SX1276WriteRxTx( txEnable );

/*!
 * DIO state read functions mapping
 */
#define DIO0                                        SX1276ReadDio0( )
#define DIO1                                        SX1276ReadDio1( )
#define DIO2                                        SX1276ReadDio2( )
#define DIO3                                        SX1276ReadDio3( )
#define DIO4                                        SX1276ReadDio4( )
#define DIO5                                        SX1276ReadDio5( )

#if defined(STM8L15X_MD)
#define GPIO_WriteLow(a, b) GPIO_WriteBit(a, b, RESET)
#define GPIO_WriteHigh(a, b) GPIO_WriteBit(a, b, SET)
#define GPIO_ReadInputPin(a, b) GPIO_ReadInputDataBit(a, b)
#endif

static tRadioDriver RadioDriver;
static uint8_t SX1276Regs[0x71];
// System tick (1ms)
volatile uint32_t TickCounter = 0;

/*!
 * Precomputed signal bandwidth log values
 * Used to compute the Packet RSSI value.
 */
static const double SignalBwLog[] =
{
    3.8927900303521316335038277369285,  // 7.8 kHz
    4.0177301567005500940384239336392,  // 10.4 kHz
    4.193820026016112828717566631653,   // 15.6 kHz
    4.31875866931372901183597627752391, // 20.8 kHz
    4.4948500216800940239313055263775,  // 31.2 kHz
    4.6197891057238405255051280399961,  // 41.6 kHz
    4.795880017344075219145044421102,   // 62.5 kHz
    5.0969100130080564143587833158265,  // 125 kHz
    5.397940008672037609572522210551,   // 250 kHz
    5.6989700043360188047862611052755   // 500 kHz
};

static const double RssiOffsetLF[] =
{   // These values need to be specify in the Lab
    -155.0,
    -155.0,
    -155.0,
    -155.0,
    -155.0,
    -155.0,
    -155.0,
    -155.0,
    -155.0,
    -155.0,
};

static const double RssiOffsetHF[] =
{   // These values need to be specify in the Lab
    -150.0,
    -150.0,
    -150.0,
    -150.0,
    -150.0,
    -150.0,
    -150.0,
    -150.0,
    -150.0,
    -150.0,
};
/*!
 * Frequency hopping frequencies table
 */
static const int32_t HoppingFrequencies[] =
{
    916500000,
    923500000,
    906500000,
    917500000,
    917500000,
    909000000,
    903000000,
    916000000,
    912500000,
    926000000,
    925000000,
    909500000,
    913000000,
    918500000,
    918500000,
    902500000,
    911500000,
    926500000,
    902500000,
    922000000,
    924000000,
    903500000,
    913000000,
    922000000,
    926000000,
    910000000,
    920000000,
    922500000,
    911000000,
    922000000,
    909500000,
    926000000,
    922000000,
    918000000,
    925500000,
    908000000,
    917500000,
    926500000,
    908500000,
    916000000,
    905500000,
    916000000,
    903000000,
    905000000,
    915000000,
    913000000,
    907000000,
    910000000,
    926500000,
    925500000,
    911000000,
};

/*!
 * Rx management support variables
 */
static uint16_t RxPacketSize = 0;
static int8_t RxPacketSnrEstimate;
static double RxPacketRssiValue;
static uint8_t RxGain = 1;
static uint32_t RxTimeoutTimer = 0;
/*!
 * PacketTimeout Stores the Rx window time value for packet reception
 */
static uint32_t PacketTimeout;

/*!
 * SX1276 LoRa registers variable
 */
static tSX1276LR* SX1276LR;

/*!
 * RF state machine variable
 */
static uint8_t RFLRState = RFLR_STATE_IDLE;
/*!
 * Local RF buffer for communication support
 */
static uint8_t RFBuffer[RF_BUFFER_SIZE];

// Default settings
static tLoRaSettings LoRaSettings =
{
    532923000,        // RFFrequency
    20,               // Power
    9,                // SignalBw [0: 7.8kHz, 1: 10.4 kHz, 2: 15.6 kHz, 3: 20.8 kHz, 4: 31.2 kHz,
                      // 5: 41.6 kHz, 6: 62.5 kHz, 7: 125 kHz, 8: 250 kHz, 9: 500 kHz, other: Reserved]
    10,                // SpreadingFactor [6: 64, 7: 128, 8: 256, 9: 512, 10: 1024, 11: 2048, 12: 4096  chips]
    2,                // ErrorCoding [1: 4/5, 2: 4/6, 3: 4/7, 4: 4/8]
    TRUE,             // CrcOn [0: OFF, 1: ON]
    FALSE,            // ImplicitHeaderOn [0: OFF, 1: ON]
    FALSE,                // RxSingleOn [0: Continuous, 1 Single]
    FALSE,                // FreqHopOn [0: OFF, 1: ON]
    4,                // HopPeriod Hops every frequency hopping period symbols
    100,              // TxPacketTimeout
    100,              // RxPacketTimeout
    17,              // PayloadLength (used for implicit header mode)
};

/*!
 * Tx management support variables
 */
static uint16_t TxPacketSize = 0;

#ifdef STM8_SOFT_SPI
void SPI_write(uint8_t addr, uint8_t *p_data, uint8_t length)
{
  uint8_t tmp = addr | 0x80;
  uint8_t i,j;
 
    GPIO_WriteLow(SPI_NSS_PORT, SPI_NSS_PIN);
    GPIO_WriteLow(SPI_SCK_PORT, SPI_SCK_PIN);
    
    for(i=8;i!=0;i--)
    {
      GPIO_WriteLow(SPI_SCK_PORT, SPI_SCK_PIN);
      if(tmp & 0x80)
      {
        GPIO_WriteHigh(SPI_MOSI_PORT, SPI_MOSI_PIN);
      }
      else
      {
        GPIO_WriteLow(SPI_MOSI_PORT, SPI_MOSI_PIN);
      }
      asm("nop");
      asm("nop");
      GPIO_WriteHigh(SPI_SCK_PORT, SPI_SCK_PIN);
      asm("nop");
      tmp <<= 1;
    }
    
    GPIO_WriteLow(SPI_SCK_PORT, SPI_SCK_PIN);
    GPIO_WriteHigh(SPI_MOSI_PORT, SPI_MOSI_PIN);
    
    for(j=0;j<length;j++)
    {
      tmp = p_data[j];
      
      GPIO_WriteLow(SPI_NSS_PORT, SPI_NSS_PIN);
      GPIO_WriteLow(SPI_SCK_PORT, SPI_SCK_PIN);
      
      for(i=8;i!=0;i--)
      {
        GPIO_WriteLow(SPI_SCK_PORT, SPI_SCK_PIN);
        if(tmp & 0x80)
        {
          GPIO_WriteHigh(SPI_MOSI_PORT, SPI_MOSI_PIN);
        }
        else
        {
          GPIO_WriteLow(SPI_MOSI_PORT, SPI_MOSI_PIN);
        }
        asm("nop");
        asm("nop");
        GPIO_WriteHigh(SPI_SCK_PORT, SPI_SCK_PIN);
        asm("nop");
        tmp <<= 1;
      }
      
      GPIO_WriteLow(SPI_SCK_PORT, SPI_SCK_PIN);
      GPIO_WriteHigh(SPI_MOSI_PORT, SPI_MOSI_PIN);
    }
    GPIO_WriteHigh(SPI_NSS_PORT, SPI_NSS_PIN);
}

void SPI_read(uint8_t addr, uint8_t *p_data, uint8_t length)
{
  uint8_t tmp = addr;
  uint8_t i,j;
  
    GPIO_WriteLow(SPI_NSS_PORT, SPI_NSS_PIN);
    GPIO_WriteLow(SPI_SCK_PORT, SPI_SCK_PIN);
    
    for(i=8;i!=0;i--)
    {
      GPIO_WriteLow(SPI_SCK_PORT, SPI_SCK_PIN);
      if(tmp & 0x80)
      {
        GPIO_WriteHigh(SPI_MOSI_PORT, SPI_MOSI_PIN);
      }
      else
      {
        GPIO_WriteLow(SPI_MOSI_PORT, SPI_MOSI_PIN);
      }
      asm("nop");
      asm("nop");
      GPIO_WriteHigh(SPI_SCK_PORT, SPI_SCK_PIN);
      asm("nop");
      tmp <<= 1;
    }
    
    GPIO_WriteLow(SPI_SCK_PORT, SPI_SCK_PIN);
    GPIO_WriteHigh(SPI_MOSI_PORT, SPI_MOSI_PIN);
    
    for(j=0;j<length;j++)
    {
      GPIO_WriteLow(SPI_NSS_PORT, SPI_NSS_PIN);
      GPIO_WriteHigh(SPI_MOSI_PORT, SPI_MOSI_PIN);
      
      tmp = 0;
      for(i=8;i!=0;i--)
      {
        GPIO_WriteLow(SPI_SCK_PORT, SPI_SCK_PIN);
        asm("nop");
        asm("nop");
        tmp <<= 1;
        GPIO_WriteHigh(SPI_SCK_PORT, SPI_SCK_PIN);
        if(GPIO_ReadInputPin(SPI_MISO_PORT, SPI_MISO_PIN))
        {
          tmp |= 0x01;
        }
        else
        {
          tmp |= 0x00;
        }
      }
      GPIO_WriteLow(SPI_SCK_PORT, SPI_SCK_PIN);
      p_data[j] = tmp;
    }
    GPIO_WriteHigh(SPI_NSS_PORT, SPI_NSS_PIN);
  
}
#else // SPI driver from STM lib
void SPI_read(uint8_t addr, uint8_t *p_data, uint8_t length)
{  
  if(length <= 0)
  {
    return;
  }

#if defined(STM8L15X_MD)
  GPIO_WriteLow(SPI_NSS_PORT, SPI_NSS_PIN);

  while((SPI1->SR & SPI_FLAG_TXE) == RESET);
  SPI1->DR = addr;
  while((SPI1->SR & SPI_FLAG_RXNE) == RESET);
  (void)SPI1->DR;
  
  while(length--)
  {
    while((SPI1->SR & SPI_FLAG_TXE) == RESET);
    SPI1->DR = 0xFF;
    while((SPI1->SR & SPI_FLAG_RXNE) == RESET);
    *(p_data++) = SPI1->DR;
  }
  
  GPIO_WriteHigh(SPI_NSS_PORT, SPI_NSS_PIN);
#elif defined(STM8S003)
  GPIO_WriteLow(SPI_NSS_PORT, SPI_NSS_PIN);

  while((SPI->SR & SPI_FLAG_TXE) == RESET);
  SPI->DR = addr;
  while((SPI->SR & SPI_FLAG_RXNE) == RESET);
  (void)SPI->DR;
  
  while(length--)
  {
    while((SPI->SR & SPI_FLAG_TXE) == RESET);
    SPI->DR = 0xFF;
    while((SPI->SR & SPI_FLAG_RXNE) == RESET);
    *(p_data++) = SPI->DR;
  }
  
  GPIO_WriteHigh(SPI_NSS_PORT, SPI_NSS_PIN);  
#endif
}

void SPI_write(uint8_t addr, uint8_t *p_data, uint8_t length)
{
  if(length <= 0)
  {
    return;
  }

#if defined(STM8L15X_MD)  
  GPIO_WriteLow(SPI_NSS_PORT, SPI_NSS_PIN);
  
  while((SPI1->SR & SPI_FLAG_TXE) == RESET);
  SPI1->DR = (addr | 0x80);
  while((SPI1->SR & SPI_FLAG_RXNE) == RESET);
  (void)SPI1->DR;
  
  while(length--)
  {
    while((SPI1->SR & SPI_FLAG_TXE) == RESET);
    SPI1->DR = *p_data++;
    while((SPI1->SR & SPI_FLAG_RXNE) == RESET);
    (void)SPI1->DR;
  }
  
  GPIO_WriteHigh(SPI_NSS_PORT, SPI_NSS_PIN);
#elif defined(STM8S003)
  GPIO_WriteLow(SPI_NSS_PORT, SPI_NSS_PIN);
  
  while((SPI->SR & SPI_FLAG_TXE) == RESET);
  SPI->DR = (addr | 0x80);
  while((SPI->SR & SPI_FLAG_RXNE) == RESET);
  (void)SPI->DR;
  
  while(length--)
  {
    while((SPI->SR & SPI_FLAG_TXE) == RESET);
    SPI->DR = *p_data++;
    while((SPI->SR & SPI_FLAG_RXNE) == RESET);
    (void)SPI->DR;
  }
  
  GPIO_WriteHigh(SPI_NSS_PORT, SPI_NSS_PIN);
#endif
}
#endif

void delay_ms(uint16_t ms)
{
  /* With fCPU = 16MHz, delay_ms(1) can approximated achieve 1ms delay */
  uint16_t i;
  
  while(ms--)
  {
    for(i=0;i<2666;i++)
    {
    }
  }
}

static void SX1276InitIo( void )
{  
#if defined(STM8S003)  
  /* Configure PC1 (nREST) as output push-pull low (LoRa module reset ON) */
  GPIO_Init(nREST_PORT, nREST_PIN, GPIO_MODE_OUT_PP_LOW_FAST);
  
  /* Configure PC3 (CTRL1) as output push-pull low (SW control) */
  GPIO_Init(CTRL1_PORT, CTRL1_PIN, GPIO_MODE_OUT_PP_LOW_FAST);
  /* Configure PC2 (CTRL2) as output push-pull low (SW control) */
  GPIO_Init(CTRL2_PORT, CTRL2_PIN, GPIO_MODE_OUT_PP_LOW_FAST);
  
  /* Configure PB0 (Dio0) as input pull-up (LoRa software configurated) */
#if defined(MALIYU_DEBUG)
  GPIO_Init(DIO0_PORT, DIO0_PIN, GPIO_MODE_IN_PU_IT);
  EXTI->CR1 |= (1<<2);
  EXTI->CR2 |= (1<<2);
#else
  GPIO_Init(DIO0_PORT, DIO0_PIN, GPIO_MODE_IN_PU_NO_IT);
#endif
  /* Configure PB1 (Dio1) as input pull-up (LoRa software configurated) */
  GPIO_Init(DIO1_PORT, DIO1_PIN, GPIO_MODE_IN_PU_NO_IT);
  /* Configure PB2 (Dio2) as input pull-up (LoRa software configurated) */
  GPIO_Init(DIO2_PORT, DIO2_PIN, GPIO_MODE_IN_PU_NO_IT);
  /* Configure PB3 (Dio3) as input pull-up (LoRa software configurated) */
  GPIO_Init(DIO3_PORT, DIO3_PIN, GPIO_MODE_IN_PU_NO_IT);
  /* Configure PB5 (Dio5) as input pull-up (LoRa software configurated) */
  GPIO_Init(DIO5_PORT, DIO5_PIN, GPIO_MODE_IN_PU_NO_IT);

  /* Configure PE5 (SPI_NSS) as output push-pull low */
  GPIO_Init(SPI_NSS_PORT, SPI_NSS_PIN, GPIO_MODE_OUT_PP_HIGH_FAST);
  //GPIO_WriteHigh(SPI_NSS_PORT, SPI_NSS_PIN);
  //delay_ms(1);
  
  /* Initialize SPI */
#ifdef STM8_SOFT_SPI
  /* Configure PC5 (SPI_SCK) as output push-pull low */
  GPIO_Init(SPI_SCK_PORT, SPI_SCK_PIN, GPIO_MODE_OUT_PP_LOW_FAST);
  /* Configure PC6 (SPI_MOSI) as output push-pull low */
  GPIO_Init(SPI_MOSI_PORT, SPI_MOSI_PIN, GPIO_MODE_OUT_PP_LOW_FAST);
  /* Configure PC7 (SPI_MISO) as input pull-up */
  GPIO_Init(SPI_MISO_PORT, SPI_MISO_PIN, GPIO_MODE_IN_PU_NO_IT);
#else
  SPI_Cmd(DISABLE);
  CLK_PeripheralClockConfig(CLK_PERIPHERAL_SPI, DISABLE);
  GPIO_Init(SPI_SCK_PORT, SPI_SCK_PIN, GPIO_MODE_IN_FL_NO_IT);
  GPIO_Init(SPI_MOSI_PORT, SPI_MOSI_PIN, GPIO_MODE_IN_FL_NO_IT);
  GPIO_Init(SPI_MISO_PORT, SPI_MISO_PIN, GPIO_MODE_IN_FL_NO_IT);
  CLK_PeripheralClockConfig(CLK_PERIPHERAL_SPI, ENABLE);
  GPIO_ExternalPullUpConfig(SPI_SCK_PORT, SPI_SCK_PIN, ENABLE);
  GPIO_ExternalPullUpConfig(SPI_MOSI_PORT, SPI_MOSI_PIN, ENABLE);
  GPIO_ExternalPullUpConfig(SPI_MISO_PORT, SPI_MISO_PIN, ENABLE);
  GPIO_WriteHigh(SPI_NSS_PORT, SPI_NSS_PIN);
  SPI_Init(SPI_FIRSTBIT_MSB, SPI_BAUDRATEPRESCALER_2, SPI_MODE_MASTER, SPI_CLOCKPOLARITY_LOW, SPI_CLOCKPHASE_1EDGE, SPI_DATADIRECTION_2LINES_FULLDUPLEX, SPI_NSS_SOFT, 0x07);
  SPI_Cmd(ENABLE);
#endif
#elif defined(STM8L15X_MD)
  /* Configure nREST as output push-pull low (LoRa module reset ON) */
  GPIO_Init(nREST_PORT, nREST_PIN, GPIO_Mode_Out_PP_Low_Fast);
  
  /* Configure CTRL1 as output push-pull low (SW control) */
  GPIO_Init(CTRL1_PORT, CTRL1_PIN, GPIO_Mode_Out_PP_Low_Fast);
  /* Configure CTRL2 as output push-pull low (SW control) */
  GPIO_Init(CTRL2_PORT, CTRL2_PIN, GPIO_Mode_Out_PP_Low_Fast);
  
  /* Configure Dio0 as input pull-up (LoRa software configurated) */
  GPIO_Init(DIO0_PORT, DIO0_PIN, GPIO_Mode_In_PU_No_IT);
  /* Configure Dio1 as input pull-up (LoRa software configurated) */
  GPIO_Init(DIO1_PORT, DIO1_PIN, GPIO_Mode_In_PU_No_IT);
  /* Configure Dio2 as input pull-up (LoRa software configurated) */
  GPIO_Init(DIO2_PORT, DIO2_PIN, GPIO_Mode_In_PU_No_IT);
  /* Configure Dio3 as input pull-up (LoRa software configurated) */
  GPIO_Init(DIO3_PORT, DIO3_PIN, GPIO_Mode_In_PU_No_IT);
  /* Configure Dio5 as input pull-up (LoRa software configurated) */
  GPIO_Init(DIO5_PORT, DIO5_PIN, GPIO_Mode_In_PU_No_IT);

  /* Configure SPI_NSS as output push-pull low */
  GPIO_Init(SPI_NSS_PORT, SPI_NSS_PIN, GPIO_Mode_Out_PP_High_Fast);
  //GPIO_WriteHigh(SPI_NSS_PORT, SPI_NSS_PIN);
  //delay_ms(1);
  
  /* Initialize SPI */
#ifdef STM8_SOFT_SPI
  /* Configure SPI_SCK as output push-pull low */
  GPIO_Init(SPI_SCK_PORT, SPI_SCK_PIN, GPIO_Mode_Out_PP_Low_Fast);
  /* Configure SPI_MOSI as output push-pull low */
  GPIO_Init(SPI_MOSI_PORT, SPI_MOSI_PIN, GPIO_Mode_Out_PP_Low_Fast);
  /* Configure SPI_MISO as input pull-up */
  GPIO_Init(SPI_MISO_PORT, SPI_MISO_PIN, GPIO_Mode_In_PU_No_IT);
#else
  //SPI_DeInit(SPI1);
  //SYSCFG_REMAPDeInit();
  //SYSCFG_REMAPPinConfig(REMAP_Pin_SPI1Full, DISABLE);
  SPI_Cmd(SPI1, DISABLE);
  CLK_PeripheralClockConfig(CLK_Peripheral_SPI1, DISABLE);
  GPIO_Init(SPI_SCK_PORT, SPI_SCK_PIN, GPIO_Mode_In_FL_No_IT);
  GPIO_Init(SPI_MOSI_PORT, SPI_MOSI_PIN, GPIO_Mode_In_FL_No_IT);
  GPIO_Init(SPI_MISO_PORT, SPI_MISO_PIN, GPIO_Mode_In_FL_No_IT);
  CLK_PeripheralClockConfig(CLK_Peripheral_SPI1, ENABLE);
  GPIO_ExternalPullUpConfig(SPI_SCK_PORT, SPI_SCK_PIN|SPI_MOSI_PIN|SPI_MISO_PIN, ENABLE);
  GPIO_WriteHigh(SPI_NSS_PORT, SPI_NSS_PIN);
  SPI_Init(SPI1, SPI_FirstBit_MSB, SPI_BaudRatePrescaler_2, SPI_Mode_Master, SPI_CPOL_Low, SPI_CPHA_1Edge, SPI_Direction_2Lines_FullDuplex, SPI_NSS_Soft, 0x07);
  SPI_Cmd(SPI1, ENABLE);
#endif  
#endif
}

static void SX1276SetLoRaOn( void )
{
  SX1276LoRaSetLoRaOn();
}

static void SX1276Init( void )
{
  SX1276LR = ( tSX1276LR* )SX1276Regs;
  
  SX1276InitIo();
  
  /* Reset SX1276/78 */
  SX1276Reset();
  
  SX1276SetLoRaOn();
  
  SX1276LoRaInit();
}

static void SX1276Reset( void )
{
  GPIO_WriteHigh(nREST_PORT, nREST_PIN);
  /* wait 1ms */
  delay_ms(6);
  //GPIO_WriteLow(nREST_PORT, nREST_PIN);
  /* wait 6ms */
  //delay_ms(6);
}

static void SX1276StartRx( void )
{
  SX1276LoRaSetRFState( RFLR_STATE_RX_INIT );
}

static void SX1276GetRxPacket( void *buffer, uint16_t *size )
{
  SX1276LoRaGetRxPacket( buffer, size );
}

static void SX1276SetTxPacket( const void *buffer, uint16_t size )
{
  SX1276LoRaSetTxPacket( buffer, size );
}

static uint32_t SX1276Process( void )
{
  return SX1276LoRaProcess();
}

tRadioDriver* RadioDriverInit( void )
{
  RadioDriver.Init = SX1276Init;
  RadioDriver.Reset = SX1276Reset;
  RadioDriver.StartRx = SX1276StartRx;
  RadioDriver.GetRxPacket = SX1276GetRxPacket;
  RadioDriver.SetTxPacket = SX1276SetTxPacket;
  RadioDriver.Process = SX1276Process;
  
  return &RadioDriver;
}

static void SX1276LoRaSetRFState( uint8_t state )
{
    RFLRState = state;
}

static void SX1276LoRaGetRxPacket( void *buffer, uint16_t *size )
{
    *size = RxPacketSize;
    RxPacketSize = 0;
    memcpy( ( void * )buffer, ( void * )RFBuffer, ( size_t )*size );
}

static void SX1276LoRaSetTxPacket( const void *buffer, uint16_t size )
{
  if( LoRaSettings.FreqHopOn == FALSE )
  {
    
    TxPacketSize = size;
  }
  else
  {
    TxPacketSize = 255;
  }
  memcpy( ( void * )RFBuffer, buffer, ( size_t )TxPacketSize ); 

  RFLRState = RFLR_STATE_TX_INIT;
}

void SX1276LoRaSetOpMode( uint8_t opMode )
{
    static uint8_t opModePrev = RFLR_OPMODE_STANDBY;
    static bool antennaSwitchTxOnPrev = TRUE;
    bool antennaSwitchTxOn = FALSE;
    
    opModePrev = SX1276LR->RegOpMode & ~RFLR_OPMODE_MASK;

    if( opMode != opModePrev )
    {
      if( opMode == RFLR_OPMODE_TRANSMITTER )
      {
        antennaSwitchTxOn = TRUE;
      }
      else
      {
        antennaSwitchTxOn = FALSE;
      }
      if( antennaSwitchTxOn != antennaSwitchTxOnPrev )
      {
        antennaSwitchTxOnPrev = antennaSwitchTxOn;
        RXTX( antennaSwitchTxOn ); // Antenna switch control
      }
      SX1276LR->RegOpMode = ( SX1276LR->RegOpMode & RFLR_OPMODE_MASK ) | opMode;

      SPI_write( REG_LR_OPMODE, &(SX1276LR->RegOpMode), 1 );        
    }
}

static void SX1276LoRaSetLoRaOn( void )        
{
  SX1276LoRaSetOpMode( RFLR_OPMODE_SLEEP );
        
  SX1276LR->RegOpMode = ( SX1276LR->RegOpMode & RFLR_OPMODE_LONGRANGEMODE_MASK ) | RFLR_OPMODE_LONGRANGEMODE_ON;
  SPI_write( REG_LR_OPMODE, &(SX1276LR->RegOpMode), 1 );
        
  SX1276LoRaSetOpMode( RFLR_OPMODE_STANDBY );
                                        // RxDone               RxTimeout                   FhssChangeChannel           CadDone
  SX1276LR->RegDioMapping1 = RFLR_DIOMAPPING1_DIO0_00 | RFLR_DIOMAPPING1_DIO1_00 | RFLR_DIOMAPPING1_DIO2_00 | RFLR_DIOMAPPING1_DIO3_00;
                                        // CadDetected          ModeReady
  SX1276LR->RegDioMapping2 = RFLR_DIOMAPPING2_DIO4_00 | RFLR_DIOMAPPING2_DIO5_00;
  SPI_write( REG_LR_DIOMAPPING1, &(SX1276LR->RegDioMapping1), 2 );
}

/*!
 * \brief Process the LoRa modem Rx and Tx state machines depending on the
 *        SX1276 operating mode.
 *
 * \retval rfState Current RF state [RF_IDLE, RF_BUSY, 
 *                                   RF_RX_DONE, RF_RX_TIMEOUT,
 *                                   RF_TX_DONE, RF_TX_TIMEOUT]
 */
static uint32_t SX1276LoRaProcess( void )
{
    uint32_t result = RF_BUSY;
    uint8_t tmp = 0;
    
    switch( RFLRState )
    {
      case RFLR_STATE_IDLE:
          break;
      case RFLR_STATE_RX_INIT:
          
          SX1276LoRaSetOpMode( RFLR_OPMODE_STANDBY );

          SX1276LR->RegIrqFlagsMask = RFLR_IRQFLAGS_RXTIMEOUT |
                                      //RFLR_IRQFLAGS_RXDONE |
                                      //RFLR_IRQFLAGS_PAYLOADCRCERROR |
                                      RFLR_IRQFLAGS_VALIDHEADER |
                                      RFLR_IRQFLAGS_TXDONE |
                                      RFLR_IRQFLAGS_CADDONE |
                                      //RFLR_IRQFLAGS_FHSSCHANGEDCHANNEL |
                                      RFLR_IRQFLAGS_CADDETECTED;
          SPI_write( REG_LR_IRQFLAGSMASK, &(SX1276LR->RegIrqFlagsMask), 1 );

          if( LoRaSettings.FreqHopOn == TRUE )
          {
              SX1276LR->RegHopPeriod = LoRaSettings.HopPeriod;

              SPI_read(REG_LR_HOPCHANNEL, &(SX1276LR->RegHopChannel), 1);
              SX1276LoRaSetRFFrequency( HoppingFrequencies[SX1276LR->RegHopChannel & RFLR_HOPCHANNEL_CHANNEL_MASK] );
          }
          else
          {
              SX1276LR->RegHopPeriod = 255;
          }
          
          SPI_write( REG_LR_HOPPERIOD, &(SX1276LR->RegHopPeriod), 1 );
                  
                                      // RxDone                    RxTimeout                   FhssChangeChannel           CadDone
          SX1276LR->RegDioMapping1 = RFLR_DIOMAPPING1_DIO0_00 | RFLR_DIOMAPPING1_DIO1_00 | RFLR_DIOMAPPING1_DIO2_00 | RFLR_DIOMAPPING1_DIO3_00;
                                      // CadDetected               ModeReady
          SX1276LR->RegDioMapping2 = RFLR_DIOMAPPING2_DIO4_00 | RFLR_DIOMAPPING2_DIO5_00;
          SPI_write( REG_LR_DIOMAPPING1, &(SX1276LR->RegDioMapping1), 2 );
      
          if( LoRaSettings.RxSingleOn == TRUE ) // Rx single mode
          {

              SX1276LoRaSetOpMode( RFLR_OPMODE_RECEIVER_SINGLE );
          }
          else // Rx continuous mode
          {
              SX1276LR->RegFifoAddrPtr = SX1276LR->RegFifoRxBaseAddr;
              SPI_write( REG_LR_FIFOADDRPTR, &(SX1276LR->RegFifoAddrPtr), 1 );
              
              SX1276LoRaSetOpMode( RFLR_OPMODE_RECEIVER );
          }
          
          memset( RFBuffer, 0, ( size_t )RF_BUFFER_SIZE );

          PacketTimeout = LoRaSettings.RxPacketTimeout;
          RxTimeoutTimer = GET_TICK_COUNT( );
          RFLRState = RFLR_STATE_RX_RUNNING;
          break;
      case RFLR_STATE_RX_RUNNING:
          
          if( DIO0 == 1 ) // RxDone
          {
              RxTimeoutTimer = GET_TICK_COUNT( );
              if( LoRaSettings.FreqHopOn == TRUE )
              {
                  SPI_read( REG_LR_HOPCHANNEL, &(SX1276LR->RegHopChannel), 1 );
                  SX1276LoRaSetRFFrequency( HoppingFrequencies[SX1276LR->RegHopChannel & RFLR_HOPCHANNEL_CHANNEL_MASK] );
              }
              // Clear Irq
              tmp = RFLR_IRQFLAGS_RXDONE;
              SPI_write( REG_LR_IRQFLAGS, &tmp, 1  );
              RFLRState = RFLR_STATE_RX_DONE;
              //GPIO_WriteLow(GPIOD, GPIO_PIN_0);
          }
          if( DIO2 == 1 ) // FHSS Changed Channel
          {
              RxTimeoutTimer = GET_TICK_COUNT( );
              if( LoRaSettings.FreqHopOn == TRUE )
              {
                  SPI_read( REG_LR_HOPCHANNEL, &(SX1276LR->RegHopChannel), 1 );
                  SX1276LoRaSetRFFrequency( HoppingFrequencies[SX1276LR->RegHopChannel & RFLR_HOPCHANNEL_CHANNEL_MASK] );
              }
              // Clear Irq
              tmp = RFLR_IRQFLAGS_FHSSCHANGEDCHANNEL;
              SPI_write( REG_LR_IRQFLAGS, &tmp, 1 );
              // Debug
              RxGain = SX1276LoRaReadRxGain( );
          }

          /*if( LoRaSettings.RxSingleOn == TRUE ) // Rx single mode
          {
              if( ( GET_TICK_COUNT( ) - RxTimeoutTimer ) > PacketTimeout )
              {
                  RFLRState = RFLR_STATE_RX_TIMEOUT;
              }
          }*/
          break;
      case RFLR_STATE_RX_DONE:
          SPI_read( REG_LR_IRQFLAGS, &(SX1276LR->RegIrqFlags), 1 );
          if( ( SX1276LR->RegIrqFlags & RFLR_IRQFLAGS_PAYLOADCRCERROR ) == RFLR_IRQFLAGS_PAYLOADCRCERROR )
          {
              // Clear Irq
              tmp = RFLR_IRQFLAGS_PAYLOADCRCERROR;
              SPI_write( REG_LR_IRQFLAGS, &tmp, 1  );
              
              if( LoRaSettings.RxSingleOn == TRUE ) // Rx single mode
              {
                  RFLRState = RFLR_STATE_RX_INIT;
              }
              else
              {
                  RFLRState = RFLR_STATE_RX_RUNNING;
              }
              break;
          }
          
          {
              uint8_t rxSnrEstimate;
              SPI_read( REG_LR_PKTSNRVALUE, &rxSnrEstimate, 1 );
              if( rxSnrEstimate & 0x80 ) // The SNR sign bit is 1
              {
                  // Invert and divide by 4
                  RxPacketSnrEstimate = ( ( ~rxSnrEstimate + 1 ) & 0xFF ) >> 2;
                  RxPacketSnrEstimate = -RxPacketSnrEstimate;
              }
              else
              {
                  // Divide by 4
                  RxPacketSnrEstimate = ( rxSnrEstimate & 0xFF ) >> 2;
              }
          }
          
          if( LoRaSettings.RFFrequency < 860000000 )  // LF
          {    
              if( RxPacketSnrEstimate < 0 )
              {
                  RxPacketRssiValue = NOISE_ABSOLUTE_ZERO + 10.0 * SignalBwLog[LoRaSettings.SignalBw] + NOISE_FIGURE_LF + ( double )RxPacketSnrEstimate;
              }
              else
              {    
                  SPI_read( REG_LR_PKTRSSIVALUE, &(SX1276LR->RegPktRssiValue), 1 );
                  RxPacketRssiValue = RssiOffsetLF[LoRaSettings.SignalBw] + ( double )SX1276LR->RegPktRssiValue;
              }
          }
          else                                        // HF
          {    
              if( RxPacketSnrEstimate < 0 )
              {
                  RxPacketRssiValue = NOISE_ABSOLUTE_ZERO + 10.0 * SignalBwLog[LoRaSettings.SignalBw] + NOISE_FIGURE_HF + ( double )RxPacketSnrEstimate;
              }
              else
              {    
                  SPI_read( REG_LR_PKTRSSIVALUE, &(SX1276LR->RegPktRssiValue), 1 );
                  RxPacketRssiValue = RssiOffsetHF[LoRaSettings.SignalBw] + ( double )SX1276LR->RegPktRssiValue;
              }
          }

          if( LoRaSettings.RxSingleOn == TRUE ) // Rx single mode
          {
              SX1276LR->RegFifoAddrPtr = SX1276LR->RegFifoRxBaseAddr;
              SPI_write( REG_LR_FIFOADDRPTR, &(SX1276LR->RegFifoAddrPtr), 1 );

              if( LoRaSettings.ImplicitHeaderOn == TRUE )
              {
                  RxPacketSize = SX1276LR->RegPayloadLength;
                  SPI_read(REG_LR_FIFO, RFBuffer, SX1276LR->RegPayloadLength);
              }
              else
              {
                  SPI_read( REG_LR_NBRXBYTES, &(SX1276LR->RegNbRxBytes), 1 );
                  RxPacketSize = SX1276LR->RegNbRxBytes;
                  SPI_read(REG_LR_FIFO, RFBuffer, SX1276LR->RegNbRxBytes );
              }
          }
          else // Rx continuous mode
          {
              SPI_read( REG_LR_FIFORXCURRENTADDR, &(SX1276LR->RegFifoRxCurrentAddr), 1 );

              if( LoRaSettings.ImplicitHeaderOn == TRUE )
              {
                  RxPacketSize = SX1276LR->RegPayloadLength;
                  SX1276LR->RegFifoAddrPtr = SX1276LR->RegFifoRxCurrentAddr;
                  SPI_write( REG_LR_FIFOADDRPTR, &(SX1276LR->RegFifoAddrPtr), 1 );
                  SPI_read(REG_LR_FIFO, RFBuffer, SX1276LR->RegPayloadLength );
              }
              else
              {
                  SPI_read( REG_LR_NBRXBYTES, &(SX1276LR->RegNbRxBytes), 1 );
                  RxPacketSize = SX1276LR->RegNbRxBytes;
                  SX1276LR->RegFifoAddrPtr = SX1276LR->RegFifoRxCurrentAddr;
                  SPI_write( REG_LR_FIFOADDRPTR, &(SX1276LR->RegFifoAddrPtr), 1 );
                  SPI_read(REG_LR_FIFO, RFBuffer, SX1276LR->RegNbRxBytes );
              }
          }
          
          if( LoRaSettings.RxSingleOn == TRUE ) // Rx single mode
          {
              RFLRState = RFLR_STATE_RX_INIT;
          }
          else // Rx continuous mode
          {
              RFLRState = RFLR_STATE_RX_RUNNING;
          }
          result = RF_RX_DONE;
          break;
      case RFLR_STATE_RX_TIMEOUT:
          RFLRState = RFLR_STATE_RX_INIT;
          result = RF_RX_TIMEOUT;
          break;
      case RFLR_STATE_TX_INIT:

          SX1276LoRaSetOpMode( RFLR_OPMODE_STANDBY );

          if( LoRaSettings.FreqHopOn == TRUE )
          {
              SX1276LR->RegIrqFlagsMask = RFLR_IRQFLAGS_RXTIMEOUT |
                                          RFLR_IRQFLAGS_RXDONE |
                                          RFLR_IRQFLAGS_PAYLOADCRCERROR |
                                          RFLR_IRQFLAGS_VALIDHEADER |
                                          //RFLR_IRQFLAGS_TXDONE |
                                          RFLR_IRQFLAGS_CADDONE |
                                          //RFLR_IRQFLAGS_FHSSCHANGEDCHANNEL |
                                          RFLR_IRQFLAGS_CADDETECTED;
              SX1276LR->RegHopPeriod = LoRaSettings.HopPeriod;

              SPI_read( REG_LR_HOPCHANNEL, &(SX1276LR->RegHopChannel), 1 );
              SX1276LoRaSetRFFrequency( HoppingFrequencies[SX1276LR->RegHopChannel & RFLR_HOPCHANNEL_CHANNEL_MASK] );
          }
          else
          {
              SX1276LR->RegIrqFlagsMask = RFLR_IRQFLAGS_RXTIMEOUT |
                                          RFLR_IRQFLAGS_RXDONE |
                                          RFLR_IRQFLAGS_PAYLOADCRCERROR |
                                          RFLR_IRQFLAGS_VALIDHEADER |
                                          //RFLR_IRQFLAGS_TXDONE |
                                          RFLR_IRQFLAGS_CADDONE |
                                          RFLR_IRQFLAGS_FHSSCHANGEDCHANNEL |
                                          RFLR_IRQFLAGS_CADDETECTED;
              SX1276LR->RegHopPeriod = 0;
          }
          SPI_write( REG_LR_HOPPERIOD, &(SX1276LR->RegHopPeriod), 1 );
          SPI_write( REG_LR_IRQFLAGSMASK, &(SX1276LR->RegIrqFlagsMask), 1 );

          // Initializes the payload size
          SX1276LR->RegPayloadLength = TxPacketSize;
          SPI_write( REG_LR_PAYLOADLENGTH, &(SX1276LR->RegPayloadLength), 1 );
          
          SX1276LR->RegFifoTxBaseAddr = 0x00; // Full buffer used for Tx
          SPI_write( REG_LR_FIFOTXBASEADDR, &(SX1276LR->RegFifoTxBaseAddr), 1 );

          SX1276LR->RegFifoAddrPtr = SX1276LR->RegFifoTxBaseAddr;
          SPI_write( REG_LR_FIFOADDRPTR, &(SX1276LR->RegFifoAddrPtr), 1 );
          
          // Write payload buffer to LORA modem
          SPI_write( REG_LR_FIFO, RFBuffer, SX1276LR->RegPayloadLength );
                                          // TxDone               RxTimeout                   FhssChangeChannel          ValidHeader         
          SX1276LR->RegDioMapping1 = RFLR_DIOMAPPING1_DIO0_01 | RFLR_DIOMAPPING1_DIO1_00 | RFLR_DIOMAPPING1_DIO2_00 | RFLR_DIOMAPPING1_DIO3_01;
                                          // PllLock              Mode Ready
          SX1276LR->RegDioMapping2 = RFLR_DIOMAPPING2_DIO4_01 | RFLR_DIOMAPPING2_DIO5_00;
          SPI_write( REG_LR_DIOMAPPING1, &(SX1276LR->RegDioMapping1), 2 );

          SX1276LoRaSetOpMode( RFLR_OPMODE_TRANSMITTER );

          RFLRState = RFLR_STATE_TX_RUNNING;
          break;
      case RFLR_STATE_TX_RUNNING:
          if( DIO0 == 1 ) // TxDone
          {
              // Clear Irq
              tmp = RFLR_IRQFLAGS_TXDONE;
              SPI_write( REG_LR_IRQFLAGS, &tmp, 1 );
              RFLRState = RFLR_STATE_TX_DONE;   
          }
          if( DIO2 == 1 ) // FHSS Changed Channel
          {
              if( LoRaSettings.FreqHopOn == TRUE )
              {
                  SPI_read( REG_LR_HOPCHANNEL, &(SX1276LR->RegHopChannel), 1 );
                  SX1276LoRaSetRFFrequency( HoppingFrequencies[SX1276LR->RegHopChannel & RFLR_HOPCHANNEL_CHANNEL_MASK] );
              }
              // Clear Irq
              tmp = RFLR_IRQFLAGS_FHSSCHANGEDCHANNEL;
              SPI_write( REG_LR_IRQFLAGS, &tmp, 1 );
          }
          break;
      case RFLR_STATE_TX_DONE:
          // optimize the power consumption by switching off the transmitter as soon as the packet has been sent
          SX1276LoRaSetOpMode( RFLR_OPMODE_STANDBY );

          RFLRState = RFLR_STATE_IDLE;
          result = RF_TX_DONE;
          break;
      case RFLR_STATE_CAD_INIT:    
          SX1276LoRaSetOpMode( RFLR_OPMODE_STANDBY );
      
          SX1276LR->RegIrqFlagsMask = RFLR_IRQFLAGS_RXTIMEOUT |
                                      RFLR_IRQFLAGS_RXDONE |
                                      RFLR_IRQFLAGS_PAYLOADCRCERROR |
                                      RFLR_IRQFLAGS_VALIDHEADER |
                                      RFLR_IRQFLAGS_TXDONE |
                                      //RFLR_IRQFLAGS_CADDONE |
                                      RFLR_IRQFLAGS_FHSSCHANGEDCHANNEL; // |
                                      //RFLR_IRQFLAGS_CADDETECTED;
          SPI_write( REG_LR_IRQFLAGSMASK, &(SX1276LR->RegIrqFlagsMask), 1 );
             
                                      // RxDone                   RxTimeout                   FhssChangeChannel           CadDone
          SX1276LR->RegDioMapping1 = RFLR_DIOMAPPING1_DIO0_00 | RFLR_DIOMAPPING1_DIO1_00 | RFLR_DIOMAPPING1_DIO2_00 | RFLR_DIOMAPPING1_DIO3_00;
                                      // CAD Detected              ModeReady
          SX1276LR->RegDioMapping2 = RFLR_DIOMAPPING2_DIO4_00 | RFLR_DIOMAPPING2_DIO5_00;
          SPI_write( REG_LR_DIOMAPPING1, &(SX1276LR->RegDioMapping1), 2 );
              
          SX1276LoRaSetOpMode( RFLR_OPMODE_CAD );
          RFLRState = RFLR_STATE_CAD_RUNNING;
          break;
      case RFLR_STATE_CAD_RUNNING:
          if( DIO3 == 1 ) //CAD Done interrupt
          { 
              // Clear Irq
              tmp = RFLR_IRQFLAGS_CADDONE;
              SPI_write( REG_LR_IRQFLAGS, &tmp, 1 );
              if( DIO4 == 1 ) // CAD Detected interrupt
              {
                  // Clear Irq
                  tmp = RFLR_IRQFLAGS_CADDETECTED;
                  SPI_write( REG_LR_IRQFLAGS, &tmp, 1 );
                  // CAD detected, we have a LoRa preamble
                  RFLRState = RFLR_STATE_RX_INIT;
                  result = RF_CHANNEL_ACTIVITY_DETECTED;
              } 
              else
              {    
                  // The device goes in Standby Mode automatically    
                  RFLRState = RFLR_STATE_IDLE;
                  result = RF_CHANNEL_EMPTY;
              }
          }   
          break;
      
      default:
          break;
    } 
    return result;
}

static void SX1276LoRaSetRFFrequency( uint32_t freq )
{
    LoRaSettings.RFFrequency = freq;

    freq = ( uint32_t )( ( double )freq / ( double )FREQ_STEP );
    SX1276LR->RegFrfMsb = ( uint8_t )( ( freq >> 16 ) & 0xFF );
    SX1276LR->RegFrfMid = ( uint8_t )( ( freq >> 8 ) & 0xFF );
    SX1276LR->RegFrfLsb = ( uint8_t )( freq & 0xFF );
    SPI_write( REG_LR_FRFMSB, &(SX1276LR->RegFrfMsb), 3 );
}

static uint8_t SX1276ReadDio0( void )
{
    return ((uint8_t)GPIO_ReadInputPin( DIO0_PORT, DIO0_PIN ));
}

static uint8_t SX1276ReadDio1( void )
{
    return ((uint8_t)GPIO_ReadInputPin( DIO1_PORT, DIO1_PIN ));
}

static uint8_t SX1276ReadDio2( void )
{
    return ((uint8_t)GPIO_ReadInputPin( DIO2_PORT, DIO2_PIN ));
}

static uint8_t SX1276ReadDio3( void )
{
    return ((uint8_t)GPIO_ReadInputPin( DIO3_PORT, DIO3_PIN ));
}

static uint8_t SX1276ReadDio4( void )
{
    //return GPIO_ReadInputPin( DIO4_PORT, DIO4_PIN );
  return 0;
}

static uint8_t SX1276ReadDio5( void )
{
    return ((uint8_t)GPIO_ReadInputPin( DIO5_PORT, DIO5_PIN ));
}

static uint8_t SX1276LoRaReadRxGain( void )
{
    SPI_read( REG_LR_LNA, &(SX1276LR->RegLna), 1 );
    return( SX1276LR->RegLna >> 5 ) & 0x07;
}

static void SX1276LoRaInit( void )
{
    uint32_t freq;
    
    RFLRState = RFLR_STATE_IDLE;

    // Read out all register settings
    SPI_read( REG_LR_OPMODE, SX1276Regs + 1, 0x70 );
    
    // use TCXO
    SX1276LR->RegTcxo = (SX1276LR->RegTcxo & 0xEF) | 0x10;
    
    // Set max LNA gain
    SX1276LR->RegLna = (SX1276LR->RegLna & ~RFLR_LNA_GAIN_G1) | RFLR_LNA_GAIN_G1;

    // set the RF settings 
    //SX1276LoRaSetRFFrequency( LoRaSettings.RFFrequency );
    freq = ( uint32_t )( ( double )LoRaSettings.RFFrequency / ( double )FREQ_STEP );
    SX1276LR->RegFrfMsb = ( uint8_t )( ( freq >> 16 ) & 0xFF );
    SX1276LR->RegFrfMid = ( uint8_t )( ( freq >> 8 ) & 0xFF );
    SX1276LR->RegFrfLsb = ( uint8_t )( freq & 0xFF );
    //SX1276LR->RegFrfMsb = 0x85;
    //SX1276LR->RegFrfMid = 0x3B;
    //SX1276LR->RegFrfLsb = 0x13;
    
    // Set PA ountpint to PA_BOOST
    // Enable +20dBm output power
    SX1276LR->RegPaConfig = (SX1276LR->RegPaConfig & ~RFLR_PACONFIG_PASELECT_PABOOST ) | RFLR_PACONFIG_PASELECT_PABOOST;
    SX1276LR->RegPaDac = 0x87;
    if( ( SX1276LR->RegPaConfig & RFLR_PACONFIG_PASELECT_PABOOST ) == RFLR_PACONFIG_PASELECT_PABOOST )
    {
        if( ( SX1276LR->RegPaDac & 0x87 ) == 0x87 )
        {
            if( LoRaSettings.Power < 5 )
            {
                LoRaSettings.Power = 5;
            }
            if( LoRaSettings.Power > 20 )
            {
                LoRaSettings.Power = 20;
            }
            SX1276LR->RegPaConfig = ( SX1276LR->RegPaConfig & RFLR_PACONFIG_MAX_POWER_MASK ) | 0x70;
            SX1276LR->RegPaConfig = ( SX1276LR->RegPaConfig & RFLR_PACONFIG_OUTPUTPOWER_MASK ) | ( ( LoRaSettings.Power - 5 ) & 0x0F );
        }
        else
        {
            if( LoRaSettings.Power < 2 )
            {
                LoRaSettings.Power = 2;
            }
            if( LoRaSettings.Power > 17 )
            {
                LoRaSettings.Power = 17;
            }
            SX1276LR->RegPaConfig = ( SX1276LR->RegPaConfig & RFLR_PACONFIG_MAX_POWER_MASK ) | 0x70;
            SX1276LR->RegPaConfig = ( SX1276LR->RegPaConfig & RFLR_PACONFIG_OUTPUTPOWER_MASK ) | ( ( LoRaSettings.Power - 2 ) & 0x0F );
        }
    }
    else
    {
        if( LoRaSettings.Power < -1 )
        {
            LoRaSettings.Power = -1;
        }
        if( LoRaSettings.Power > 14 )
        {
            LoRaSettings.Power = 14;
        }
        SX1276LR->RegPaConfig = ( SX1276LR->RegPaConfig & RFLR_PACONFIG_MAX_POWER_MASK ) | 0x70;
        SX1276LR->RegPaConfig = ( SX1276LR->RegPaConfig & RFLR_PACONFIG_OUTPUTPOWER_MASK ) | ( ( LoRaSettings.Power + 1 ) & 0x0F );
    }

    //SX1276LoRaSetSpreadingFactor( LoRaSettings.SpreadingFactor ); // SF6 only operates in implicit header mode.
    if(LoRaSettings.SpreadingFactor > 12)
    {
      LoRaSettings.SpreadingFactor = 12;
    }
    else if(LoRaSettings.SpreadingFactor <6)
    {
      LoRaSettings.SpreadingFactor = 6;
    }
    
    if(LoRaSettings.SpreadingFactor == 6)
    {
      SX1276LR->RegTestReserved31 = ( SX1276LR->RegTestReserved31 & 0xF8 ) | 5;
    }
    else
    {
      SX1276LR->RegTestReserved31 = ( SX1276LR->RegTestReserved31 & 0xF8 ) | 3;
    }
    SX1276LR->RegModemConfig2 = ( SX1276LR->RegModemConfig2 & RFLR_MODEMCONFIG2_SF_MASK ) | ( LoRaSettings.SpreadingFactor << 4 );
    
    //SX1276LoRaSetErrorCoding( LoRaSettings.ErrorCoding );
    SX1276LR->RegModemConfig1 = ( SX1276LR->RegModemConfig1 & RFLR_MODEMCONFIG1_CODINGRATE_MASK ) | ( LoRaSettings.ErrorCoding << 1 );
    
    //SX1276LoRaSetPacketCrcOn( LoRaSettings.CrcOn );
    SX1276LR->RegModemConfig2 = ( SX1276LR->RegModemConfig2 & RFLR_MODEMCONFIG2_RXPAYLOADCRC_MASK ) | ( LoRaSettings.CrcOn << 2 );

    //SX1276LoRaSetSignalBandwidth( LoRaSettings.SignalBw );
    SX1276LR->RegModemConfig1 = ( SX1276LR->RegModemConfig1 & RFLR_MODEMCONFIG1_BW_MASK ) | ( LoRaSettings.SignalBw << 4 );

    //SX1276LoRaSetImplicitHeaderOn( LoRaSettings.ImplicitHeaderOn );
    SX1276LR->RegModemConfig1 = ( SX1276LR->RegModemConfig1 & RFLR_MODEMCONFIG1_IMPLICITHEADER_MASK ) | ( LoRaSettings.ImplicitHeaderOn );
    
    //SX1276LoRaSetSymbTimeout( 0x3FF );
    SX1276LR->RegModemConfig2 = ( SX1276LR->RegModemConfig2 & RFLR_MODEMCONFIG2_SYMBTIMEOUTMSB_MASK ) | ( ( 0x3FF >> 8 ) & ~RFLR_MODEMCONFIG2_SYMBTIMEOUTMSB_MASK );
    SX1276LR->RegSymbTimeoutLsb = 0x3FF & 0xFF;
    
    //SX1276LoRaSetPayloadLength( LoRaSettings.PayloadLength );
    SX1276LR->RegPayloadLength = LoRaSettings.PayloadLength;

    //SX1276LoRaSetLowDatarateOptimize( TRUE );
    SX1276LR->RegModemConfig3 = ( SX1276LR->RegModemConfig3 & RFLR_MODEMCONFIG3_LOWDATARATEOPTIMIZE_MASK ) | ( TRUE << 3 );
    
    SX1276LR->RegPreambleMsb = 0;
    SX1276LR->RegPreambleLsb = 16;
    
    SPI_write( REG_LR_OPMODE, SX1276Regs + 1, 0x70 - 1 );
    
    SX1276LoRaSetOpMode( RFLR_OPMODE_STANDBY );
}

/*static void SX1276LoRaSetSpreadingFactor( uint8_t factor )
{

    if( factor > 12 )
    {
        factor = 12;
    }
    else if( factor < 6 )
    {
        factor = 6;
    }

    if( factor == 6 )
    {
        SX1276LoRaSetNbTrigPeaks( 5 );
    }
    else
    {
        SX1276LoRaSetNbTrigPeaks( 3 );
    }

    SPI_read( REG_LR_MODEMCONFIG2, &(SX1276LR->RegModemConfig2), 1 );    
    SX1276LR->RegModemConfig2 = ( SX1276LR->RegModemConfig2 & RFLR_MODEMCONFIG2_SF_MASK ) | ( factor << 4 );
    SPI_write( REG_LR_MODEMCONFIG2, &(SX1276LR->RegModemConfig2), 1 );    
    LoRaSettings.SpreadingFactor = factor;
}*/

/*static void SX1276LoRaSetErrorCoding( uint8_t value )
{
    SPI_read( REG_LR_MODEMCONFIG1, &(SX1276LR->RegModemConfig1), 1 );
    SX1276LR->RegModemConfig1 = ( SX1276LR->RegModemConfig1 & RFLR_MODEMCONFIG1_CODINGRATE_MASK ) | ( value << 1 );
    SPI_write( REG_LR_MODEMCONFIG1, &(SX1276LR->RegModemConfig1), 1 );
    LoRaSettings.ErrorCoding = value;
}*/

/*static void SX1276LoRaSetPacketCrcOn( bool enable )
{
    SPI_read( REG_LR_MODEMCONFIG2, &(SX1276LR->RegModemConfig2), 1 );
    SX1276LR->RegModemConfig2 = ( SX1276LR->RegModemConfig2 & RFLR_MODEMCONFIG2_RXPAYLOADCRC_MASK ) | ( enable << 2 );
    SPI_write( REG_LR_MODEMCONFIG2, &(SX1276LR->RegModemConfig2), 1 );
    LoRaSettings.CrcOn = enable;
}*/

/*static void SX1276LoRaSetSignalBandwidth( uint8_t bw )
{
    SPI_read( REG_LR_MODEMCONFIG1, &(SX1276LR->RegModemConfig1), 1 );
    SX1276LR->RegModemConfig1 = ( SX1276LR->RegModemConfig1 & RFLR_MODEMCONFIG1_BW_MASK ) | ( bw << 4 );
    SPI_write( REG_LR_MODEMCONFIG1, &(SX1276LR->RegModemConfig1), 1 );
    LoRaSettings.SignalBw = bw;
}*/

/*static void SX1276LoRaSetImplicitHeaderOn( bool enable )
{
    SPI_read( REG_LR_MODEMCONFIG1, &(SX1276LR->RegModemConfig1), 1 );
    SX1276LR->RegModemConfig1 = ( SX1276LR->RegModemConfig1 & RFLR_MODEMCONFIG1_IMPLICITHEADER_MASK ) | ( enable );
    SPI_write( REG_LR_MODEMCONFIG1, &(SX1276LR->RegModemConfig1), 1 );
    LoRaSettings.ImplicitHeaderOn = enable;
}*/

/*static void SX1276LoRaSetSymbTimeout( uint16_t value )
{
    SPI_read( REG_LR_MODEMCONFIG2, &(SX1276LR->RegModemConfig2), 2 );

    SX1276LR->RegModemConfig2 = ( SX1276LR->RegModemConfig2 & RFLR_MODEMCONFIG2_SYMBTIMEOUTMSB_MASK ) | ( ( value >> 8 ) & ~RFLR_MODEMCONFIG2_SYMBTIMEOUTMSB_MASK );
    SX1276LR->RegSymbTimeoutLsb = value & 0xFF;
    SPI_write( REG_LR_MODEMCONFIG2, &(SX1276LR->RegModemConfig2), 2 );
}*/

/*static void SX1276LoRaSetPayloadLength( uint8_t value )
{
    SX1276LR->RegPayloadLength = value;
    SPI_write( REG_LR_PAYLOADLENGTH, &(SX1276LR->RegPayloadLength), 1 );
    LoRaSettings.PayloadLength = value;
}*/

/*static void SX1276LoRaSetLowDatarateOptimize( bool enable )
{
    SPI_read( REG_LR_MODEMCONFIG3, &(SX1276LR->RegModemConfig3), 1 );
    SX1276LR->RegModemConfig3 = ( SX1276LR->RegModemConfig3 & RFLR_MODEMCONFIG3_LOWDATARATEOPTIMIZE_MASK ) | ( enable << 3 );
    SPI_write( REG_LR_MODEMCONFIG3, &(SX1276LR->RegModemConfig3), 1 );
}*/

/*static void SX1276LoRaSetPAOutput( uint8_t outputPin )
{
    SPI_read( REG_LR_PACONFIG, &(SX1276LR->RegPaConfig), 1 );
    SX1276LR->RegPaConfig = (SX1276LR->RegPaConfig & RFLR_PACONFIG_PASELECT_MASK ) | outputPin;
    SPI_write( REG_LR_PACONFIG, &(SX1276LR->RegPaConfig), 1 );
}*/

/*static void SX1276LoRaSetPa20dBm( bool enale )
{
    SPI_read( REG_LR_PADAC, &(SX1276LR->RegPaDac), 1 );
    SPI_read( REG_LR_PACONFIG, &(SX1276LR->RegPaConfig), 1 );

    if( ( SX1276LR->RegPaConfig & RFLR_PACONFIG_PASELECT_PABOOST ) == RFLR_PACONFIG_PASELECT_PABOOST )
    {    
        if( enale == TRUE )
        {
            SX1276LR->RegPaDac = 0x87;
        }
    }
    else
    {
        SX1276LR->RegPaDac = 0x84;
    }
    SPI_write( REG_LR_PADAC, &(SX1276LR->RegPaDac), 1 );
}*/

/*static void SX1276LoRaSetRFPower( int8_t power )
{
    SPI_read( REG_LR_PACONFIG, &(SX1276LR->RegPaConfig), 1 );
    SPI_read( REG_LR_PADAC, &(SX1276LR->RegPaDac), 1 );
    
    if( ( SX1276LR->RegPaConfig & RFLR_PACONFIG_PASELECT_PABOOST ) == RFLR_PACONFIG_PASELECT_PABOOST )
    {
        if( ( SX1276LR->RegPaDac & 0x87 ) == 0x87 )
        {
            if( power < 5 )
            {
                power = 5;
            }
            if( power > 20 )
            {
                power = 20;
            }
            SX1276LR->RegPaConfig = ( SX1276LR->RegPaConfig & RFLR_PACONFIG_MAX_POWER_MASK ) | 0x70;
            SX1276LR->RegPaConfig = ( SX1276LR->RegPaConfig & RFLR_PACONFIG_OUTPUTPOWER_MASK ) | ( uint8_t )( ( uint16_t )( power - 5 ) & 0x0F );
        }
        else
        {
            if( power < 2 )
            {
                power = 2;
            }
            if( power > 17 )
            {
                power = 17;
            }
            SX1276LR->RegPaConfig = ( SX1276LR->RegPaConfig & RFLR_PACONFIG_MAX_POWER_MASK ) | 0x70;
            SX1276LR->RegPaConfig = ( SX1276LR->RegPaConfig & RFLR_PACONFIG_OUTPUTPOWER_MASK ) | ( uint8_t )( ( uint16_t )( power - 2 ) & 0x0F );
        }
    }
    else
    {
        if( power < -1 )
        {
            power = -1;
        }
        if( power > 14 )
        {
            power = 14;
        }
        SX1276LR->RegPaConfig = ( SX1276LR->RegPaConfig & RFLR_PACONFIG_MAX_POWER_MASK ) | 0x70;
        SX1276LR->RegPaConfig = ( SX1276LR->RegPaConfig & RFLR_PACONFIG_OUTPUTPOWER_MASK ) | ( uint8_t )( ( uint16_t )( power + 1 ) & 0x0F );
    }
    SPI_write( REG_LR_PACONFIG, &(SX1276LR->RegPaConfig), 1 );
    LoRaSettings.Power = power;
}*/

/*static void SX1276LoRaSetNbTrigPeaks( uint8_t value )
{
    SPI_read( 0x31, &(SX1276LR->RegTestReserved31), 1 );
    SX1276LR->RegTestReserved31 = ( SX1276LR->RegTestReserved31 & 0xF8 ) | value;
    SPI_write( 0x31, &(SX1276LR->RegTestReserved31), 1 );
}*/

static void SX1276WriteRxTx( uint8_t txEnable )
{
    if( txEnable != 0 )
    {
        GPIO_WriteLow(CTRL1_PORT, CTRL1_PIN);
        GPIO_WriteHigh(CTRL2_PORT, CTRL2_PIN);
    }
    else
    {
        GPIO_WriteHigh(CTRL1_PORT, CTRL1_PIN);
        GPIO_WriteLow(CTRL2_PORT, CTRL2_PIN);
    }
}

#if defined(MALIYU_DEBUG)
uint8_t get_RFLRState(void)
{
  return RFLRState;
}
#endif