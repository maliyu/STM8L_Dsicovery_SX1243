/*
 * THE FOLLOWING FIRMWARE IS PROVIDED: (1) "AS IS" WITH NO WARRANTY; AND 
 * (2)TO ENABLE ACCESS TO CODING INFORMATION TO GUIDE AND FACILITATE CUSTOMER.
 * CONSEQUENTLY, SEMTECH SHALL NOT BE HELD LIABLE FOR ANY DIRECT, INDIRECT OR
 * CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING FROM THE CONTENT
 * OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE CODING INFORMATION
 * CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
 * 
 * Copyright (C) SEMTECH S.A.
 */
/*! 
 * \file        sx1243.h
 * \brief       SX1243 transmitter RF chip driver
 *
 * \version     2.0 
 * \date        Feb 26 2013
 * \author      Gregory Cristian
 */

#ifndef __SX1243_H__
#define __SX1243_H__

//#include "MCU-Hal.h"

/*!
 *	Basic types definition
 */
#define U8 unsigned char
#define U16 unsigned short
#define U32 unsigned long
#define S8 signed char
#define S16 signed short
#define S32 signed long
#define F24 float
#define F32 double

/*!
 *	boolean types definition
 */
#define TRUE 1
#define true 1
#define FALSE 0
#define false 0

/*!
 * This is the crytal frequency of the sx1243. THIS SHOULD BE CHANGED DEPENDING ON YOUR IMPLEMENTATION
 */
#define XTAL_FREQ                                   26000000
/*!
 * This is the center frequency for the transmission. THIS SHOULD BE CHANGED DEPENDING ON YOUR SYSTEM
 */
#define RF_FREQUENCY                                903000000
/*!
 * This is the frequency deviation. THIS SHOULD BE CHANGED DEPENDING ON YOUR SYSTEM
 */
#define RF_FREQUENCY_DEVIATION                      20000
/*!
 * This is the bitrate used for the transmission. THIS SHOULD BE CHANGED DEPENDING ON YOUR SYSTEM
 */
#define RF_BITRATE                                  9600
/*!
 * SX1243 internal frequency step
 */
#define FREQ_STEP                                   ( double )( ( double )XTAL_FREQ / DIVIDER )

/*!
 * SX1243 internal divider set depending of the frequency band
 */
#if (RF_FREQUENCY > 850000000)
    #define DIVIDER                                 8192    // 2^13
#else
    #define DIVIDER                                 16384   // 2^14
#endif

/*!
 * FRF is caclulated based on the transmission frequency and the internal frequency step
 */
#define FRF                                         ( U32 )( ( double ) RF_FREQUENCY / FREQ_STEP )
/*!
 * FDEV is caclulated based on the frequency deviation and the internal frequency step
 */
#define FDEV                                        ( ( U16 )( ( double ) RF_FREQUENCY_DEVIATION / FREQ_STEP ) << 5 )

/*!
 * Timer and Interrupt handling functions
 */
#define TIMER1_CLK                                  16000000 // CPU clk at 16M for internal operations
#define TIMER1_PRESCALER_VALUE                      1   // 1:1 prescaler


/*!
 * \brief Compute the bitrate for STM MCU (incrementing timer)
 */
#define ComputeBitrate( bitrate ) ( U16 )( TIMER1_CLK / (  TIMER1_PRESCALER_VALUE * bitrate ) )


/*!
 * RF packet definition
 */
#define RF_BUFFER_SIZE_MAX                          256

/*!
 * Functions return codes definition
 */
#define SX_OK           ( U8 ) 0x00
#define SX_ERROR        ( U8 ) 0x01
#define SX_BUSY         ( U8 ) 0x02
#define SX_EMPTY        ( U8 ) 0x03
#define SX_DONE         ( U8 ) 0x04
#define SX_TIMEOUT      ( U8 ) 0x05
#define SX_UNSUPPORTED  ( U8 ) 0x06
#define SX_WAIT         ( U8 ) 0x07
#define SX_CLOSE        ( U8 ) 0x08
#define SX_ACK          ( U8 ) 0x09
#define SX_NACK         ( U8 ) 0x0A
#define SX_YES          ( U8 ) 0x0B
#define SX_NO           ( U8 ) 0x0C

/*!
 * SX1243 pins
 */
#define RF_RESET        SX1243RF_Reset
#define RF_DATA         SX1243RF_DataWrite               
#define RF_DATA_I       SX1243RF_DataRead()
#define RF_CTRL         SX1243RF_Ctrl
#define RF_TX_RDY       SX1243RF_RxTxReady()

/*
 * SX1243 instructions
 */
#define INSTRUCTION_CONFIG                          0
#define INSTRUCTION_FREQUENCY                       1
#define INSTRUCTION_STATUS                          2
#define INSTRUCTION_BIST                            3
#define INSTRUCTION_TEST                            4
#define INSTRUCTION_RECOVERY                        5


/*!
 * sx1243 instructions operations
 */
#define INST_WRITE_CFG                              0x000000
#define INST_WRITE_FREQ                             0x180000
#define INST_WRITE_TEST                             0x211000
#define INST_READ_CFG                               0x330000
#define INST_READ_FREQ                              0x440000
#define INST_READ_STAT                              0x550000
#define INST_READ_BIST                              0x660000
#define INST_READ_TEST                              0x77F800
#define INST_RECOVERY                               0xFFFFFF


/*!
 * SX1243 Parameters
 */
#define	MODE_AUTOMATIC                              0x0000
#define MODE_FORCED                                 0x8000

#define MODULATION_FSK                              0x0000
#define MODULATION_OOK                              0x4000

#define BAND_0                                      0x0000
#define BAND_1                                      0x2000

#define POWER_0DBM                                  0x0000
#define POWER_10DBM                                 0x0010

#define TIMEOFF_2MS                                 0x0000
#define TIMEOFF_20MS                                0x0008

#define RESERVED                                    0x0004

#define ENABLE                                      1
#define DISABLE                                     0

#define FIXED_PACKET_LENGTH                         0
#define VARIABLE_PACKET_LENGTH                      1

#define ADDRESS_NOT_IN_PAYLOAD                      0
#define ADDRESS_IN_PAYLOAD                          1

#define CRC_OFF                                     0
#define CRC_ON                                      1

#define CRC_TYPE_CCITT                              0
#define CRC_TYPE_IBM                                1


//! Polynomial = X^16 + X^12 + X^5 + 1
#define POLYNOMIAL_CCITT                            0x1021
//! Polynomial = X^16 + X^15 + X^2 + 1
#define POLYNOMIAL_IBM                              0x8005

/*!
 * CRC seeds
 */
#define CRC_IBM_SEED                                0xFFFF
#define CRC_CCITT_SEED                              0x1D0F



/*!
 * Radio packet structure
 */
typedef union
{
    struct sPacketConfig
    {
        U8 SyncSize             : 4;    // Sync word size
        U8 Format               : 1;    // 0: Fixed packet length; 1: Variable packet length
        U8 CrcOn                : 1;    // 1: CRC on; 0: CRC off
        U8 CrcType              : 1;    // 1: CRC type IBM; 0: CRC type CCITT
        U8 NodeAddressInPayload : 1;    // Add address in payload
    }bits;
    U8 byte;
}tPacketConfig;


typedef union
{
    struct
    {
        U8 ChannelNumber  : 7;      // NUmber of channel use in Frequency hopping
        U8 Enable         : 1;      // Frequency hopping enable
    }bits;
    U8 byte;
}tFreqHopping;


typedef struct sTxParam
{
    U32 Freq;                       // Center frequency
    U16 Reserved;                   // Reserved
    U16 OffTimer;                   // Time between automatic sleep mode
    U16 RfPower;                    // RF Power
    U16 FDev;                       // Frequency deviation
    U16 Band;                       // Modualtion Band
    U16 Modulation;                 // Modulation type
    U16 Mode;                       // Transmission mode
    U16 Bitrate;                    // Bitrate
    U8 Delay;                       // delay between transmission
    U32 Repeat;                     // number of transmission
    tFreqHopping freqHopping;
    U32 FreqHoppingChannelStep;
    U8 PreambleSize;                // Size of preamble (in byte)
    tPacketConfig Config;
    U8 *syncValue;                  // Sync value
    U8 NodeAddress;                 // Node address
    U8 MessageSize;                 // Payload size
    U8 BufferSize;                  // Size of the payload in Byte
    U8 *buffer;                     // Payload
    U16 CrcValue;                   // CRC value
}tTxParam;

/*!
 * \brief Initializes the SX1243
 *
 * \retval status [OK]
 */
U8 SX1243Init( void );

/*!
 * \brief Resets the SX1243
 */
void SX1243Reset( void );

/*!
 * \brief Set the Sync Value for transmission
 *
 * \param [IN]: *syncValue        pointer on the sync value
 * \param [IN]:  size             Syze of sync word in bytes
 * \retval status
 */
U8 SX1243SetTxSyncValue( U8 *syncValue, U8 size );

/*!
 * \brief Set the payload for the transmission
 *
 * \param [IN]: *buffer        pointer on the payload
 * \param [IN]:  size          size of the payload in byte
 * \retval status
 */
U8 SX1243SetTxPacketBuffer( U8 *buffer, U8 size );

/*!
 * \brief Process with the transmission for the packets
 *
 * \retval status
 */
U8 SX1243Process( void );

#endif //__SX1243_H__
