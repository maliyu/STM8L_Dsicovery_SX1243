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
 * \file        sx1243.c
 * \brief       SX1243 transmitter RF chip driver
 *
 * \version     2.0
 * \date        Feb 26 2013
 * \author      Gregory Cristian
 */

#include "stm8l15x.h"
#include "stm8l15x_gpio.h"
#include "board.h"
#include "sx1243.h"


/*!
 * Radio packet to be transmitted
 */
tTxParam TxParam;

static U16 Bitrate = 0;

/*!
 * psedo random number
 */
#define RANDL_MAX 2147483647
static unsigned long next = 1;

#define DisableInterruptsHandler			disableInterrupts
#define EnableInterruptsHandler			enableInterrupts

#define INIT_BITRATE_HANDLING_TIMER		Init_TIM1
#define RESET_BITRATE_HANDLING_TIMER()	Reset_TIM1(Bitrate)
#define BITRATE_HANDLING_TIMER_WAIT		Wait_TIM1
#define ENABLE_BITRATE_TIMER				Enable_TIM1
#define DISABLE_BITRATE_TIMER				Disable_TIM1

/*!
 * In the current application example, we are using a timer to generate a tick every 1 ms
 */
void StandardWait(U16 delay)
{
	delay_ms(delay);
}

/*!
 *  Generate a pseudo-random 16 bit value
 */
static int randl( void )
{
    return ( ( next = next * 1103515245 + 12345 ) % RANDL_MAX );
}

/*!
 *  Seed the pseudo random function
 */
static void srandl( unsigned int seed )
{
    next = seed;
}

/*!
 * Return a peudo-random 16 bit value between min and max
 */
static U32 randr( U32 min, U32 max )
{
    return ( U32 )randl( ) % ( max - min + 1 ) + min;
}

static void SX1243RF_Reset(U8 a)
{
  GPIO_Init(GPIOC, GPIO_Pin_6, GPIO_Mode_Out_PP_High_Fast);
  GPIO_WriteBit(GPIOC, GPIO_Pin_6, ( a & 0x01 ) ? SET: RESET);
}

static void SX1243RF_DataWrite(U8 a)
{
  GPIO_Init(GPIOD, GPIO_Pin_0, GPIO_Mode_Out_PP_High_Fast);
  GPIO_WriteBit(GPIOD, GPIO_Pin_0, ( a & 0x01 ) ? SET: RESET);
}

static U8 SX1243RF_DataRead(void)
{
  GPIO_Init(GPIOD, GPIO_Pin_0, GPIO_Mode_In_PU_No_IT);
  return ((U8)GPIO_ReadInputDataBit(GPIOD, GPIO_Pin_0));
}

static void SX1243RF_Ctrl(U8 a)
{
  GPIO_Init(GPIOA, GPIO_Pin_3, GPIO_Mode_Out_PP_High_Fast);
  GPIO_WriteBit(GPIOA, GPIO_Pin_3, ( a & 0x01 ) ? SET: RESET);
}

static U8 SX1243RF_RxTxReady(void)
{
  GPIO_Init(GPIOB, GPIO_Pin_0, GPIO_Mode_In_PU_No_IT);
  return ((U8)GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_0));
}

/*
 * Send data through the SPI
 *
 * To ease the porting on MCU not necessary equipped with standard SPI interface,
 * the SPI is organized around toggling GPIOs
 *
 * \param[IN] outData Data to send on SPI
 *
 * \retval None
 */
void SpiOut( U8 outData  )
{  
    RF_CTRL(0);
    RF_DATA(( outData & 0x80 ) ? 1 : 0);
    RF_CTRL(1);
    RF_CTRL(0);
    RF_DATA(( outData & 0x40 ) ? 1 : 0);
    RF_CTRL(1);
    RF_CTRL(0);
    RF_DATA(( outData & 0x20 ) ? 1 : 0);
    RF_CTRL(1);
    RF_CTRL(0);
    RF_DATA(( outData & 0x10 ) ? 1 : 0);
    RF_CTRL(1);
    RF_CTRL(0);
    RF_DATA(( outData & 0x08 ) ? 1 : 0);
    RF_CTRL(1);
    RF_CTRL(0);
    RF_DATA(( outData & 0x04 ) ? 1 : 0);
    RF_CTRL(1);
    RF_CTRL(0);
    RF_DATA(( outData & 0x02 ) ? 1 : 0);
    RF_CTRL(1);
    RF_CTRL(0);
    RF_DATA(( outData & 0x01 ) ? 1 : 0);
    RF_CTRL(1);
    RF_CTRL(0);
}

/*
 * Read data from the SPI
 *
 * To ease the porting on MCU not necessary equipped with standard SPI interface,
 * the SPI is organized around toggling GPIOs
 *
 * \param[IN] None
 *
 * \retval Value from the SPI
 */
U8 SpiIn( void )
{
    U8 inData = 0;

    RF_CTRL(0);
    nop();
    RF_CTRL(1);
    inData |= ( RF_DATA_I ) ? 0x80 : 0;
    RF_CTRL(0);
    nop();
    RF_CTRL(1);
    inData |= ( RF_DATA_I ) ? 0x40 : 0;
    RF_CTRL(0);
    nop();
    RF_CTRL(1);
    inData |= ( RF_DATA_I ) ? 0x20 : 0;
    RF_CTRL(0);
    nop();
    RF_CTRL(1);
    inData |= ( RF_DATA_I ) ? 0x10 : 0;
    RF_CTRL(0);
    nop();
    RF_CTRL(1);
    inData |= ( RF_DATA_I ) ? 0x08 : 0;
    RF_CTRL(0);
    nop();
    RF_CTRL(1);
    inData |= ( RF_DATA_I ) ? 0x04 : 0;
    RF_CTRL(0);
    nop();
    RF_CTRL(1);
    inData |= ( RF_DATA_I) ? 0x02 : 0;
    RF_CTRL(0);
    nop();
    RF_CTRL(1);
    inData |= ( RF_DATA_I) ? 0x01 : 0;
    RF_CTRL(0);
    nop();

    return inData;
}

/*
 * Recover the communication with the sx1243
 *
 * If, for any reasons, the communication with the sx1243 was to become
 * unstable, it is possible to recover the communication by using the
 * recovery command
 *
 * \param[IN] None
 *
 * \retval None
 */
void SpiOutRecovery( void )
{
    RF_DATA(0);
    RF_CTRL(0);
    RF_DATA(1);
    RF_CTRL(1);
    RF_DATA(0);
    RF_CTRL(0);
    RF_DATA(1);
    RF_CTRL(1);
    RF_DATA(0);
    RF_CTRL(0);
    RF_DATA(1);
    RF_CTRL(1);
    RF_DATA(0);
    RF_CTRL(0);
    RF_DATA(1);
    RF_CTRL(1);
    RF_DATA(0);
    RF_CTRL(0);
    RF_DATA(1);
    RF_CTRL(1);
    RF_DATA(0);
    RF_CTRL(0);
    RF_DATA(1);
    RF_CTRL(1);
    RF_DATA(0);
    RF_CTRL(0);
    RF_DATA(1);
    RF_CTRL(1);
    RF_DATA(0);
    RF_CTRL(0);
    RF_DATA(1);
    RF_CTRL(1);
    RF_DATA(0);
    RF_CTRL(0);
    RF_DATA(1);
    RF_CTRL(1);
    RF_DATA(0);
    RF_CTRL(0);
    RF_DATA(1);
    RF_CTRL(1);
    RF_DATA(0);
    RF_CTRL(0);
    RF_DATA(1);
    RF_CTRL(1);
    RF_DATA(0);
    RF_CTRL(0);
    RF_DATA(1);
    RF_CTRL(1);
    RF_DATA(0);
    RF_CTRL(0);
    RF_DATA(1);
    RF_CTRL(1);
    RF_DATA(0);
    RF_CTRL(0);
    RF_DATA(1);
    RF_CTRL(1);
    RF_DATA(0);
    RF_CTRL(0);
    RF_DATA(1);
    RF_CTRL(1);
    RF_DATA(0);
    RF_CTRL(0);
    RF_DATA(1);
    RF_CTRL(1);
    RF_DATA(0);
    RF_CTRL(0);
    RF_DATA(1);
    RF_CTRL(1);
    RF_DATA(0);
    RF_CTRL(0);
    RF_DATA(1);
    RF_CTRL(1);
    RF_DATA(0);
    RF_CTRL(0);
    RF_DATA(1);
    RF_CTRL(1);
    RF_DATA(0);
    RF_CTRL(0);
    RF_DATA(1);
    RF_CTRL(1);
    RF_DATA(0);
    RF_CTRL(0);
    RF_DATA(1);
    RF_CTRL(1);
    RF_DATA(0);
    RF_CTRL(0);
    RF_DATA(1);
    RF_CTRL(1);
    RF_DATA(0);
    RF_CTRL(0);
    RF_DATA(1);
    RF_CTRL(1);
    RF_DATA(0);
    RF_CTRL(0);
    RF_DATA(1);
    RF_CTRL(1);
    RF_DATA(0);
    RF_CTRL(0);
}

/*
 * CRC algorithm implementation
 *
 * \param[IN] crc Previous CRC value
 * \param[IN] data New data to be added to the CRC
 * \param[IN] polynomial CRC polynomial selection [CRC_TYPE_CCITT, CRC_TYPE_IBM]
 *
 * \retval crc New computed CRC
 */
U16 ComputeCrc( U16 crc, U8 data, U16 polynomial )
{
    U8 i;
    for( i = 0; i < 8; i++ )
    {
        if( ( ( ( crc & 0x8000 ) >> 8 ) ^ ( data & 0x80 ) ) != 0 )
        {
            crc <<= 1;              // shift left once
            crc ^= polynomial;      // XOR with polynomial
        }
        else
        {
            crc <<= 1;              // shift left once
        }
        data <<= 1;                 // Next data bit
    }
    return crc;
}

/*
 * Packet data CRC computation
 *
 * \param[IN] radioPacket Radio packet structure containing all information
 *
 * \retval crc Packet computed CRC
 */
U16 RadioPacketComputeCrc( tTxParam *TxParam )
{
    U8 i;
    U16 crc;
    U16 polynomial;

    polynomial = ( TxParam->Config.bits.CrcType == CRC_TYPE_IBM ) ? POLYNOMIAL_IBM : POLYNOMIAL_CCITT;
    crc = ( TxParam->Config.bits.CrcType == CRC_TYPE_IBM ) ? CRC_IBM_SEED : CRC_CCITT_SEED;

    if( TxParam->Config.bits.Format == 1 )
    {
        crc = ComputeCrc( crc, TxParam->MessageSize, polynomial );
    }
    if( TxParam->Config.bits.NodeAddressInPayload == 1 )
    {
        crc = ComputeCrc( crc, TxParam->NodeAddress, polynomial );
    }

    for( i = 0; i < TxParam->BufferSize; i++ )
    {
        crc = ComputeCrc( crc, TxParam->buffer[i], polynomial );
    }
    if( TxParam->Config.bits.CrcType == CRC_TYPE_IBM )
    {
        return crc;
    }
    else
    {
        return ( U16 )( ~crc );
    }
}

U8 SX1243Init( void )
{
    //SetMcuIO( );

    SX1243Reset( );

    // Packet configuration
    TxParam.Config.bits.SyncSize   = 4;
    TxParam.Config.bits.CrcOn      = CRC_ON;
    TxParam.Config.bits.CrcType    = CRC_TYPE_CCITT;
    TxParam.Config.bits.Format     = VARIABLE_PACKET_LENGTH;
    TxParam.Config.bits.NodeAddressInPayload = ADDRESS_NOT_IN_PAYLOAD;
    TxParam.PreambleSize           = 5;
    TxParam.NodeAddress            = 0x00;
    TxParam.MessageSize            = 16;
    TxParam.BufferSize             = 16;

    // sx1243 configuration
    TxParam.Mode                   = MODE_AUTOMATIC;
    TxParam.Modulation             = MODULATION_FSK;
    TxParam.Band                   = BAND_1;
    TxParam.FDev                   = FDEV;
    TxParam.RfPower                = POWER_10DBM;
    TxParam.OffTimer               = TIMEOFF_2MS;
    TxParam.Reserved               = RESERVED;

    // radio configuration
    TxParam.Freq                               = FRF;
    TxParam.Bitrate                            = RF_BITRATE;
    TxParam.freqHopping.bits.Enable            = DISABLE;
    TxParam.freqHopping.bits.ChannelNumber     = 1;
    TxParam.Repeat                             = 0;
    TxParam.Delay                              = 100;   // 100ms delay between each packet
    TxParam.FreqHoppingChannelStep             = 0;

    Bitrate = ComputeBitrate( TxParam.Bitrate );

    // Small correction of the bitrate value
    // compensates the executed instructions.
     Bitrate += 32;
    
    return SX_OK;
}

void SX1243Reset( void )
{
    RF_RESET(0); // Set NRESET pin to 0

    // Wait 100ms
    delay_ms(100);
    
    RF_RESET(1); // Set NRESET pin to 1
}

/*!
 * \brief Writes the register at the specified address
 *
 * \param [IN]: instruction Register instruction
 * \param [IN]: data New register value
 * \retval status [OK]
 */
U8 SX1243Write( U8 instruction, U32 data )
{
    switch( instruction )
    {
    case INSTRUCTION_CONFIG: // Config
        SpiOut( INST_WRITE_CFG );
        SpiOut( ( data >> 8 ) & 0xFF );
        SpiOut( data & 0xFF );
        break;
    case INSTRUCTION_FREQUENCY: // Freq
        SpiOut( ( INST_WRITE_FREQ | ( data & 0x070000 ) ) >> 16 );
        SpiOut( ( data >> 8 ) & 0xFF );
        SpiOut( data & 0xFF );
        break;
    case INSTRUCTION_TEST: // Test
        SpiOut( INST_WRITE_TEST >> 16 );
        SpiOut( INST_WRITE_TEST | ( ( data >> 8 ) & 0x07 ) );
        SpiOut( data & 0xFF );
        break;
    case INSTRUCTION_RECOVERY: // Recovery
        SpiOutRecovery( );
        break;
    default:
        return SX_ERROR;
    }
    RF_DATA(0);
    return SX_OK;
}

/*!
 * \brief Reads the register at the specified address
 *
 * \param [IN]: instruction Register instruction
 * \param [OUT]: data Register value
 * \retval status [OK]
 */
U8 SX1243Read( U8 instruction, U32 *data )
{
    U8 stat = SX_OK;

    switch( instruction )
    {
    case INSTRUCTION_CONFIG: // Config
        SpiOut( INST_READ_CFG >> 16 );
        *data = ( ( U16 )SpiIn( ) << 8 ) | SpiIn( );
        break;
    case INSTRUCTION_FREQUENCY: // Freq
        SpiOut( INST_READ_FREQ >> 16 );
        *data = ( ( U16 )SpiIn( ) << 8 ) | SpiIn( );
        SpiOut( INST_READ_STAT >> 16 );
        *data |= ( U32 )( ( ( ( U16 )SpiIn( ) << 8 ) | SpiIn( ) ) & 0x07 ) << 16 ;
        break;
    case INSTRUCTION_STATUS: // Status
        SpiOut( INST_READ_STAT >> 16 );
        *data = ( ( U16 )SpiIn( ) << 8 ) | SpiIn( );
        break;
    case INSTRUCTION_BIST: // Bist
        SpiOut( INST_READ_BIST >> 16 );
        *data = ( ( U16 )SpiIn( ) << 8 ) | SpiIn( );
        break;
    case INSTRUCTION_TEST: // Test
        SpiOut( INST_READ_TEST >> 16 );
        *data = ( ( U16 )SpiIn( ) << 8 ) | SpiIn( );
        break;
    default:
        *data = 0;
        stat = SX_ERROR;
        break;
    }
    
    RF_DATA(0);
    return stat;
}

U8 SX1243SetTxSyncValue( U8 *SyncValue, U8 size )
{
    if( size > 8 )
    {
        return SX_UNSUPPORTED;
    }
    else
    {
        TxParam.Config.bits.SyncSize = size;
        TxParam.syncValue = SyncValue;
    }
    return SX_OK;

}

U8 SX1243SetTxPacketBuffer( U8 *PayloadBuffer, U8 size )
{
    TxParam.MessageSize = size;
    TxParam.buffer = PayloadBuffer;
   
    if( TxParam.Config.bits.CrcOn == CRC_ON )
    {
        TxParam.CrcValue = RadioPacketComputeCrc( &TxParam );
    }
    
    return SX_OK;
}

U8 SX1243Process( void )
{
    U16 BitMask = 0x80;
    U8 ByteCounter = 0;
    U8 PacketTxState = 0;
    U8 hopCnt;
    S32 freqRange;
    U16 config = 0;
    U8 packetCnt = 0;

    RF_DATA(0);
    RF_CTRL(0);

    Bitrate = ComputeBitrate( TxParam.Bitrate );
    // Small correction of the bitrate value
    // compensates the executed instructions.
    //Bitrate += 20;


    INIT_BITRATE_HANDLING_TIMER( );

    while ( ( packetCnt == 0 ) || ( packetCnt < TxParam.Repeat ))
    {
        packetCnt++;

        RESET_BITRATE_HANDLING_TIMER( );

        // Set new config and Force Tx
        config = TxParam.Mode | TxParam.Modulation | TxParam.Band |
                 TxParam.FDev | TxParam.RfPower | TxParam.OffTimer | TxParam.Reserved;

        // Force transmission
        config |= 0x8000;
        SX1243Write( INSTRUCTION_CONFIG, config );

        if( TxParam.freqHopping.bits.Enable == DISABLE )
        {
            TxParam.freqHopping.bits.ChannelNumber = 1;
            // Set new channel frequency
            SX1243Write( INSTRUCTION_FREQUENCY, TxParam.Freq );
        }
        else
        {
            freqRange = TxParam.freqHopping.bits.ChannelNumber / 2;
            srandl( 1 );
        }

        // We disable all the interrupt to ensure the bitrate handling function is not altered
        DisableInterruptsHandler( );

        for( hopCnt = 0; hopCnt < TxParam.freqHopping.bits.ChannelNumber; hopCnt++ )
        {
            if( TxParam.freqHopping.bits.Enable == ENABLE )
            {
                // Set new hop frequency
                SX1243Write( INSTRUCTION_FREQUENCY, TxParam.Freq + randr( -freqRange, freqRange ) * TxParam.FreqHoppingChannelStep );
            }

            // Wait for Tx Ready
            // REMARK: Can be replaced by Status register reading when using 2 pins
            // interface
            while( RF_TX_RDY == 0 );

            ENABLE_BITRATE_TIMER( );

            PacketTxState = 0;
            BitMask = 0x80;

            while( PacketTxState < 7 )
            {
                switch( PacketTxState )
                {
                case 0:                   
                    if( TxParam.PreambleSize > 0 )
                    {
                        RF_DATA (( 0x55 & BitMask ) ? 1 : 0);
                        BitMask >>= 1;
                        if( BitMask == 0 )
                        {
                            BitMask = 0x80;
                            ByteCounter++;
                            if( ByteCounter >= TxParam.PreambleSize )
                            {
                                ByteCounter = 0;
                                PacketTxState = 1;
                            }
                        }
                        BITRATE_HANDLING_TIMER_WAIT( );
                        break;
                    }
                case 1:
                    if( TxParam.Config.bits.SyncSize > 0 )
                    {
                        RF_DATA (( TxParam.syncValue[ ByteCounter ] & BitMask ) ? 1 : 0);
                        BitMask >>= 1;
                        if( BitMask == 0 )
                        {
                            BitMask = 0x80;
                            ByteCounter++;
                            if( ByteCounter >= TxParam.Config.bits.SyncSize )
                            {
                                ByteCounter = 0;
                                PacketTxState = 2;
                            }
                        }
                        BITRATE_HANDLING_TIMER_WAIT( );
                        break;
                    }
                case 2:
                    if( TxParam.Config.bits.Format == VARIABLE_PACKET_LENGTH )
                    {
                        RF_DATA(( TxParam.MessageSize & BitMask ) ? 1 : 0);
                        BitMask >>= 1;
                        if( BitMask == 0 )
                        {
                            BitMask = 0x80;
                            ByteCounter++;
                            if( ByteCounter >= 1 )
                            {
                                ByteCounter = 0;
                                PacketTxState = 3;
                            }
                        }
                        BITRATE_HANDLING_TIMER_WAIT( );
                        break;
                    }
                case 3:
                    if( TxParam.Config.bits.NodeAddressInPayload == ADDRESS_IN_PAYLOAD )
                    {
                        RF_DATA(( TxParam.NodeAddress & BitMask ) ? 1 : 0);
                        BitMask >>= 1;
                        if( BitMask == 0 )
                        {
                            BitMask = 0x80;
                            ByteCounter++;
                            if( ByteCounter >= 1 )
                            {
                                ByteCounter = 0;
                                PacketTxState = 4;
                            }
                        }
                        BITRATE_HANDLING_TIMER_WAIT( );
                        break;
                    }
                case 4:
                    if( TxParam.BufferSize > 0 )
                    {
                        RF_DATA(( TxParam.buffer[ ByteCounter ] & BitMask ) ? 1 : 0);
                        BitMask >>= 1;
                        if( BitMask == 0 )
                        {
                            BitMask = 0x80;
                            ByteCounter++;
                            if( ByteCounter >= TxParam.BufferSize )
                            {
                                ByteCounter = 0;
                                PacketTxState = 5;
                                if( TxParam.Config.bits.CrcOn == CRC_ON )
                                {
                                    BitMask = 0x8000;
                                }
                            }
                        }
                        BITRATE_HANDLING_TIMER_WAIT( );
                        break;
                    }
                case 5:
                    if( TxParam.Config.bits.CrcOn == CRC_ON )
                    {
                        RF_DATA(( TxParam.CrcValue & BitMask ) ? 1 : 0);
                        BitMask >>= 1;
                        if( BitMask == 0 )
                        {
                            BitMask = 0x80;
                            ByteCounter++;
                            if( ByteCounter >= 1 )
                            {
                                ByteCounter = 0;
                                PacketTxState = 6;
                            }
                        }
                        BITRATE_HANDLING_TIMER_WAIT( );
                        break;
                    }
                case 6:
                    DISABLE_BITRATE_TIMER( );
                    PacketTxState = 7;
                    break;
                }
            }
        }

        // Remove force transmission
        config = TxParam.Mode | TxParam.Modulation | TxParam.Band |
                 TxParam.FDev | TxParam.RfPower | TxParam.OffTimer | TxParam.Reserved;
        SX1243Write( INSTRUCTION_CONFIG, config );

        // Reload original frequency (only necessary in when frequency hopping is used)
        SX1243Write( INSTRUCTION_FREQUENCY, TxParam.Freq );

        EnableInterruptsHandler( );

        RF_DATA(0);
        RF_CTRL(0);

        // delay before next transmission
        StandardWait( TxParam.Delay );
    }
    return SX_OK;
}
