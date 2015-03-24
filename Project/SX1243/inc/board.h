/******************************************************************************
 * Project        : STM8L_Discovery_SX1278_SFM1L
 * File           : board.h
 * Copyright      : 2014 Yosun Singapore Pte Ltd
 ******************************************************************************
  Change History:

    Version 1.0 - Sep 2014
    > Initial revision

******************************************************************************/
#ifndef _BOARD_H_
#define _BOARD_H_

#define REMOTE_MAX_NUMBER       1
#define VERSION_MAX_SIZE        5
#if defined(STM8S003)
#define FIRMWARE_VERSION_ADDRESS        0x4000
#elif defined(STM8L15X_MD)
#define FIRMWARE_VERSION_ADDRESS        ((uint32_t)0x00001000)
#endif
#define DEVICE_PARAMETERS_ADDRESS       (FIRMWARE_VERSION_ADDRESS + VERSION_MAX_SIZE)

typedef struct t_DeviceParameters
{
  unsigned char hostID;
  unsigned char remoteID;
}s_DeviceParameters;

void Board_Init(void);
void LoRaRX_Indicate(void);
void Uart_Prints(unsigned char *p_data, int length);
//void EEPROM_Write(unsigned short address, unsigned char *p_data, unsigned short len);
//void EEPROM_Read(unsigned short address, unsigned char *p_data, unsigned short len);
#endif /* _BOARD_H_ */
