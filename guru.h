// guru.h

#include "inc/hw_memmap.h"

#ifndef _GURU_H_
#define _GURU_H_ value


// Supplied XTAL Frequency
#define XTAL_HZ 16000000

#define DEBUG_UART			UART0_BASE
#define UPSTREAM_UART		UART4_BASE
#define DNSTREAM_UART		UART3_BASE
#define BAUD_RATE			115200

#define I2C_SLAVE			I2C0_BASE
#define I2C_MASTER			I2C1_BASE

#define SPI_EEPROM			SSI0_BASE
#define SPI_SD_CARD			SSI2_BASE

#define AM2320				0x5C		// 0xB8 >> 1





extern void Guru_Init(void);

extern void ConfigureUART(void);

extern int AM2320Read(uint16_t *p_tempature, uint16_t *p_humidity);

#endif