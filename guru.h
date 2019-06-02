// guru.h

#include "inc/hw_memmap.h"


#ifndef _GURU_H_
#define _GURU_H_

int g_error_counter;
int g_err_addr_ack;
int g_err_data_ack;
int g_err_arb_lost;
int g_err_clk_tout;
int g_total_transactions;
int g_i2c_device_pres;

uint32_t pui32ADC0Value;

// Supplied XTAL Frequency
#define XTAL_HZ 					16000000

#define DEBUG_UART					UART0_BASE
#define UPSTREAM_UART				UART4_BASE
#define DNSTREAM_UART				UART3_BASE
#define BAUD_RATE					115200

#define I2C_SLAVE					I2C0_BASE
#define I2C_MASTER					I2C1_BASE
	
#define SPI_EEPROM					SSI2_BASE
#define SPI_SD_CARD					SSI0_BASE

#define I2C_READ					true
#define I2C_WRITE					false

#define AM2320						0x5C		// 0xB8 >> 1
#define AM2320_PRES					0x01

#define DS1621 						0x48
#define DS1621_PRES					0x02
#define DS1621_READ_TEMP			0xAA
#define DS1621_ACCESS_HIGH			0xA1
#define DS1621_ACCESS_LOW			0xA2
#define DS1621_ACCESS_CONFIG		0xAC
#define DS1621_READ_COUNTER			0xA8
#define DS1621_READ_SLOPE			0xA9
#define DS1621_START_CONVERT		0xEE
#define DS1621_STOP_CONVERT			0x22


#define SI7006						0x40		// 0x80 >> 1
#define SI7006_PRES					0x04
#define SI7006_MEASURE_RH_HOLD		0xE5
#define SI7006_MEASURE_RH 			0xF5
#define SI7006_MEASURE_TEMP_HOLD 	0xE3
#define SI7006_MEASURE_TEMP 		0xF3
#define SI7006_READ_TEMP 			0xE0
#define SI7006_RESET 				0xFE 
#define SI7006_WRITE_USER_REG 		0xE6
#define SI7006_READ_USER_REG 		0xE7
#define SI7006_WRITE_HEAT_CON_REG 	0x51
#define SI7006_READ_HEAT_CON_REG 	0x11
#define SI7006_READ_ID_REG_1 		0xFA
#define SI7006_READ_ID_REG_2 		0xFC
#define SI7006_READ_FW_VERSION 		0x84


#define EEPROM_ERASE				0b111
#define EEPROM_ERASE_ALL			0b10010
#define EEPROM_WRITE_ENABLE			0b10011
#define EEPROM_WRITE_DISABLE		0b10000
#define EEPROM_READ 				0b110
#define EEPROM_WRITE 				0b101

#ifdef DEBUG
    #define DEBUG_PRINT UARTprintf
#else
    #define DEBUG_PRINT
#endif

extern void Guru_Init(void);

extern int I2C_Scan(void);

extern void I2C_Error_Check(int error_code);

extern void ConfigureUART(void);

extern int AM2320Read(int16_t *p_tempature, uint16_t *p_humidity);

extern int DS1621Read(int8_t *p_tempature);

extern int SI7006Read(int16_t *p_tempature, uint16_t *p_humidity);

extern uint32_t CheckLightSensor();

extern uint16_t CheckMoistureSensor();

extern int PrintCounters();

#endif