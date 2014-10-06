#ifndef __BMP180_H
#define __BMP180_H

#ifdef __cplusplus
 extern "C" {
#endif
	 
#include "stm32f10x.h"
#include "delay.h"
#include "usart.h"

#define SW_I2C

#if defined(SW_I2C)
	#include "sw_i2c.h"
#else 
	#include "hw_i2c.h"
#endif

#define BMP_DEVADDR		0xEE	//BMP180 I2C address

#define BMP180_PDEBUG
#ifdef BMP180_PDEBUG
#define BMP180_DEBUG(string,args...)	printf("[BMP180_DEBUG] ");	\
									printf(string, ##args)
#else
#define BMP180_DEBUG(string,args...)
#endif	

//BMP180 register address define
#define CTRL_ADDR			0xF4

#define TEMP_CTRL_DATA		0x2E

#define PRESS_CTRL_0_DATA	0x34	//ultra low power
#define PRESS_CTRL_1_DATA	0x74	//standard
#define PRESS_CTRL_2_DATA	0xB4	//high resolution
#define PRESS_CTRL_3_DATA	0xF4	//ultra high resolution
#define PRESS_CTRL_DATA		PRESS_CTRL_0_DATA

#define OSRS_0			0
#define OSRS_1			1
#define OSRS_2			2
#define OSRS_3			3
#define OSRS			OSRS_1

#define CONV_TIME_0		delay_ms(5)		// 4.5ms
#define CONV_TIME_1		delay_ms(8)		// 7.5ms
#define CONV_TIME_2		delay_ms(14)	// 13.5ms
#define CONV_TIME_3		delay_ms(26)	// 25.5ms
#define CONV_TIME		CONV_TIME_1

#define DATA_MSB		0xF6
#define DATA_LSB		0xF7
#define DATA_XLSB		0xF8

#define EEPROM_BASE		0xAA
#define AC1_MSB			EEPROM_BASE
#define AC1_LSB			(EEPROM_BASE+1)
#define AC2_MSB			(EEPROM_BASE+2)
#define AC2_LSB			(EEPROM_BASE+3)
#define AC3_MSB			(EEPROM_BASE+4)
#define AC3_LSB			(EEPROM_BASE+5)
#define AC4_MSB			(EEPROM_BASE+6)
#define AC4_LSB			(EEPROM_BASE+7)
#define AC5_MSB			(EEPROM_BASE+8)
#define AC5_LSB			(EEPROM_BASE+9)
#define AC6_MSB			(EEPROM_BASE+10)
#define AC6_LSB			(EEPROM_BASE+11)
#define B1_MSB			(EEPROM_BASE+12)
#define B1_LSB			(EEPROM_BASE+13)
#define B2_MSB			(EEPROM_BASE+14)
#define B2_LSB			(EEPROM_BASE+15)
#define MB_MSB			(EEPROM_BASE+16)
#define MB_LSB			(EEPROM_BASE+17)
#define MC_MSB			(EEPROM_BASE+18)
#define MC_LSB			(EEPROM_BASE+19)
#define MD_MSB			(EEPROM_BASE+20)
#define MD_LSB			(EEPROM_BASE+21)

bool bmp180_getUdata(int16_t *m_UT, long *m_UP);
bool bmp180_getData(long *temp, long *pres);

#ifdef __cplusplus
}
#endif

#endif /* __BMP180_H */

