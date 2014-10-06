/******************************************************************************
  File			: bmp180.c
  Description	: bmp180 barometric pressure sensor driver
  Author		: Xiaoming Li
*******************************************************************************
  Modify List:
-------------------------------------------------------------------------------
  2014/9/6 21:00 PM	| Created
******************************************************************************/

#include "bmp180.h"

uint8_t mbuffer[22];

static bool firstReadCali = TRUE;
static int16_t AC1, AC2, AC3, B1, B2, /*MB, */MC, MD;
static uint16_t AC4, AC5, AC6;

bool bmp180_getUdata(int16_t *m_UT, long *m_UP)
{
	if(!SwI2C_WriteByte(BMP_DEVADDR, CTRL_ADDR, TEMP_CTRL_DATA)){	//start measure temperature
		BMP180_DEBUG("Start measure Temp Fail!\r\n");
		return FALSE;
	}
	CONV_TIME_0;	//wait 4.5ms 
	if(!SwI2C_ReadBytes(BMP_DEVADDR, DATA_MSB, 2, mbuffer)){		//read temperature
		BMP180_DEBUG("Read Temp data Fail!\r\n");
		return FALSE;
	}
	*m_UT = (((int16_t)mbuffer[0]) << 8) | mbuffer[1];

	if(!SwI2C_WriteByte(BMP_DEVADDR, CTRL_ADDR, (PRESS_CTRL_DATA+(OSRS<<6)))){	//start measure pressure
		BMP180_DEBUG("Start measure Pressure Fail!\r\n");
		return FALSE;
	}
	CONV_TIME;	//wait 7.5ms
	if(!SwI2C_ReadBytes(BMP_DEVADDR, DATA_MSB, 3, mbuffer)){
		BMP180_DEBUG("Read Pressure data Fail!\r\n");
		return FALSE;
	}
	*m_UP = ((((long)mbuffer[0]) << 16) | (((long)mbuffer[1]) << 8) | mbuffer[0]) >> (8 - OSRS);
	BMP180_DEBUG("UT=%d UP=%d\r\n", *m_UT, *m_UP);

	return TRUE;
}

bool bmp180_getData(long *temp, long *pres)
{
	int16_t UT;
	long UP;
	long X1, X2, X3, B3, B5, B6, B7;
	unsigned long B4;

	//get calibration data
	//only first read data for calculate,because follow data is constant.
	if(firstReadCali){
		firstReadCali = FALSE;
		if(!SwI2C_ReadBytes(BMP_DEVADDR, DATA_MSB, 22, mbuffer)){
			BMP180_DEBUG("Read calibration data Fail!\r\n");
			return FALSE;
		}
		AC1 = (((int16_t)mbuffer[0]) << 8) | mbuffer[1];
		AC2 = (((int16_t)mbuffer[2]) << 8) | mbuffer[3];
		AC3 = (((int16_t)mbuffer[4]) << 8) | mbuffer[5];
		AC4 = (((uint16_t)mbuffer[6]) << 8) | mbuffer[7];
		AC5 = (((uint16_t)mbuffer[8]) << 8) | mbuffer[9];
		AC6 = (((uint16_t)mbuffer[10]) << 8) | mbuffer[11];
		B1 = (((int16_t)mbuffer[12]) << 8) | mbuffer[13];
		B2 = (((int16_t)mbuffer[14]) << 8) | mbuffer[15];
		//MB = (((int16_t)mbuffer[16]) << 8) | mbuffer[17];
		MC = (((int16_t)mbuffer[18]) << 8) | mbuffer[19];
		MD = (((int16_t)mbuffer[20]) << 8) | mbuffer[21];
	}

	if(!bmp180_getUdata(&UT, &UP)){
		BMP180_DEBUG("get data Fail!\r\n");
		return FALSE;
	}

	X1 = (((long)UT - AC6) * AC5)>>15;
	X2 = ((long)MC<<11) / (X1 + MD);
	B5 = X1 + X2;
	*temp = (B5 + 8)>>4;

	B6 = B5 - 4000;
	X1 = (B2 * ((B6 * B6)>>12))>>11;
	X2 = (AC2 * B6)>>11;
	X3 = X1 + X2;
	B3 = (((long)AC1 * 4 + X3) << OSRS + 2)>>2;
	X1 = (AC3 * B6)>>13;
	X2 = (B1 * ((B6 * B6)>>12))>>16;
	X3 = ((X1 + X2) + 2)>>2;
	B4 = (AC4 * (unsigned long)(X3 + 32768))>>15;
	B7 = ((unsigned long)UP - B3) * (50000 >> OSRS);
	if(B7 < 0x80000000){
		*pres = (B7 * 2) / B4;
	}
	else{
		*pres = (B7 / B4) * 2;
	}
	X1 = ((*pres)>>8) * ((*pres)>>8);
	X1 = (X1 * 3038)>>16;
	X2 = (-7357 * (*pres))>>16;
	*pres += ((X1 + X2 +  3791)>>4);

	return TRUE;
}
