#ifndef __MPU6050_H
#define __MPU6050_H

#ifdef __cplusplus
 extern "C" {
#endif
	 
#include "stm32f10x.h"
#include "delay.h"
#include "usart.h"
#include "math.h"
#include "value_struct.h"
	 
#define DEVADDR (MPU6050_DEFAULT_ADDRESS << 1)

#define MPU6050_ADDRESS_AD0_LOW     0x68 // address pin low (GND), default for InvenSense evaluation board
#define MPU6050_ADDRESS_AD0_HIGH    0x69 // address pin high (VCC)
#define MPU6050_DEFAULT_ADDRESS     MPU6050_ADDRESS_AD0_LOW

#define MPU6050_RA_XG_OFFS_TC       0x00 //[7] PWR_MODE, [6:1] XG_OFFS_TC, [0] OTP_BNK_VLD
#define MPU6050_RA_YG_OFFS_TC       0x01 //[7] PWR_MODE, [6:1] YG_OFFS_TC, [0] OTP_BNK_VLD
#define MPU6050_RA_ZG_OFFS_TC       0x02 //[7] PWR_MODE, [6:1] ZG_OFFS_TC, [0] OTP_BNK_VLD
#define MPU6050_RA_X_FINE_GAIN      0x03 //[7:0] X_FINE_GAIN
#define MPU6050_RA_Y_FINE_GAIN      0x04 //[7:0] Y_FINE_GAIN
#define MPU6050_RA_Z_FINE_GAIN      0x05 //[7:0] Z_FINE_GAIN
#define MPU6050_RA_XA_OFFS_H        0x06 //[15:0] XA_OFFS
#define MPU6050_RA_XA_OFFS_L_TC     0x07
#define MPU6050_RA_YA_OFFS_H        0x08 //[15:0] YA_OFFS
#define MPU6050_RA_YA_OFFS_L_TC     0x09
#define MPU6050_RA_ZA_OFFS_H        0x0A //[15:0] ZA_OFFS
#define MPU6050_RA_ZA_OFFS_L_TC     0x0B
#define MPU6050_RA_XG_OFFS_USRH     0x13 //[15:0] XG_OFFS_USR
#define MPU6050_RA_XG_OFFS_USRL     0x14
#define MPU6050_RA_YG_OFFS_USRH     0x15 //[15:0] YG_OFFS_USR
#define MPU6050_RA_YG_OFFS_USRL     0x16
#define MPU6050_RA_ZG_OFFS_USRH     0x17 //[15:0] ZG_OFFS_USR
#define MPU6050_RA_ZG_OFFS_USRL     0x18
#define MPU6050_RA_SMPLRT_DIV       0x19
#define MPU6050_RA_CONFIG           0x1A
#define MPU6050_RA_GYRO_CONFIG      0x1B
#define MPU6050_RA_ACCEL_CONFIG     0x1C
#define MPU6050_RA_FF_THR           0x1D
#define MPU6050_RA_FF_DUR           0x1E
#define MPU6050_RA_MOT_THR          0x1F
#define MPU6050_RA_MOT_DUR          0x20
#define MPU6050_RA_ZRMOT_THR        0x21
#define MPU6050_RA_ZRMOT_DUR        0x22
#define MPU6050_RA_FIFO_EN          0x23
#define MPU6050_RA_I2C_MST_CTRL     0x24
#define MPU6050_RA_I2C_SLV0_ADDR    0x25
#define MPU6050_RA_I2C_SLV0_REG     0x26
#define MPU6050_RA_I2C_SLV0_CTRL    0x27
#define MPU6050_RA_I2C_SLV1_ADDR    0x28
#define MPU6050_RA_I2C_SLV1_REG     0x29
#define MPU6050_RA_I2C_SLV1_CTRL    0x2A
#define MPU6050_RA_I2C_SLV2_ADDR    0x2B
#define MPU6050_RA_I2C_SLV2_REG     0x2C
#define MPU6050_RA_I2C_SLV2_CTRL    0x2D
#define MPU6050_RA_I2C_SLV3_ADDR    0x2E
#define MPU6050_RA_I2C_SLV3_REG     0x2F
#define MPU6050_RA_I2C_SLV3_CTRL    0x30
#define MPU6050_RA_I2C_SLV4_ADDR    0x31
#define MPU6050_RA_I2C_SLV4_REG     0x32
#define MPU6050_RA_I2C_SLV4_DO      0x33
#define MPU6050_RA_I2C_SLV4_CTRL    0x34
#define MPU6050_RA_I2C_SLV4_DI      0x35
#define MPU6050_RA_I2C_MST_STATUS   0x36
#define MPU6050_RA_INT_PIN_CFG      0x37
#define MPU6050_RA_INT_ENABLE       0x38
#define MPU6050_RA_DMP_INT_STATUS   0x39
#define MPU6050_RA_INT_STATUS       0x3A
#define MPU6050_RA_ACCEL_XOUT_H     0x3B
#define MPU6050_RA_ACCEL_XOUT_L     0x3C
#define MPU6050_RA_ACCEL_YOUT_H     0x3D
#define MPU6050_RA_ACCEL_YOUT_L     0x3E
#define MPU6050_RA_ACCEL_ZOUT_H     0x3F
#define MPU6050_RA_ACCEL_ZOUT_L     0x40
#define MPU6050_RA_TEMP_OUT_H       0x41
#define MPU6050_RA_TEMP_OUT_L       0x42
#define MPU6050_RA_GYRO_XOUT_H      0x43
#define MPU6050_RA_GYRO_XOUT_L      0x44
#define MPU6050_RA_GYRO_YOUT_H      0x45
#define MPU6050_RA_GYRO_YOUT_L      0x46
#define MPU6050_RA_GYRO_ZOUT_H      0x47
#define MPU6050_RA_GYRO_ZOUT_L      0x48
#define MPU6050_RA_EXT_SENS_DATA_00 0x49
#define MPU6050_RA_EXT_SENS_DATA_01 0x4A
#define MPU6050_RA_EXT_SENS_DATA_02 0x4B
#define MPU6050_RA_EXT_SENS_DATA_03 0x4C
#define MPU6050_RA_EXT_SENS_DATA_04 0x4D
#define MPU6050_RA_EXT_SENS_DATA_05 0x4E
#define MPU6050_RA_EXT_SENS_DATA_06 0x4F
#define MPU6050_RA_EXT_SENS_DATA_07 0x50
#define MPU6050_RA_EXT_SENS_DATA_08 0x51
#define MPU6050_RA_EXT_SENS_DATA_09 0x52
#define MPU6050_RA_EXT_SENS_DATA_10 0x53
#define MPU6050_RA_EXT_SENS_DATA_11 0x54
#define MPU6050_RA_EXT_SENS_DATA_12 0x55
#define MPU6050_RA_EXT_SENS_DATA_13 0x56
#define MPU6050_RA_EXT_SENS_DATA_14 0x57
#define MPU6050_RA_EXT_SENS_DATA_15 0x58
#define MPU6050_RA_EXT_SENS_DATA_16 0x59
#define MPU6050_RA_EXT_SENS_DATA_17 0x5A
#define MPU6050_RA_EXT_SENS_DATA_18 0x5B
#define MPU6050_RA_EXT_SENS_DATA_19 0x5C
#define MPU6050_RA_EXT_SENS_DATA_20 0x5D
#define MPU6050_RA_EXT_SENS_DATA_21 0x5E
#define MPU6050_RA_EXT_SENS_DATA_22 0x5F
#define MPU6050_RA_EXT_SENS_DATA_23 0x60
#define MPU6050_RA_MOT_DETECT_STATUS    0x61
#define MPU6050_RA_I2C_SLV0_DO      0x63
#define MPU6050_RA_I2C_SLV1_DO      0x64
#define MPU6050_RA_I2C_SLV2_DO      0x65
#define MPU6050_RA_I2C_SLV3_DO      0x66
#define MPU6050_RA_I2C_MST_DELAY_CTRL   0x67
#define MPU6050_RA_SIGNAL_PATH_RESET    0x68
#define MPU6050_RA_MOT_DETECT_CTRL      0x69
#define MPU6050_RA_USER_CTRL        0x6A
#define MPU6050_RA_PWR_MGMT_1       0x6B
#define MPU6050_RA_PWR_MGMT_2       0x6C
#define MPU6050_RA_BANK_SEL         0x6D
#define MPU6050_RA_MEM_START_ADDR   0x6E
#define MPU6050_RA_MEM_R_W          0x6F
#define MPU6050_RA_DMP_CFG_1        0x70
#define MPU6050_RA_DMP_CFG_2        0x71
#define MPU6050_RA_FIFO_COUNTH      0x72
#define MPU6050_RA_FIFO_COUNTL      0x73
#define MPU6050_RA_FIFO_R_W         0x74
#define MPU6050_RA_WHO_AM_I         0x75

#define MPU6050_TC_PWR_MODE_BIT     7
#define MPU6050_TC_OFFSET_BIT       6
#define MPU6050_TC_OFFSET_LENGTH    6
#define MPU6050_TC_OTP_BNK_VLD_BIT  0

#define MPU6050_VDDIO_LEVEL_VLOGIC  0
#define MPU6050_VDDIO_LEVEL_VDD     1

#define MPU6050_CFG_EXT_SYNC_SET_BIT    5
#define MPU6050_CFG_EXT_SYNC_SET_LENGTH 3
#define MPU6050_CFG_DLPF_CFG_BIT    2
#define MPU6050_CFG_DLPF_CFG_LENGTH 3

#define MPU6050_EXT_SYNC_DISABLED       0x0
#define MPU6050_EXT_SYNC_TEMP_OUT_L     0x1
#define MPU6050_EXT_SYNC_GYRO_XOUT_L    0x2
#define MPU6050_EXT_SYNC_GYRO_YOUT_L    0x3
#define MPU6050_EXT_SYNC_GYRO_ZOUT_L    0x4
#define MPU6050_EXT_SYNC_ACCEL_XOUT_L   0x5
#define MPU6050_EXT_SYNC_ACCEL_YOUT_L   0x6
#define MPU6050_EXT_SYNC_ACCEL_ZOUT_L   0x7

#define MPU6050_DLPF_BW_256         0x00
#define MPU6050_DLPF_BW_188         0x01
#define MPU6050_DLPF_BW_98          0x02
#define MPU6050_DLPF_BW_42          0x03
#define MPU6050_DLPF_BW_20          0x04
#define MPU6050_DLPF_BW_10          0x05
#define MPU6050_DLPF_BW_5           0x06

#define MPU6050_GCONFIG_FS_SEL_BIT      4
#define MPU6050_GCONFIG_FS_SEL_LENGTH   2

#define MPU6050_GYRO_FS_250         0x00
#define MPU6050_GYRO_FS_500         0x01
#define MPU6050_GYRO_FS_1000        0x02
#define MPU6050_GYRO_FS_2000        0x03

#define MPU6050_ACONFIG_XA_ST_BIT           7
#define MPU6050_ACONFIG_YA_ST_BIT           6
#define MPU6050_ACONFIG_ZA_ST_BIT           5
#define MPU6050_ACONFIG_AFS_SEL_BIT         4
#define MPU6050_ACONFIG_AFS_SEL_LENGTH      2
#define MPU6050_ACONFIG_ACCEL_HPF_BIT       2
#define MPU6050_ACONFIG_ACCEL_HPF_LENGTH    3

#define MPU6050_ACCEL_FS_2          0x00
#define MPU6050_ACCEL_FS_4          0x01
#define MPU6050_ACCEL_FS_8          0x02
#define MPU6050_ACCEL_FS_16         0x03

#define MPU6050_DHPF_RESET          0x00
#define MPU6050_DHPF_5              0x01
#define MPU6050_DHPF_2P5            0x02
#define MPU6050_DHPF_1P25           0x03
#define MPU6050_DHPF_0P63           0x04
#define MPU6050_DHPF_HOLD           0x07

#define MPU6050_TEMP_FIFO_EN_BIT    7
#define MPU6050_XG_FIFO_EN_BIT      6
#define MPU6050_YG_FIFO_EN_BIT      5
#define MPU6050_ZG_FIFO_EN_BIT      4
#define MPU6050_ACCEL_FIFO_EN_BIT   3
#define MPU6050_SLV2_FIFO_EN_BIT    2
#define MPU6050_SLV1_FIFO_EN_BIT    1
#define MPU6050_SLV0_FIFO_EN_BIT    0

#define MPU6050_MULT_MST_EN_BIT     7
#define MPU6050_WAIT_FOR_ES_BIT     6
#define MPU6050_SLV_3_FIFO_EN_BIT   5
#define MPU6050_I2C_MST_P_NSR_BIT   4
#define MPU6050_I2C_MST_CLK_BIT     3
#define MPU6050_I2C_MST_CLK_LENGTH  4

#define MPU6050_CLOCK_DIV_348       0x0
#define MPU6050_CLOCK_DIV_333       0x1
#define MPU6050_CLOCK_DIV_320       0x2
#define MPU6050_CLOCK_DIV_308       0x3
#define MPU6050_CLOCK_DIV_296       0x4
#define MPU6050_CLOCK_DIV_286       0x5
#define MPU6050_CLOCK_DIV_276       0x6
#define MPU6050_CLOCK_DIV_267       0x7
#define MPU6050_CLOCK_DIV_258       0x8
#define MPU6050_CLOCK_DIV_500       0x9
#define MPU6050_CLOCK_DIV_471       0xA
#define MPU6050_CLOCK_DIV_444       0xB
#define MPU6050_CLOCK_DIV_421       0xC
#define MPU6050_CLOCK_DIV_400       0xD
#define MPU6050_CLOCK_DIV_381       0xE
#define MPU6050_CLOCK_DIV_364       0xF

#define MPU6050_I2C_SLV_RW_BIT      7
#define MPU6050_I2C_SLV_ADDR_BIT    6
#define MPU6050_I2C_SLV_ADDR_LENGTH 7
#define MPU6050_I2C_SLV_EN_BIT      7
#define MPU6050_I2C_SLV_BYTE_SW_BIT 6
#define MPU6050_I2C_SLV_REG_DIS_BIT 5
#define MPU6050_I2C_SLV_GRP_BIT     4
#define MPU6050_I2C_SLV_LEN_BIT     3
#define MPU6050_I2C_SLV_LEN_LENGTH  4

#define MPU6050_I2C_SLV4_RW_BIT         7
#define MPU6050_I2C_SLV4_ADDR_BIT       6
#define MPU6050_I2C_SLV4_ADDR_LENGTH    7
#define MPU6050_I2C_SLV4_EN_BIT         7
#define MPU6050_I2C_SLV4_INT_EN_BIT     6
#define MPU6050_I2C_SLV4_REG_DIS_BIT    5
#define MPU6050_I2C_SLV4_MST_DLY_BIT    4
#define MPU6050_I2C_SLV4_MST_DLY_LENGTH 5

#define MPU6050_MST_PASS_THROUGH_BIT    7
#define MPU6050_MST_I2C_SLV4_DONE_BIT   6
#define MPU6050_MST_I2C_LOST_ARB_BIT    5
#define MPU6050_MST_I2C_SLV4_NACK_BIT   4
#define MPU6050_MST_I2C_SLV3_NACK_BIT   3
#define MPU6050_MST_I2C_SLV2_NACK_BIT   2
#define MPU6050_MST_I2C_SLV1_NACK_BIT   1
#define MPU6050_MST_I2C_SLV0_NACK_BIT   0

#define MPU6050_INTCFG_INT_LEVEL_BIT        7
#define MPU6050_INTCFG_INT_OPEN_BIT         6
#define MPU6050_INTCFG_LATCH_INT_EN_BIT     5
#define MPU6050_INTCFG_INT_RD_CLEAR_BIT     4
#define MPU6050_INTCFG_FSYNC_INT_LEVEL_BIT  3
#define MPU6050_INTCFG_FSYNC_INT_EN_BIT     2
#define MPU6050_INTCFG_I2C_BYPASS_EN_BIT    1
#define MPU6050_INTCFG_CLKOUT_EN_BIT        0

#define MPU6050_INTMODE_ACTIVEHIGH  0x00
#define MPU6050_INTMODE_ACTIVELOW   0x01

#define MPU6050_INTDRV_PUSHPULL     0x00
#define MPU6050_INTDRV_OPENDRAIN    0x01

#define MPU6050_INTLATCH_50USPULSE  0x00
#define MPU6050_INTLATCH_WAITCLEAR  0x01

#define MPU6050_INTCLEAR_STATUSREAD 0x00
#define MPU6050_INTCLEAR_ANYREAD    0x01

#define MPU6050_INTERRUPT_FF_BIT            7
#define MPU6050_INTERRUPT_MOT_BIT           6
#define MPU6050_INTERRUPT_ZMOT_BIT          5
#define MPU6050_INTERRUPT_FIFO_OFLOW_BIT    4
#define MPU6050_INTERRUPT_I2C_MST_INT_BIT   3
#define MPU6050_INTERRUPT_PLL_RDY_INT_BIT   2
#define MPU6050_INTERRUPT_DMP_INT_BIT       1
#define MPU6050_INTERRUPT_DATA_RDY_BIT      0

// TODO: figure out what these actually do
// UMPL source code is not very obivous
#define MPU6050_DMPINT_5_BIT            5
#define MPU6050_DMPINT_4_BIT            4
#define MPU6050_DMPINT_3_BIT            3
#define MPU6050_DMPINT_2_BIT            2
#define MPU6050_DMPINT_1_BIT            1
#define MPU6050_DMPINT_0_BIT            0

#define MPU6050_MOTION_MOT_XNEG_BIT     7
#define MPU6050_MOTION_MOT_XPOS_BIT     6
#define MPU6050_MOTION_MOT_YNEG_BIT     5
#define MPU6050_MOTION_MOT_YPOS_BIT     4
#define MPU6050_MOTION_MOT_ZNEG_BIT     3
#define MPU6050_MOTION_MOT_ZPOS_BIT     2
#define MPU6050_MOTION_MOT_ZRMOT_BIT    0

#define MPU6050_DELAYCTRL_DELAY_ES_SHADOW_BIT   7
#define MPU6050_DELAYCTRL_I2C_SLV4_DLY_EN_BIT   4
#define MPU6050_DELAYCTRL_I2C_SLV3_DLY_EN_BIT   3
#define MPU6050_DELAYCTRL_I2C_SLV2_DLY_EN_BIT   2
#define MPU6050_DELAYCTRL_I2C_SLV1_DLY_EN_BIT   1
#define MPU6050_DELAYCTRL_I2C_SLV0_DLY_EN_BIT   0

#define MPU6050_PATHRESET_GYRO_RESET_BIT    2
#define MPU6050_PATHRESET_ACCEL_RESET_BIT   1
#define MPU6050_PATHRESET_TEMP_RESET_BIT    0

#define MPU6050_DETECT_ACCEL_ON_DELAY_BIT       5
#define MPU6050_DETECT_ACCEL_ON_DELAY_LENGTH    2
#define MPU6050_DETECT_FF_COUNT_BIT             3
#define MPU6050_DETECT_FF_COUNT_LENGTH          2
#define MPU6050_DETECT_MOT_COUNT_BIT            1
#define MPU6050_DETECT_MOT_COUNT_LENGTH         2

#define MPU6050_DETECT_DECREMENT_RESET  0x0
#define MPU6050_DETECT_DECREMENT_1      0x1
#define MPU6050_DETECT_DECREMENT_2      0x2
#define MPU6050_DETECT_DECREMENT_4      0x3

#define MPU6050_USERCTRL_DMP_EN_BIT             7
#define MPU6050_USERCTRL_FIFO_EN_BIT            6
#define MPU6050_USERCTRL_I2C_MST_EN_BIT         5
#define MPU6050_USERCTRL_I2C_IF_DIS_BIT         4
#define MPU6050_USERCTRL_DMP_RESET_BIT          3
#define MPU6050_USERCTRL_FIFO_RESET_BIT         2
#define MPU6050_USERCTRL_I2C_MST_RESET_BIT      1
#define MPU6050_USERCTRL_SIG_COND_RESET_BIT     0

#define MPU6050_PWR1_DEVICE_RESET_BIT   7
#define MPU6050_PWR1_SLEEP_BIT          6
#define MPU6050_PWR1_CYCLE_BIT          5
#define MPU6050_PWR1_TEMP_DIS_BIT       3
#define MPU6050_PWR1_CLKSEL_BIT         2
#define MPU6050_PWR1_CLKSEL_LENGTH      3

#define MPU6050_CLOCK_INTERNAL          0x00
#define MPU6050_CLOCK_PLL_XGYRO         0x01
#define MPU6050_CLOCK_PLL_YGYRO         0x02
#define MPU6050_CLOCK_PLL_ZGYRO         0x03
#define MPU6050_CLOCK_PLL_EXT32K        0x04
#define MPU6050_CLOCK_PLL_EXT19M        0x05
#define MPU6050_CLOCK_KEEP_RESET        0x07

#define MPU6050_PWR2_LP_WAKE_CTRL_BIT       7
#define MPU6050_PWR2_LP_WAKE_CTRL_LENGTH    2
#define MPU6050_PWR2_STBY_XA_BIT            5
#define MPU6050_PWR2_STBY_YA_BIT            4
#define MPU6050_PWR2_STBY_ZA_BIT            3
#define MPU6050_PWR2_STBY_XG_BIT            2
#define MPU6050_PWR2_STBY_YG_BIT            1
#define MPU6050_PWR2_STBY_ZG_BIT            0

#define MPU6050_WAKE_FREQ_1P25      0x0
#define MPU6050_WAKE_FREQ_2P5       0x1
#define MPU6050_WAKE_FREQ_5         0x2
#define MPU6050_WAKE_FREQ_10        0x3

#define MPU6050_BANKSEL_PRFTCH_EN_BIT       6
#define MPU6050_BANKSEL_CFG_USER_BANK_BIT   5
#define MPU6050_BANKSEL_MEM_SEL_BIT         4
#define MPU6050_BANKSEL_MEM_SEL_LENGTH      5

#define MPU6050_WHO_AM_I_BIT        6
#define MPU6050_WHO_AM_I_LENGTH     6

#define MPU6050_DMP_MEMORY_BANKS        8
#define MPU6050_DMP_MEMORY_BANK_SIZE    256
#define MPU6050_DMP_MEMORY_CHUNK_SIZE   16	 

#define MPU6050_DEG_PER_LSB_250  (float)((2 * 250.0) / 65536.0)
#define MPU6050_DEG_PER_LSB_500  (float)((2 * 500.0) / 65536.0)
#define MPU6050_DEG_PER_LSB_1000 (float)((2 * 1000.0) / 65536.0)
#define MPU6050_DEG_PER_LSB_2000 (float)((2 * 2000.0) / 65536.0)

#define MPU6050_DEG_PER_LSB		MPU6050_DEG_PER_LSB_500

#define MPU6050_G_PER_LSB_2      (float)((2 * 2) / 65536.0)
#define MPU6050_G_PER_LSB_4      (float)((2 * 4) / 65536.0)
#define MPU6050_G_PER_LSB_8      (float)((2 * 8) / 65536.0)
#define MPU6050_G_PER_LSB_16     (float)((2 * 16) / 65536.0)

#define MPU6050_G_PER_LSB		MPU6050_G_PER_LSB_2

#define MPU6050_ST_GYRO_LOW      10.0   // deg/s
#define MPU6050_ST_GYRO_HIGH     105.0  // deg/s
#define MPU6050_ST_ACCEL_LOW     0.300  // G
#define MPU6050_ST_ACCEL_HIGH    0.950  // G

#define MPU6050_TEMP_PER_LSB		340

void mpu6050_init(void);
//bool mpu6050_testConnection();

//// AUX_VDDIO register
//uint8_t mpu6050_getAuxVDDIOLevel();
//void mpu6050_setAuxVDDIOLevel(uint8_t level);

//// SMPLRT_DIV register
uint8_t mpu6050_getRate(void);
void mpu6050_setRate(uint8_t rate);

//// CONFIG register
//uint8_t mpu6050_getExternalFrameSync();
//void mpu6050_setExternalFrameSync(uint8_t sync);
uint8_t mpu6050_getDLPFMode(void);
void mpu6050_setDLPFMode(uint8_t bandwidth);

//// GYRO_CONFIG register
uint8_t mpu6050_getFullScaleGyroRange(void);
void mpu6050_setFullScaleGyroRange(uint8_t range);

//// ACCEL_CONFIG register
//bool mpu6050_getAccelXSelfTest();
//void mpu6050_setAccelXSelfTest(bool enabled);
//bool mpu6050_getAccelYSelfTest();
//void mpu6050_setAccelYSelfTest(bool enabled);
//bool mpu6050_getAccelZSelfTest();
//void mpu6050_setAccelZSelfTest(bool enabled);
//uint8_t mpu6050_getFullScaleAccelRange();
void mpu6050_setFullScaleAccelRange(uint8_t range);
//uint8_t mpu6050_getDHPFMode();
//void mpu6050_setDHPFMode(uint8_t mode);

//// FF_THR register
//uint8_t mpu6050_getFreefallDetectionThreshold();
//void mpu6050_setFreefallDetectionThreshold(uint8_t threshold);

//// FF_DUR register
//uint8_t mpu6050_getFreefallDetectionDuration();
//void mpu6050_setFreefallDetectionDuration(uint8_t duration);

//// MOT_THR register
//uint8_t mpu6050_getMotionDetectionThreshold();
//void mpu6050_setMotionDetectionThreshold(uint8_t threshold);

//// MOT_DUR register
//uint8_t mpu6050_getMotionDetectionDuration();
//void mpu6050_setMotionDetectionDuration(uint8_t duration);

//// ZRMOT_THR register
//uint8_t mpu6050_getZeroMotionDetectionThreshold();
//void mpu6050_setZeroMotionDetectionThreshold(uint8_t threshold);

//// ZRMOT_DUR register
//uint8_t mpu6050_getZeroMotionDetectionDuration();
//void mpu6050_setZeroMotionDetectionDuration(uint8_t duration);

//// FIFO_EN register
//bool mpu6050_getTempFIFOEnabled();
//void mpu6050_setTempFIFOEnabled(bool enabled);
//bool mpu6050_getXGyroFIFOEnabled();
//void mpu6050_setXGyroFIFOEnabled(bool enabled);
//bool mpu6050_getYGyroFIFOEnabled();
//void mpu6050_setYGyroFIFOEnabled(bool enabled);
//bool mpu6050_getZGyroFIFOEnabled();
//void mpu6050_setZGyroFIFOEnabled(bool enabled);
//bool mpu6050_getAccelFIFOEnabled();
//void mpu6050_setAccelFIFOEnabled(bool enabled);
//bool mpu6050_getSlave2FIFOEnabled();
//void mpu6050_setSlave2FIFOEnabled(bool enabled);
//bool mpu6050_getSlave1FIFOEnabled();
//void mpu6050_setSlave1FIFOEnabled(bool enabled);
//bool mpu6050_getSlave0FIFOEnabled();
//void mpu6050_setSlave0FIFOEnabled(bool enabled);

//// I2C_MST_CTRL register
//bool mpu6050_getMultiMasterEnabled();
//void mpu6050_setMultiMasterEnabled(bool enabled);
//bool mpu6050_getWaitForExternalSensorEnabled();
//void mpu6050_setWaitForExternalSensorEnabled(bool enabled);
//bool mpu6050_getSlave3FIFOEnabled();
//void mpu6050_setSlave3FIFOEnabled(bool enabled);
//bool mpu6050_getSlaveReadWriteTransitionEnabled();
//void mpu6050_setSlaveReadWriteTransitionEnabled(bool enabled);
//uint8_t mpu6050_getMasterClockSpeed();
//void mpu6050_setMasterClockSpeed(uint8_t speed);

//// I2C_SLV* registers (Slave 0-3)
//uint8_t mpu6050_getSlaveAddress(uint8_t num);
//void mpu6050_setSlaveAddress(uint8_t num, uint8_t address);
//uint8_t mpu6050_getSlaveRegister(uint8_t num);
//void mpu6050_setSlaveRegister(uint8_t num, uint8_t reg);
//bool mpu6050_getSlaveEnabled(uint8_t num);
//void mpu6050_setSlaveEnabled(uint8_t num, bool enabled);
//bool mpu6050_getSlaveWordByteSwap(uint8_t num);
//void mpu6050_setSlaveWordByteSwap(uint8_t num, bool enabled);
//bool mpu6050_getSlaveWriteMode(uint8_t num);
//void mpu6050_setSlaveWriteMode(uint8_t num, bool mode);
//bool mpu6050_getSlaveWordGroupOffset(uint8_t num);
//void mpu6050_setSlaveWordGroupOffset(uint8_t num, bool enabled);
//uint8_t mpu6050_getSlaveDataLength(uint8_t num);
//void mpu6050_setSlaveDataLength(uint8_t num, uint8_t length);

//// I2C_SLV* registers (Slave 4)
//uint8_t mpu6050_getSlave4Address();
//void mpu6050_setSlave4Address(uint8_t address);
//uint8_t mpu6050_getSlave4Register();
//void mpu6050_setSlave4Register(uint8_t reg);
//void mpu6050_setSlave4OutputByte(uint8_t data);
//bool mpu6050_getSlave4Enabled();
//void mpu6050_setSlave4Enabled(bool enabled);
//bool mpu6050_getSlave4InterruptEnabled();
//void mpu6050_setSlave4InterruptEnabled(bool enabled);
//bool mpu6050_getSlave4WriteMode();
//void mpu6050_setSlave4WriteMode(bool mode);
//uint8_t mpu6050_getSlave4MasterDelay();
//void mpu6050_setSlave4MasterDelay(uint8_t delay);
//uint8_t mpu6050_getSlate4InputByte();

//// I2C_MST_STATUS register
//bool mpu6050_getPassthroughStatus();
//bool mpu6050_getSlave4IsDone();
//bool mpu6050_getLostArbitration();
//bool mpu6050_getSlave4Nack();
//bool mpu6050_getSlave3Nack();
//bool mpu6050_getSlave2Nack();
//bool mpu6050_getSlave1Nack();
//bool mpu6050_getSlave0Nack();

//// INT_PIN_CFG register
//bool mpu6050_getInterruptMode();
//void mpu6050_setInterruptMode(bool mode);
//bool mpu6050_getInterruptDrive();
//void mpu6050_setInterruptDrive(bool drive);
//bool mpu6050_getInterruptLatch();
//void mpu6050_setInterruptLatch(bool latch);
//bool mpu6050_getInterruptLatchClear();
//void mpu6050_setInterruptLatchClear(bool clear);
//bool mpu6050_getFSyncInterruptLevel();
//void mpu6050_setFSyncInterruptLevel(bool level);
//bool mpu6050_getFSyncInterruptEnabled();
//void mpu6050_setFSyncInterruptEnabled(bool enabled);
u8 mpu6050_getI2CBypassEnabled(void);
void mpu6050_setI2CBypassEnabled(u8 enabled);
//bool mpu6050_getClockOutputEnabled();
//void mpu6050_setClockOutputEnabled(bool enabled);

//// INT_ENABLE register
uint8_t mpu6050_getIntEnabled(void);
void mpu6050SetIntEnabled(uint8_t enabled);
//bool mpu6050_getIntFreefallEnabled();
//void mpu6050_setIntFreefallEnabled(bool enabled);
//bool mpu6050_getIntMotionEnabled();
//void mpu6050_setIntMotionEnabled(bool enabled);
//bool mpu6050_getIntZeroMotionEnabled();
//void mpu6050_setIntZeroMotionEnabled(bool enabled);
//bool mpu6050_getIntFIFOBufferOverflowEnabled();
//void mpu6050_setIntFIFOBufferOverflowEnabled(bool enabled);
//bool mpu6050_getIntI2CMasterEnabled();
//void mpu6050_setIntI2CMasterEnabled(bool enabled);
//bool mpu6050_getIntDataReadyEnabled();
//void mpu6050_setIntDataReadyEnabled(bool enabled);

//// INT_STATUS register
//bool mpu6050_getIntFreefallStatus();
//bool mpu6050_getIntMotionStatus();
//bool mpu6050_getIntZeroMotionStatus();
//bool mpu6050_getIntFIFOBufferOverflowStatus();
//bool mpu6050_getIntI2CMasterStatus();
//bool mpu6050_getIntDataReadyStatus();

//// ACCEL_*OUT_* registers
void mpu6050_getMotion9(int16_t* ax, int16_t* ay, int16_t* az, int16_t* gx, int16_t* gy, int16_t* gz, int16_t* mx, int16_t* my, int16_t* mz);
void mpu6050_getMotion6(int16_t* ax, int16_t* ay, int16_t* az, int16_t* gx, int16_t* gy, int16_t* gz);
void mpu6050_getAcceleration(int16_t* x, int16_t* y, int16_t* z);
int16_t mpu6050_getAccelerationX(void);
int16_t mpu6050_getAccelerationY(void);
int16_t mpu6050_getAccelerationZ(void);

//// TEMP_OUT_* registers
int16_t mpu6050_getTemperature(void);

//// GYRO_*OUT_* registers
void mpu6050_getRotation(int16_t* x, int16_t* y, int16_t* z);
int16_t mpu6050_getRotationX(void);
int16_t mpu6050_getRotationY(void);
int16_t mpu6050_getRotationZ(void);

//// EXT_SENS_DATA_* registers
//uint8_t mpu6050_getExternalSensorByte(int position);
//uint16_t mpu6050_getExternalSensorWord(int position);
//uint32_t mpu6050_getExternalSensorDWord(int position);

//// MOT_DETECT_STATUS register
//bool mpu6050_getXNegMotionDetected();
//bool mpu6050_getXPosMotionDetected();
//bool mpu6050_getYNegMotionDetected();
//bool mpu6050_getYPosMotionDetected();
//bool mpu6050_getZNegMotionDetected();
//bool mpu6050_getZPosMotionDetected();
//bool mpu6050_getZeroMotionDetected();

//// I2C_SLV*_DO register
//void mpu6050_setSlaveOutputByte(uint8_t num, uint8_t data);

//// I2C_MST_DELAY_CTRL register
//bool mpu6050_getExternalShadowDelayEnabled();
//void mpu6050_setExternalShadowDelayEnabled(bool enabled);
//bool mpu6050_getSlaveDelayEnabled(uint8_t num);
//void mpu6050_setSlaveDelayEnabled(uint8_t num, bool enabled);

//// SIGNAL_PATH_RESET register
//void mpu6050_resetGyroscopePath();
//void mpu6050_resetAccelerometerPath();
//void mpu6050_resetTemperaturePath();

//// MOT_DETECT_CTRL register
//uint8_t mpu6050_getAccelerometerPowerOnDelay();
//void mpu6050_setAccelerometerPowerOnDelay(uint8_t delay);
//uint8_t mpu6050_getFreefallDetectionCounterDecrement();
//void mpu6050_setFreefallDetectionCounterDecrement(uint8_t decrement);
//uint8_t mpu6050_getMotionDetectionCounterDecrement();
//void mpu6050_setMotionDetectionCounterDecrement(uint8_t decrement);

//// USER_CTRL register
//bool mpu6050_getFIFOEnabled();
//void mpu6050_setFIFOEnabled(bool enabled);
bool mpu6050_getI2CMasterModeEnabled(void);
void mpu6050_setI2CMasterModeEnabled(bool enabled);
//void mpu6050_switchSPIEnabled(bool enabled);
//void mpu6050_resetFIFO();
//void mpu6050_resetI2CMaster();
//void mpu6050_resetSensors();

//// PWR_MGMT_1 register
bool mpu6050_reset(void);
//bool mpu6050_getSleepEnabled();
void mpu6050_setSleepEnabled(u8 enabled);
//bool mpu6050_getWakeCycleEnabled();
//void mpu6050_setWakeCycleEnabled(bool enabled);
u8 mpu6050_getTempSensorEnabled(void);
void mpu6050_setTempSensorEnabled(u8 enabled);
uint8_t mpu6050_getClockSource(void);
void mpu6050_setClockSource(uint8_t source);

//// PWR_MGMT_2 register
//uint8_t mpu6050_getWakeFrequency();
//void mpu6050_setWakeFrequency(uint8_t frequency);
//bool mpu6050_getStandbyXAccelEnabled();
//void mpu6050_setStandbyXAccelEnabled(bool enabled);
//bool mpu6050_getStandbyYAccelEnabled();
//void mpu6050_setStandbyYAccelEnabled(bool enabled);
//bool mpu6050_getStandbyZAccelEnabled();
//void mpu6050_setStandbyZAccelEnabled(bool enabled);
//bool mpu6050_getStandbyXGyroEnabled();
//void mpu6050_setStandbyXGyroEnabled(bool enabled);
//bool mpu6050_getStandbyYGyroEnabled();
//void mpu6050_setStandbyYGyroEnabled(bool enabled);
//bool mpu6050_getStandbyZGyroEnabled();
//void mpu6050_setStandbyZGyroEnabled(bool enabled);

//// FIFO_COUNT_* registers
//uint16_t mpu6050_getFIFOCount();

//// FIFO_R_W register
//uint8_t mpu6050_getFIFOByte();
//void mpu6050_setFIFOByte(uint8_t data);

//// WHO_AM_I register
uint8_t mpu6050_getDeviceID(void);
void mpu6050_setDeviceID(uint8_t id);

//// ======== UNDOCUMENTED/DMP REGISTERS/METHODS ========

//// XG_OFFS_TC register
//int8_t mpu6050_getXGyroOffset();
//void mpu6050_setXGyroOffset(int8_t offset);

//// YG_OFFS_TC register
//int8_t mpu6050_getYGyroOffset();
//void mpu6050_setYGyroOffset(int8_t offset);

//// ZG_OFFS_TC register
//int8_t mpu6050_getZGyroOffset();
//void mpu6050_setZGyroOffset(int8_t offset);

//// X_FINE_GAIN register
//int8_t mpu6050_getXFineGain();
//void mpu6050_setXFineGain(int8_t gain);

//// Y_FINE_GAIN register
//int8_t mpu6050_getYFineGain();
//void mpu6050_setYFineGain(int8_t gain);

//// Z_FINE_GAIN register
//int8_t mpu6050_getZFineGain();
//void mpu6050_setZFineGain(int8_t gain);

//// XA_OFFS_* registers
//int16_t mpu6050_getXAccelOffset();
//void mpu6050_setXAccelOffset(int16_t offset);

//// YA_OFFS_* register
//int16_t mpu6050_getYAccelOffset();
//void mpu6050_setYAccelOffset(int16_t offset);

//// ZA_OFFS_* register
//int16_t mpu6050_getZAccelOffset();
//void mpu6050_setZAccelOffset(int16_t offset);

//// XG_OFFS_USR* registers
//int16_t mpu6050_getXGyroOffsetUser();
//void mpu6050_setXGyroOffsetUser(int16_t offset);

//// YG_OFFS_USR* register
//int16_t mpu6050_getYGyroOffsetUser();
//void mpu6050_setYGyroOffsetUser(int16_t offset);

//// ZG_OFFS_USR* register
//int16_t mpu6050_getZGyroOffsetUser();
//void mpu6050_setZGyroOffsetUser(int16_t offset);

//// INT_ENABLE register (DMP functions)
//bool mpu6050_getIntPLLReadyEnabled();
//void mpu6050_setIntPLLReadyEnabled(bool enabled);
//bool mpu6050_getIntDMPEnabled();
//void mpu6050_setIntDMPEnabled(bool enabled);

//// DMP_INT_STATUS
//bool mpu6050_getDMPInt5Status();
//bool mpu6050_getDMPInt4Status();
//bool mpu6050_getDMPInt3Status();
//bool mpu6050_getDMPInt2Status();
//bool mpu6050_getDMPInt1Status();
//bool mpu6050_getDMPInt0Status();

//// INT_STATUS register (DMP functions)
//bool mpu6050_getIntPLLReadyStatus();
//bool mpu6050_getIntDMPStatus();

//// USER_CTRL register (DMP functions)
//bool mpu6050_getDMPEnabled();
//void mpu6050_setDMPEnabled(bool enabled);
//void mpu6050_resetDMP();

//// BANK_SEL register
//void mpu6050_setMemoryBank(uint8_t bank, bool prefetchEnabled, bool userBank);

//// MEM_START_ADDR register
//void mpu6050_setMemoryStartAddress(uint8_t address);

//// MEM_R_W register
//uint8_t mpu6050_readMemoryByte();
//void mpu6050_writeMemoryByte(uint8_t data);
//void mpu6050_readMemoryBlock(uint8_t *data, uint16_t dataSize, uint8_t bank, uint8_t address);
//bool mpu6050_writeMemoryBlock(uint8_t *data, uint16_t dataSize, uint8_t bank, uint8_t address, bool verify, bool useProgMem);
//bool mpu6050_writeProgMemoryBlock(uint8_t *data, uint16_t dataSize, uint8_t bank, uint8_t address, bool verify);

//// DMP_CFG_1 register
//uint8_t mpu6050_getDMPConfig1();
//void mpu6050_setDMPConfig1(uint8_t config);

//// DMP_CFG_2 register
//uint8_t mpu6050_getDMPConfig2();
//void mpu6050_setDMPConfig2(uint8_t config);

bool mpu6050_check(void);
uint8_t mpu6050_getAngle(float x, float y, float z, uint8_t dir);

//用于MPU605功能调试
#define MPU6050_PDEBUG
#ifdef MPU6050_PDEBUG
#define MPU6050_DEBUG(string,args...)	printf("[MPU6050_DEBUG] ");	\
									printf(string, ##args)
#else
#define MPU6050_DEBUG(string,args...)
#endif

//用于MPU6050数据调试
//#define MPU6050_DDEBUG
#ifdef MPU6050_DDEBUG
#define MPU6050_DPDEBUG(string,args...)	printf("[MPU6050_DATA] ");	\
									printf(string, ##args)
#else
#define MPU6050_DPDEBUG(string,args...)
#endif
									
#define SW_I2C

#if defined(SW_I2C)
	#include "sw_i2c.h"
#else 
	#include "hw_i2c.h"
#endif

#ifdef __cplusplus
}
#endif

#endif /* __MPU6050_H */
