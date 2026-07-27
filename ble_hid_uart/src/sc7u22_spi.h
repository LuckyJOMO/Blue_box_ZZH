/**
 * @file    sc7u22_spi.h
 * @brief   SC7U22 6-axis IMU SPI driver for PAN1080 BLE SoC
 *
 * Pin mapping:
 *   CS   - P02 (GPIO manual control)
 *   SCK  - P03 (SPI0_CLK)
 *   MOSI - P30 (SPI0_MOSI)
 *   MISO - P31 (SPI0_MISO)
 *   INT1 - P05 (data ready interrupt)
 *   INT2 - P24 (reserved)
 *   PWM  - P04 (PWM1_CH0)
 *
 * SPI Mode 0 (CPOL=0, CPHA=0), MSB first, 8-bit frame
 */

#ifndef _SC7U22_SPI_H_
#define _SC7U22_SPI_H_

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/*===========================================================================
 * Pin definitions  (port, pin, function are hardcoded because SYS_SET_MFP
 * requires literal arguments for its ## token-pasting)
 *
 * Same style as icm42670_spi.h
 *===========================================================================*/

/* CS - P02, manual GPIO */
#define SC7U22_CS_INIT()        SYS_SET_MFP(P0, 2, GPIO)
#define SC7U22_CS_GPIO_INIT()   GPIO_SetMode(P0, BIT2, GPIO_MODE_OUTPUT)
#define SC7U22_CS_HIGH()        (P02 = 1)
#define SC7U22_CS_LOW()         (P02 = 0)

/* SCK - P03, SPI0_CLK */
#define SC7U22_SCK_INIT()       SYS_SET_MFP(P0, 3, SPI0_CLK)
#define SC7U22_SCK_DIGITAL()    GPIO_EnableDigitalPath(P0, BIT3)

/* MOSI - P30, SPI0_MOSI */
#define SC7U22_MOSI_INIT()      SYS_SET_MFP(P3, 0, SPI0_MOSI)
#define SC7U22_MOSI_DIGITAL()   GPIO_EnableDigitalPath(P3, BIT0)

/* MISO - P31, SPI0_MISO */
#define SC7U22_MISO_INIT()      SYS_SET_MFP(P3, 1, SPI0_MISO)
#define SC7U22_MISO_DIGITAL()   GPIO_EnableDigitalPath(P3, BIT1)

/* INT1 - P05, GPIO input */
#define SC7U22_INT1_INIT()      SYS_SET_MFP(P0, 5, GPIO)
#define SC7U22_INT1_DIGITAL()   GPIO_EnableDigitalPath(P0, BIT5)

/* INT2 - P24, GPIO input */
#define SC7U22_INT2_INIT()      SYS_SET_MFP(P2, 4, GPIO)
#define SC7U22_INT2_DIGITAL()   GPIO_EnableDigitalPath(P2, BIT4)

/*===========================================================================
 * SC7U22 Register Map
 *===========================================================================*/
#define SC7U22_DEVICE_ID        0x6A

/* Common registers */
#define SC7U22_WHO_AM_I         0x01
#define SC7U22_COM_CONFIG       0x04
#define SC7U22_INT1_OUT1        0x05
#define SC7U22_INT1_OUT2        0x06
#define SC7U22_INT2_OUT1        0x07
#define SC7U22_INT2_OUT2        0x08
#define SC7U22_INT1_STAT        0x09
#define SC7U22_INT2_STAT        0x0A
#define SC7U22_DATA_STAT        0x0B

/* Accelerometer data registers */
#define SC7U22_ACC_HX           0x0C
#define SC7U22_ACC_LX           0x0D
#define SC7U22_ACC_HY           0x0E
#define SC7U22_ACC_LY           0x0F
#define SC7U22_ACC_HZ           0x10
#define SC7U22_ACC_LZ           0x11

/* Gyroscope data registers */
#define SC7U22_GYRO_HX          0x12
#define SC7U22_GYRO_LX          0x13
#define SC7U22_GYRO_HY          0x14
#define SC7U22_GYRO_LY          0x15
#define SC7U22_GYRO_HZ          0x16
#define SC7U22_GYRO_LZ          0x17

/* Sensor configuration */
#define SC7U22_ACC_CONFIG       0x40
#define SC7U22_ACC_RANGE        0x41
#define SC7U22_GYRO_CONFIG      0x42
#define SC7U22_GYRO_RANGE       0x43

#define SC7U22_SOFT_RESET       0x4A
#define SC7U22_POWER_CTRL       0x7D

/*===========================================================================
 * Register value definitions
 *===========================================================================*/
/* Power control bits */
#define SC7U22_ACC_ENABLE       0x04
#define SC7U22_GYRO_ENABLE      0x02
#define SC7U22_TEMP_ENABLE      0x08

/* Accelerometer range */
#define SC7U22_ACC_RANGE_MASK   0x03
#define SC7U22_ACC_2G           0x00
#define SC7U22_ACC_4G           0x01
#define SC7U22_ACC_8G           0x02
#define SC7U22_ACC_16G          0x03

/* Gyroscope range */
#define SC7U22_GYRO_RANGE_MASK  0x07
#define SC7U22_GYRO_125DPS      0x04
#define SC7U22_GYRO_250DPS      0x03
#define SC7U22_GYRO_500DPS      0x02
#define SC7U22_GYRO_1000DPS     0x01
#define SC7U22_GYRO_2000DPS     0x00

/* Accelerometer ODR */
#define SC7U22_ACC_ODR_MASK     0x0F
#define SC7U22_ACC_100HZ        0x08
#define SC7U22_ACC_200HZ        0x09
#define SC7U22_ACC_400HZ        0x0A
#define SC7U22_ACC_800HZ        0x0B

/* Gyroscope ODR */
#define SC7U22_GYRO_ODR_MASK    0x0F
#define SC7U22_GYRO_100HZ       0x08
#define SC7U22_GYRO_200HZ       0x09
#define SC7U22_GYRO_400HZ       0x0A
#define SC7U22_GYRO_800HZ       0x0B

/*===========================================================================
 * Sensitivity constants
 *===========================================================================*/
/* Accelerometer: +/-8g -> 4096 LSB/g */
#define SC7U22_ACC_SENSITIVITY  4096.0f
/* Gyroscope: +/-2000dps -> 16.4 LSB/dps */
#define SC7U22_GYRO_SENSITIVITY 16.4f

/*===========================================================================
 * Data structures
 *===========================================================================*/
typedef struct {
	float X, Y, Z;
} Vector3f;

typedef struct {
	Vector3f Accelerate;
	Vector3f Angular;
	Vector3f BiasAccelerate;
	Vector3f BiasAngular;
} tsIMUData;

typedef enum {
	SENSOR_ACC,
	SENSOR_GYRO,
	SENSOR_TEMP
} sensor_type_t;

typedef struct {
	uint32_t req_value;
	uint8_t  reg_value;
} config_table_t;

/*===========================================================================
 * Global variables (defined in sc7u22_spi.c)
 *===========================================================================*/
extern tsIMUData IMUData;
extern signed short reg_ax, reg_ay, reg_az;
extern signed short reg_gx, reg_gy, reg_gz;

/*===========================================================================
 * API functions
 *===========================================================================*/

/* ---- SPI hardware / pinmux ---- */
void SC7U22_SPI_Init(void);

/* ---- Sensor init & config ---- */
uint8_t SC7U22_Init(void);
void SC7U22_SetAccRange(uint32_t range_g);
void SC7U22_SetGyroRange(uint32_t range_dps);
void SC7U22_SensorEnable(sensor_type_t type, uint8_t enable);

/* ---- Data reading ---- */
void SC7U22_ReadIMU(void);

/* ---- Bias ---- */
void SC7U22_BiasCalculate(void);

/* ---- Low-level SPI R/W ---- */
uint8_t SC7U22_SPI_ReadReg(uint8_t reg);
void     SC7U22_SPI_WriteReg(uint8_t reg, uint8_t value);

/* ---- PWM output (P04 = PWM1_CH0) ---- */
void SC7U22_PWM_Init(uint32_t freq_hz, uint32_t duty_pct);
void SC7U22_PWM_SetDuty(uint32_t duty_pct);

/* ---- INT pin init (P05, P24) ---- */
void SC7U22_INT_Init(void);

#ifdef __cplusplus
}
#endif

#endif /* _SC7U22_SPI_H_ */
