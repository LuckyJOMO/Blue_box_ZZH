/**
 * @file    sc7u22_spi.h
 * @brief   SC7U22 6-axis IMU SPI driver for PAN1080 BLE SoC
 *
 * Pin mapping:
 *   CS   - P02 (GPIO manual control)
 *   SCK  - P03 (SPI0_CLK)
 *   MOSI - P30 (SPI0_MOSI)
 *   MISO - P31 (SPI0_MISO)
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
 * Pin definitions
 *===========================================================================*/
#define SC7U22_CS_PORT          P0
#define SC7U22_CS_PIN           2           /* P02 */
#define SC7U22_SCK_PORT         P0
#define SC7U22_SCK_PIN          3           /* P03 */
#define SC7U22_MOSI_PORT        P3
#define SC7U22_MOSI_PIN         0           /* P30 */
#define SC7U22_MISO_PORT        P3
#define SC7U22_MISO_PIN         1           /* P31 */

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

/* FIFO registers */
#define SC7U22_FIFO_CONFIG0     0x1C
#define SC7U22_FIFO_CONFIG1     0x1D
#define SC7U22_FIFO_CONFIG2     0x1E
#define SC7U22_FIFO_STAT0       0x1F
#define SC7U22_FIFO_STAT1       0x20
#define SC7U22_FIFO_DATA        0x21

/* Temperature registers */
#define SC7U22_TEMP_H           0x22
#define SC7U22_TEMP_L           0x23

/* Sensor configuration */
#define SC7U22_ACC_CONFIG       0x40
#define SC7U22_ACC_RANGE        0x41
#define SC7U22_GYRO_CONFIG      0x42
#define SC7U22_GYRO_RANGE       0x43

#define SC7U22_SOFT_RESET       0x4A
#define SC7U22_POWER_CTRL       0x7D
#define SC7U22_SEG_SEL          0x7F
#define SC7U22_SEG_EXIT         0x00

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
 *=========================================================================== */

/**
 * @brief  Initialize SPI hardware and configure pinmux for SC7U22.
 * @note   Call once at system startup before any SC7U22 access.
 */
void SC7U22_SPI_Init(void);

/**
 * @brief  Initialize SC7U22 sensor (check ID, soft-reset, config range, enable sensors).
 * @return 0 on success, 1 on failure (chip ID mismatch).
 */
uint8_t SC7U22_Init(void);

/**
 * @brief  Read all 6-axis IMU raw data, convert to physical units, store in IMUData.
 * @note   Updates reg_ax/y/z, reg_gx/y/z and IMUData globals.
 */
void SC7U22_ReadIMU(void);

/**
 * @brief  Calculate gyroscope and accelerometer bias (200 samples, last 140 averaged).
 * @note   Sensor must be stationary during this call.
 */
void SC7U22_BiasCalculate(void);

/**
 * @brief  Read a single register via SPI.
 * @param  reg  Register address (bit 7 auto-set for read).
 * @return Register value.
 */
uint8_t SC7U22_SPI_ReadReg(uint8_t reg);

/**
 * @brief  Write a single register via SPI.
 * @param  reg    Register address.
 * @param  value  Value to write.
 */
void SC7U22_SPI_WriteReg(uint8_t reg, uint8_t value);

/**
 * @brief  Set accelerometer range.
 * @param  range_g  Full-scale range in g (2, 4, 8, 16).
 */
void SC7U22_SetAccRange(uint32_t range_g);

/**
 * @brief  Set gyroscope range.
 * @param  range_dps  Full-scale range in dps (125, 250, 500, 1000, 2000).
 */
void SC7U22_SetGyroRange(uint32_t range_dps);

/**
 * @brief  Enable or disable a specific sensor.
 * @param  type    Sensor type (SENSOR_ACC, SENSOR_GYRO, SENSOR_TEMP).
 * @param  enable  1 = enable, 0 = disable.
 */
void SC7U22_SensorEnable(sensor_type_t type, uint8_t enable);

#ifdef __cplusplus
}
#endif

#endif /* _SC7U22_SPI_H_ */
