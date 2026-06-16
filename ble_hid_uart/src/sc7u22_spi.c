/**
 * @file    sc7u22_spi.c
 * @brief   SC7U22 6-axis IMU SPI driver implementation for PAN1080 BLE SoC.
 *
 * Uses PAN1080 hardware SPI0 with manual GPIO CS control.
 * SPI Mode 0 (CPOL=0, CPHA=0), MSB first, 8-bit data frame.
 *
 * Pin mapping:
 *   CS   - P02 (GPIO output, manual control)
 *   SCK  - P03 (SPI0_CLK)
 *   MOSI - P30 (SPI0_MOSI)
 *   MISO - P31 (SPI0_MISO)
 */

#include "sc7u22_spi.h"
#include <soc.h>
#include <kernel.h>
/*===========================================================================
 * Global variables
 *===========================================================================*/
tsIMUData IMUData;
signed short reg_ax, reg_ay, reg_az;
signed short reg_gx, reg_gy, reg_gz;

static uint8_t sensor_enable_flag;

/*===========================================================================
 * Internal helpers
 *===========================================================================*/

static inline void cs_low(void)  { P0->DOUT &= ~(1 << 2); }   /* P02 low */
static inline void cs_high(void) { P0->DOUT |=  (1 << 2); }   /* P02 high */

static inline void spi_wait_tx_done(void)
{
    while (SPI_IsBusy(SPI0));
}

/**
 * @brief  Send one byte via SPI and return the received byte.
 * @param  tx_data  Byte to send.
 * @return Byte received during transmission.
 */
static uint8_t spi_transfer_byte(uint8_t tx_data)
{
    while (SPI_IsTxFifoFull(SPI0));
    SPI_SendData(SPI0, tx_data);
    spi_wait_tx_done();
    return (uint8_t)SPI_ReceiveData(SPI0);
}

/*===========================================================================
 * Public API
 *===========================================================================*/

/**
 * @brief  Initialize SPI0 hardware and pinmux for SC7U22 communication.
 */
void SC7U22_SPI_Init(void)
{
    SPI_InitTypeDef spi_cfg;

    /* 1. Enable SPI0 peripheral clock */
    CLK_APB1PeriphClockCmd(CLK_APB1Periph_SPI0, ENABLE);

    /* 2. Configure pinmux
     *    P02 -> GPIO (manual CS, NOT SPI0_CS function)
     *    P03 -> SPI0_CLK
     *    P30 -> SPI0_MOSI
     *    P31 -> SPI0_MISO
     */
    SYS_SET_MFP(P0, 2, GPIO);          /* CS - manual GPIO control */
    SYS_SET_MFP(P0, 3, SPI0_CLK);
    SYS_SET_MFP(P3, 0, SPI0_MOSI);
    SYS_SET_MFP(P3, 1, SPI0_MISO);

    /* 3. Enable digital path for input pins */
    GPIO_EnableDigitalPath(P0, (1 << 3));   /* SCK - enable digital input */
    GPIO_EnableDigitalPath(P3, (1 << 1));   /* MISO - enable digital input */

    /* 4. Configure CS pin (P02) as output, default high (deselected) */
    GPIO_EnableDigitalPath(P0, (1 << 2));
    P0->MODE = (P0->MODE & ~(0x0F << 8)) | (0x01 << 8);  /* P02: output mode */
    cs_high();

    /* 5. Configure SPI0 as master
     *    Mode 0: CPOL=Low (idle low), CPHA=FirstEdge (sample on rising edge)
     *    8-bit data frame, MSB first (default), Motorola format
     *    Baud rate divisor = 32 (~500kHz @ 16MHz APB clock)
     */
    spi_cfg.SPI_role = SPI_RoleMaster;
    spi_cfg.SPI_format = SPI_FormatMotorola;
    spi_cfg.SPI_dataFrameSize = SPI_DataFrame_8b;
    spi_cfg.SPI_CPOL = SPI_ClockPolarityLow;
    spi_cfg.SPI_CPHA = SPI_ClockPhaseFirstEdge;
    spi_cfg.SPI_baudRateDiv = SPI_BaudRateDiv_32;

    SPI_Init(SPI0, &spi_cfg);
    SPI_EnableSpi(SPI0);
}

/**
 * @brief  Read a single register from SC7U22 via SPI.
 *
 * Protocol:
 *   1. CS low
 *   2. Send (reg | 0x80)  -- bit 7 = 1 means READ
 *   3. Discard first received byte (garbage)
 *   4. Send dummy byte to clock in the actual data
 *   5. Read received data byte
 *   6. CS high
 */
uint8_t SC7U22_SPI_ReadReg(uint8_t reg)
{
    uint8_t result;

    cs_low();
    spi_transfer_byte(reg | 0x80);      /* Send register address with read flag */
    result = spi_transfer_byte(0x00);    /* Send dummy, receive data */
    cs_high();

    return result;
}

/**
 * @brief  Write a value to a single SC7U22 register via SPI.
 *
 * Protocol:
 *   1. CS low
 *   2. Send reg  -- bit 7 = 0 means WRITE
 *   3. Discard first received byte
 *   4. Send value byte
 *   5. Discard received byte
 *   6. CS high
 */
void SC7U22_SPI_WriteReg(uint8_t reg, uint8_t value)
{
    cs_low();
    spi_transfer_byte(reg);             /* Send register address (write) */
    spi_transfer_byte(value);           /* Send data */
    cs_high();
}

/*===========================================================================
 * Sensor range configuration tables
 *===========================================================================*/

static const config_table_t acc_range_table[] = {
    {2,  SC7U22_ACC_2G},
    {4,  SC7U22_ACC_4G},
    {8,  SC7U22_ACC_8G},
    {16, SC7U22_ACC_16G}
};

static const config_table_t gyro_range_table[] = {
    {125,  SC7U22_GYRO_125DPS},
    {250,  SC7U22_GYRO_250DPS},
    {500,  SC7U22_GYRO_500DPS},
    {1000, SC7U22_GYRO_1000DPS},
    {2000, SC7U22_GYRO_2000DPS}
};

#define ARRAY_SIZE(arr) (sizeof(arr) / sizeof(arr[0]))

void SC7U22_SetAccRange(uint32_t range_g)
{
    uint8_t reg_val;
    int i;

    for (i = 0; i < (int)ARRAY_SIZE(acc_range_table); i++) {
        if (range_g == acc_range_table[i].req_value) {
            reg_val = SC7U22_SPI_ReadReg(SC7U22_ACC_RANGE);
            reg_val &= ~SC7U22_ACC_RANGE_MASK;
            reg_val |= acc_range_table[i].reg_value;
            SC7U22_SPI_WriteReg(SC7U22_ACC_RANGE, reg_val);
            k_msleep(2);
            SC7U22_SPI_WriteReg(SC7U22_ACC_RANGE, reg_val);
            break;
        }
    }
}

void SC7U22_SetGyroRange(uint32_t range_dps)
{
    uint8_t reg_val;
    int i;

    for (i = 0; i < (int)ARRAY_SIZE(gyro_range_table); i++) {
        if (range_dps == gyro_range_table[i].req_value) {
            reg_val = SC7U22_SPI_ReadReg(SC7U22_GYRO_RANGE);
            reg_val &= ~SC7U22_GYRO_RANGE_MASK;
            reg_val |= gyro_range_table[i].reg_value;
            SC7U22_SPI_WriteReg(SC7U22_GYRO_RANGE, reg_val);
            k_msleep(2);
            SC7U22_SPI_WriteReg(SC7U22_GYRO_RANGE, reg_val);
            break;
        }
    }
}

/**
 * @brief  Enable or disable an individual sensor (accel, gyro, or temp).
 */
void SC7U22_SensorEnable(sensor_type_t type, uint8_t enable)
{
    uint8_t reg_val;

    reg_val = SC7U22_SPI_ReadReg(SC7U22_POWER_CTRL);

    if (enable) {
        switch (type) {
        case SENSOR_ACC:
            reg_val |= SC7U22_ACC_ENABLE;
            sensor_enable_flag |= SC7U22_ACC_ENABLE;
            break;
        case SENSOR_GYRO:
            reg_val |= SC7U22_GYRO_ENABLE;
            sensor_enable_flag |= SC7U22_GYRO_ENABLE;
            break;
        case SENSOR_TEMP:
            reg_val |= SC7U22_TEMP_ENABLE;
            sensor_enable_flag |= SC7U22_TEMP_ENABLE;
            break;
        }
    } else {
        switch (type) {
        case SENSOR_ACC:
            reg_val &= ~SC7U22_ACC_ENABLE;
            sensor_enable_flag &= ~SC7U22_ACC_ENABLE;
            break;
        case SENSOR_GYRO:
            reg_val &= ~SC7U22_GYRO_ENABLE;
            sensor_enable_flag &= ~SC7U22_GYRO_ENABLE;
            break;
        case SENSOR_TEMP:
            reg_val &= ~SC7U22_TEMP_ENABLE;
            sensor_enable_flag &= ~SC7U22_TEMP_ENABLE;
            break;
        }
    }

    SC7U22_SPI_WriteReg(SC7U22_POWER_CTRL, reg_val);
}

/*===========================================================================
 * Sensor initialization
 *===========================================================================*/

static void sc7u22_setup(void)
{
    /* Soft reset */
    SC7U22_SPI_WriteReg(SC7U22_SOFT_RESET, 0xA5);
    SC7U22_SPI_WriteReg(SC7U22_SOFT_RESET, 0xA5);
    k_msleep(2);

    /* Configure ranges */
    SC7U22_SetAccRange(8);         /* +/-8g */
    SC7U22_SetGyroRange(2000);     /* +/-2000 dps */

    /* Enable sensors */
    SC7U22_SensorEnable(SENSOR_ACC, 1);
    SC7U22_SensorEnable(SENSOR_GYRO, 1);
}

uint8_t SC7U22_Init(void)
{
    uint8_t chip_id = 0;
    int retry = 0;

    while ((chip_id != SC7U22_DEVICE_ID) && (retry < 3)) {
        chip_id = SC7U22_SPI_ReadReg(SC7U22_WHO_AM_I);
        retry++;
        if (chip_id == SC7U22_DEVICE_ID) {
            printk("SC7U22 init success, Chip ID = 0x%02x\n", chip_id);
            break;
        }
    }

    if (chip_id != SC7U22_DEVICE_ID) {
        printk("SC7U22 init fail, Chip ID = 0x%02x\n", chip_id);
        return 1;
    }

    sc7u22_setup();
    return 0;
}

/*===========================================================================
 * IMU data reading
 *===========================================================================*/

/**
 * @brief  Read a 16-bit signed IMU value from two consecutive registers (high, low).
 */
static int16_t read_sensor_word(uint8_t reg_high, uint8_t reg_low)
{
    uint8_t h = SC7U22_SPI_ReadReg(reg_high);
    uint8_t l = SC7U22_SPI_ReadReg(reg_low);
    return (int16_t)(((uint16_t)h << 8) | l);
}

/**
 * @brief  Read all 6-axis IMU raw data (accel + gyro), convert to physical units.
 *
 * Accelerometer sensitivity: 4096 LSB/g (@ 8g range, 16-bit)
 *   Value in m/s^2 = raw * 9.8 / 4096
 *
 * Gyroscope sensitivity: 16.4 LSB/(deg/s) (@ 2000dps range, 16-bit)
 *   Value in rad/s = raw / 57.3 / 16.4
 */

//读取数据6轴数据到IMUData全局变量
void SC7U22_ReadIMU(void)
{
    reg_ax = read_sensor_word(SC7U22_ACC_HX, SC7U22_ACC_LX);
    reg_ay = read_sensor_word(SC7U22_ACC_HY, SC7U22_ACC_LY);
    reg_az = read_sensor_word(SC7U22_ACC_HZ, SC7U22_ACC_LZ);
    reg_gx = read_sensor_word(SC7U22_GYRO_HX, SC7U22_GYRO_LX);
    reg_gy = read_sensor_word(SC7U22_GYRO_HY, SC7U22_GYRO_LY);
    reg_gz = read_sensor_word(SC7U22_GYRO_HZ, SC7U22_GYRO_LZ);

    IMUData.Accelerate.X = (float)reg_ax * 9.8f / 4096.0f - IMUData.BiasAccelerate.X;
    IMUData.Accelerate.Y = (float)reg_ay * 9.8f / 4096.0f - IMUData.BiasAccelerate.Y;
    IMUData.Accelerate.Z = (float)reg_az * 9.8f / 4096.0f - IMUData.BiasAccelerate.Z;
    IMUData.Angular.X    = (float)reg_gx / 57.3f / 16.4f - IMUData.BiasAngular.X;
    IMUData.Angular.Y    = (float)reg_gy / 57.3f / 16.4f - IMUData.BiasAngular.Y;
    IMUData.Angular.Z    = (float)reg_gz / 57.3f / 16.4f - IMUData.BiasAngular.Z;

    printk("%f, %f, %f, %f, %f, %f\n",
           (double)IMUData.Accelerate.X, (double)IMUData.Accelerate.Y,
           (double)IMUData.Accelerate.Z,
           (double)IMUData.Angular.X, (double)IMUData.Angular.Y,
           (double)IMUData.Angular.Z);
}

/*===========================================================================
 * Bias calculation
 *===========================================================================*/

/**
 * @brief  Calculate bias by averaging 140 samples (after discarding first 60)
 *         from 200 total readings. Sensor must be stationary.
 */
void SC7U22_BiasCalculate(void)
{
    uint16_t count = 200;
    float sum_bias_acc_x  = 0.0f;
    float sum_bias_acc_y  = 0.0f;
    float sum_bias_acc_z  = 0.0f;
    float sum_bias_gyro_x = 0.0f;
    float sum_bias_gyro_y = 0.0f;
    float sum_bias_gyro_z = 0.0f;

    while (count > 0) {
        int16_t ax = read_sensor_word(SC7U22_ACC_HX, SC7U22_ACC_LX);
        int16_t ay = read_sensor_word(SC7U22_ACC_HY, SC7U22_ACC_LY);
        int16_t az = read_sensor_word(SC7U22_ACC_HZ, SC7U22_ACC_LZ);
        int16_t gx = read_sensor_word(SC7U22_GYRO_HX, SC7U22_GYRO_LX);
        int16_t gy = read_sensor_word(SC7U22_GYRO_HY, SC7U22_GYRO_LY);
        int16_t gz = read_sensor_word(SC7U22_GYRO_HZ, SC7U22_GYRO_LZ);

        if (count <= 140) {
            sum_bias_acc_x  += (float)ax * 9.8f / 4096.0f;
            sum_bias_acc_y  += (float)ay * 9.8f / 4096.0f;
            sum_bias_acc_z  += (float)az * 9.8f / 4096.0f;
            sum_bias_gyro_x += (float)gx / 57.3f / 16.4f;
            sum_bias_gyro_y += (float)gy / 57.3f / 16.4f;
            sum_bias_gyro_z += (float)gz / 57.3f / 16.4f;
        }
        count--;
    }

    IMUData.BiasAccelerate.X = sum_bias_acc_x / 140.0f;
    IMUData.BiasAccelerate.Y = sum_bias_acc_y / 140.0f;
    IMUData.BiasAccelerate.Z = sum_bias_acc_z / 140.0f - 9.8f;
    IMUData.BiasAngular.X    = sum_bias_gyro_x / 140.0f;
    IMUData.BiasAngular.Y    = sum_bias_gyro_y / 140.0f;
    IMUData.BiasAngular.Z    = sum_bias_gyro_z / 140.0f;

    printk("Bias: %f, %f, %f, %f, %f, %f\n",
           (double)IMUData.BiasAccelerate.X, (double)IMUData.BiasAccelerate.Y,
           (double)IMUData.BiasAccelerate.Z,
           (double)IMUData.BiasAngular.X, (double)IMUData.BiasAngular.Y,
           (double)IMUData.BiasAngular.Z);
}
