/*
 * sc7u22_spi.c - SC7U22 SPI driver for PAN1080 BLE SoC
 *
 * SPI communication style follows icm42670_spi.c
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
 * Internal helpers (same style as icm42670_spi.c)
 *===========================================================================*/

static void cs_low(void)  { SC7U22_CS_LOW(); }
static void cs_high(void) { SC7U22_CS_HIGH(); }

/*===========================================================================
 * SPI peripheral init (same style as icm42670_spi.c)
 *===========================================================================*/

void SC7U22_SPI_Init(void)
{
	SPI_InitTypeDef spi_cfg = {0};

	CLK_APB1PeriphClockCmd(CLK_APB1Periph_SPI0, ENABLE);

	/* CS pin init */
	SC7U22_CS_INIT();
	SC7U22_CS_GPIO_INIT();
	SC7U22_CS_HIGH();

	/* SCK, MOSI, MISO pin init */
	SC7U22_SCK_INIT();
	SC7U22_MOSI_INIT();
	SC7U22_MISO_INIT();

	SC7U22_SCK_DIGITAL();
	SC7U22_MOSI_DIGITAL();
	SC7U22_MISO_DIGITAL();

	spi_cfg.SPI_role          = SPI_RoleMaster;
	spi_cfg.SPI_format        = SPI_FormatMotorola;
	spi_cfg.SPI_dataFrameSize = SPI_DataFrame_8b;
	spi_cfg.SPI_CPOL          = SPI_ClockPolarityLow;
	spi_cfg.SPI_CPHA          = SPI_ClockPhaseFirstEdge;
	spi_cfg.SPI_baudRateDiv   = SPI_BaudRateDiv_32;

	SPI_DisableSpi(SPI0);
	SPI_Init(SPI0, &spi_cfg);
	SPI_TxLsbEnable(SPI0, false);
	SPI_RxLsbEnable(SPI0, false);
	SPI_EnableSpi(SPI0);
}

/*===========================================================================
 * Pure SPI read / write (same style as icm42670_spi.c)
 *===========================================================================*/

uint8_t SC7U22_SPI_ReadReg(uint8_t reg)
{
	uint8_t status, value;

	cs_low();

	/* Send register address (MSB=1 for read) */
	SPI_SendData(SPI0, reg | 0x80);
	while (SPI_IsBusy(SPI0)) {}
	status = SPI_ReceiveData(SPI0);

	/* Send dummy byte, receive register value */
	SPI_SendData(SPI0, 0x00);
	while (SPI_IsBusy(SPI0)) {}
	value = SPI_ReceiveData(SPI0);

	cs_high();
	return value;
}

void SC7U22_SPI_WriteReg(uint8_t reg, uint8_t value)
{
	cs_low();

	/* Send register address (MSB=0 for write) */
	SPI_SendData(SPI0, reg);
	while (SPI_IsBusy(SPI0)) {}
	SPI_ReceiveData(SPI0);   /* discard status */

	/* Send register value */
	SPI_SendData(SPI0, value);
	while (SPI_IsBusy(SPI0)) {}
	SPI_ReceiveData(SPI0);   /* discard response */

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
	k_msleep(1);
	SC7U22_SPI_WriteReg(SC7U22_POWER_CTRL, reg_val); /* 参考工程: 必须回写两次才生效 */
}

/*===========================================================================
 * Sensor initialization
 *===========================================================================*/

static void sc7u22_setup(void)
{
	uint8_t pwr_val;

	/* Soft reset */
	SC7U22_SPI_WriteReg(SC7U22_SOFT_RESET, 0xA5);
	SC7U22_SPI_WriteReg(SC7U22_SOFT_RESET, 0xA5);
	k_msleep(2);

	/* 设置陀螺仪 ODR (Gyro_config必须设, 否则陀螺仪可能不输出数据) */
	SC7U22_SPI_WriteReg(SC7U22_GYRO_CONFIG, 0x86); /* 50Hz ODR + HP mode (参考Silan SDK) */

	/* Configure ranges */
	SC7U22_SetAccRange(8);         /* +/-8g */
	SC7U22_SetGyroRange(2000);     /* +/-2000 dps */

	/* Enable sensors */
	SC7U22_SensorEnable(SENSOR_ACC, 1);
	SC7U22_SensorEnable(SENSOR_GYRO, 1);

	/* 回读 POWER_CTRL 确认写入成功 */
	pwr_val = SC7U22_SPI_ReadReg(SC7U22_POWER_CTRL);
	printk("SC7U22 setup done, PWR_CTRL=0x%02X (expected 0x06)\n", pwr_val);

}

uint8_t SC7U22_Init(void)
{
	uint8_t chip_id = 0;
	int retry = 0;

	while ((chip_id != SC7U22_DEVICE_ID) && (retry < 3))
	{
		chip_id = SC7U22_SPI_ReadReg(SC7U22_WHO_AM_I);
		retry++;
		if (chip_id == SC7U22_DEVICE_ID) {
			printk("SC7U22 init success\n");
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

static void read_sensor_12bytes(uint8_t buf[12])
{
	cs_low();
	SPI_SendData(SPI0, SC7U22_ACC_HX | 0x80);
	while (SPI_IsBusy(SPI0)) {}
	SPI_ReceiveData(SPI0);
	for (int i = 0; i < 12; i++) {
		SPI_SendData(SPI0, 0x00);
		while (SPI_IsBusy(SPI0)) {}
		buf[i] = (uint8_t)SPI_ReceiveData(SPI0);
	}
	cs_high();
}

void SC7U22_ReadIMU(void)
{
	uint8_t buf[12];
	read_sensor_12bytes(buf);

	reg_ax = (int16_t)(((uint16_t)buf[0] << 8)  | buf[1]);
	reg_ay = (int16_t)(((uint16_t)buf[2] << 8)  | buf[3]);
	reg_az = (int16_t)(((uint16_t)buf[4] << 8)  | buf[5]);
	reg_gx = (int16_t)(((uint16_t)buf[6] << 8)  | buf[7]);
	reg_gy = (int16_t)(((uint16_t)buf[8] << 8)  | buf[9]);
	reg_gz = (int16_t)(((uint16_t)buf[10] << 8) | buf[11]);

	IMUData.Accelerate.X = (float)reg_ax * 9.8f / SC7U22_ACC_SENSITIVITY - IMUData.BiasAccelerate.X;
	IMUData.Accelerate.Y = (float)reg_ay * 9.8f / SC7U22_ACC_SENSITIVITY - IMUData.BiasAccelerate.Y;
	IMUData.Accelerate.Z = (float)reg_az * 9.8f / SC7U22_ACC_SENSITIVITY - IMUData.BiasAccelerate.Z;
	IMUData.Angular.X    = (float)reg_gx / 57.3f / SC7U22_GYRO_SENSITIVITY - IMUData.BiasAngular.X;
	IMUData.Angular.Y    = (float)reg_gy / 57.3f / SC7U22_GYRO_SENSITIVITY - IMUData.BiasAngular.Y;
	IMUData.Angular.Z    = (float)reg_gz / 57.3f / SC7U22_GYRO_SENSITIVITY - IMUData.BiasAngular.Z;
}

/*===========================================================================
 * Bias calculation
 *===========================================================================*/

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
		uint8_t buf[12];
		read_sensor_12bytes(buf);
		int16_t ax = (int16_t)(((uint16_t)buf[0] << 8)  | buf[1]);
		int16_t ay = (int16_t)(((uint16_t)buf[2] << 8)  | buf[3]);
		int16_t az = (int16_t)(((uint16_t)buf[4] << 8)  | buf[5]);
		int16_t gx = (int16_t)(((uint16_t)buf[6] << 8)  | buf[7]);
		int16_t gy = (int16_t)(((uint16_t)buf[8] << 8)  | buf[9]);
		int16_t gz = (int16_t)(((uint16_t)buf[10] << 8) | buf[11]);

		if (count <= 140)
		{
			sum_bias_acc_x  += (float)ax * 9.8f / SC7U22_ACC_SENSITIVITY;
			sum_bias_acc_y  += (float)ay * 9.8f / SC7U22_ACC_SENSITIVITY;
			sum_bias_acc_z  += (float)az * 9.8f / SC7U22_ACC_SENSITIVITY;
			sum_bias_gyro_x += (float)gx / 57.3f / SC7U22_GYRO_SENSITIVITY;
			sum_bias_gyro_y += (float)gy / 57.3f / SC7U22_GYRO_SENSITIVITY;
			sum_bias_gyro_z += (float)gz / 57.3f / SC7U22_GYRO_SENSITIVITY;
		}
		count--;
	}

	IMUData.BiasAccelerate.X = sum_bias_acc_x / 140.0f;
	IMUData.BiasAccelerate.Y = sum_bias_acc_y / 140.0f;
	IMUData.BiasAccelerate.Z = sum_bias_acc_z / 140.0f - 9.8f;
	IMUData.BiasAngular.X    = sum_bias_gyro_x / 140.0f;
	IMUData.BiasAngular.Y    = sum_bias_gyro_y / 140.0f;
	IMUData.BiasAngular.Z    = sum_bias_gyro_z / 140.0f;

	printk("Bias Acc: %d.%03d %d.%03d %d.%03d | Gyro: %d.%03d %d.%03d %d.%03d\n",
	       (int)IMUData.BiasAccelerate.X, (int)((IMUData.BiasAccelerate.X - (int)IMUData.BiasAccelerate.X) * 1000),
	       (int)IMUData.BiasAccelerate.Y, (int)((IMUData.BiasAccelerate.Y - (int)IMUData.BiasAccelerate.Y) * 1000),
	       (int)IMUData.BiasAccelerate.Z, (int)((IMUData.BiasAccelerate.Z - (int)IMUData.BiasAccelerate.Z) * 1000),
	       (int)IMUData.BiasAngular.X, (int)((IMUData.BiasAngular.X - (int)IMUData.BiasAngular.X) * 1000),
	       (int)IMUData.BiasAngular.Y, (int)((IMUData.BiasAngular.Y - (int)IMUData.BiasAngular.Y) * 1000),
	       (int)IMUData.BiasAngular.Z, (int)((IMUData.BiasAngular.Z - (int)IMUData.BiasAngular.Z) * 1000));

}

/*===========================================================================
 * PWM output (P04 = PWM1_CH0)
 *===========================================================================*/

void SC7U22_PWM_Init(uint32_t freq_hz, uint32_t duty_pct)
{
	SYS_SET_MFP(P0, 4, PWM1_CH0);
	GPIO_EnableDigitalPath(P0, BIT4);
	P0->MODE = (P0->MODE & ~(0x0F << 16)) | (0x01 << 16);

	CLK_APB1PeriphClockCmd(CLK_APB1Periph_PWM1_EN | CLK_APB1Periph_PWM1_CH01, ENABLE);

	PWM_ConfigOutputChannel(PWM1, 0, freq_hz, duty_pct, OPERATION_EDGE_ALIGNED);
	PWM_Start(PWM1, BIT(0));
}

void SC7U22_PWM_SetDuty(uint32_t duty_pct)
{
	if (duty_pct > 100) duty_pct = 100;
	PWM_ConfigOutputChannel(PWM1, 0, 1000, duty_pct, OPERATION_EDGE_ALIGNED);
}

/*===========================================================================
 * INT pin init (INT1 = P05, INT2 = P24)
 *===========================================================================*/

void SC7U22_INT_Init(void)
{
	SC7U22_INT1_INIT();
	SC7U22_INT1_DIGITAL();
	GPIO_SetMode(P0, BIT5, GPIO_MODE_INPUT);

	SC7U22_INT2_INIT();
	SC7U22_INT2_DIGITAL();
	GPIO_SetMode(P2, BIT4, GPIO_MODE_INPUT);
}
