/*
 * gyroscope.c - 陀螺仪姿态解算模块
 *
 * 移植自: Wheelie_AngleMeassure_SC7/User/src/Gyroscope.c
 *
 * Mahony 互补滤波 + 角度转 PWM 输出 (P04)
 * 依赖: sc7u22_spi.c (SC7U22_ReadIMU, SC7U22_BiasCalculate, SC7U22_PWM_SetDuty)
 */

#include "gyroscope.h"
#include "sc7u22_spi.h"
#include <string.h>

/*===========================================================================
 * 嵌入式数学函数 (替代 libm, 避免引入 newlib 导致栈/堆溢出)
 *===========================================================================*/

static inline float sqrtf_approx(float x)
{
	int32_t i;
	float y;

	if (x <= 0.0f) return 0.0f;

	/* 初始猜测 (IEEE 754指数折半) */
	memcpy(&i, &x, sizeof(i));
	i = 0x1FBD3F7D + (i >> 1);
	memcpy(&y, &i, sizeof(y));

	/* 两次牛顿迭代 → ~0.01% 精度 */
	y = (y + x / y) * 0.5f;
	y = (y + x / y) * 0.5f;

	return y;
}

static inline float atan2f_approx(float y, float x)
{
	float z, abs_z, atan;

	if (x == 0.0f) {
		if (y > 0.0f) return 1.57079633f;
		if (y < 0.0f) return -1.57079633f;
		return 0.0f;
	}

	z = y / x;
	abs_z = (z < 0.0f) ? -z : z;

	if (abs_z <= 1.0f) {
		float z2 = z * z;
		atan = z * (0.9998660f + z2 * (-0.3302995f +
			z2 * (0.1801410f + z2 * (-0.0851330f + z2 * 0.0208351f))));
	} else {
		float inv = 1.0f / z;
		float inv2 = inv * inv;
		atan = 1.57079633f - inv * (0.9998660f + inv2 * (-0.3302995f +
			inv2 * (0.1801410f + inv2 * (-0.0851330f + inv2 * 0.0208351f))));
		if (z < 0.0f) atan = -atan;
	}

	if (x < 0.0f)
		atan += (y >= 0.0f) ? 3.14159265f : -3.14159265f;

	return atan;
}

static inline float asinf_approx(float x)
{
	float abs_x;

	/* 限幅 */
	if (x > 1.0f) x = 1.0f;
	if (x < -1.0f) x = -1.0f;

	abs_x = (x < 0.0f) ? -x : x;

	if (abs_x < 0.9f) {
		float x2 = x * x;
		return x * (1.0f + x2 * (0.16666667f + x2 * (0.075f +
			x2 * (0.04464286f + x2 * (0.03038194f + x2 * 0.02237216f)))));
	} else {
		/* 近 ±1 时用补角公式 */
		float sqrt_term = sqrtf_approx(1.0f - abs_x);
		float result = 1.57079633f - sqrt_term * (1.57079633f - sqrt_term *
			(0.2146018f - sqrt_term * (0.0886915f - sqrt_term * 0.0424717f)));
		return (x < 0.0f) ? -result : result;
	}
}

/*===========================================================================
 * Mahony 滤波参数 (对应原工程 Gyroscope.c:12-15)
 *===========================================================================*/
#define Kp     20.0f       /* 比例增益 */
#define Ki     0.01f       /* 积分增益 */
#define HALF_T 0.01f     /* 半采样周期: 20ms/2 (匹配 SC7U22 200Hz ODR) */

/*===========================================================================
 * 四元数 & 积分误差 (对应原工程 Gyroscope.c:18-19)
 *===========================================================================*/
static float q0 = 1.0f, q1 = 0.0f, q2 = 0.0f, q3 = 0.0f;
static float exInt = 0.0f, eyInt = 0.0f, ezInt = 0.0f;

/*===========================================================================
 * 全局变量 (对应原工程 Gyroscope.c:20, Gyroscope.h:49-52)
 *===========================================================================*/
float Q_ANGLE_X = 0.0f, Q_ANGLE_Y = 0.0f, Q_ANGLE_Z = 0.0f;
tsGyroscope Gyroscope;
uint8_t flag_gyro = 0;

/*===========================================================================
 * GyroscopeInit (对应原工程 Gyroscope.c:29-33)
 *
 * 原工程: memset + BiasCalculate (5样本均值)
 * 新工程: memset + SC7U22_BiasCalculate (200样本/140有效)
 *===========================================================================*/
void GyroscopeInit(void)
{
	memset(&Gyroscope, 0, sizeof(Gyroscope));
	SC7U22_BiasCalculate();
}

/*===========================================================================
 * GyrosCopeDataRead (对应原工程 Gyroscope.c:74-78)
 *
 * 原工程: sc7u22_read_imu() + IMUupdate(&IMUData)
 * 新工程: SC7U22_ReadIMU() + IMUupdate()
 *===========================================================================*/
void GyrosCopeDataRead(void)
{
	SC7U22_ReadIMU();
	IMUupdate();
}

/*===========================================================================
 * IMUupdate - Mahony 互补滤波 (对应原工程 Gyroscope.c:80-184)
 *
 * 输入: sc7u22_spi.c 的全局 IMUData (tsIMUData 类型)
 * 输出: Gyroscope.Angle (pitch, 度)
 *       PWM 占空比通过 SC7U22_PWM_SetDuty() → P04
 *
 * 角度到PWM映射 (与旧工程一致, 12-bit → 百分比转换):
 *   Q_ANGLE_Y < 0      → duty = 0%
 *   Q_ANGLE_Y >= 79.98  → duty = 100%
 *   ax < 0              → duty = 100%
 *   其他                → duty = Q_ANGLE_Y * 51.2 / 4095 * 100
 *===========================================================================*/
void IMUupdate(void)
{
	uint16_t PwmDuty;
	float norm;
	float vx, vy, vz;
	float ex, ey, ez;
	float gx, gy, gz;
	float ax, ay, az;

	/* 预计算四元数乘积 */
	float q0q0 = q0 * q0;
	float q0q1 = q0 * q1;
	float q0q2 = q0 * q2;
	float q0q3 = q0 * q3;
	float q1q1 = q1 * q1;
	float q1q2 = q1 * q2;
	float q1q3 = q1 * q3;
	float q2q2 = q2 * q2;
	float q2q3 = q2 * q3;
	float q3q3 = q3 * q3;

	/* 从 SC7U22 驱动获取已减零偏的传感器数据
	 * gy 取反: 原工程坐标转换, 保持一致 */
	gx = IMUData.Angular.X;
	gy = -IMUData.Angular.Y;
	gz = IMUData.Angular.Z;
	ax = IMUData.Accelerate.X;
	ay = IMUData.Accelerate.Y;
	az = IMUData.Accelerate.Z;

	if (ax * ay * az == 0.0f) {
		return;
	}

	/* 加速度归一化 */
	norm = sqrtf_approx(ax * ax + ay * ay + az * az);
	ax = ax / norm;
	ay = ay / norm;
	az = az / norm;

	/* 重力向量估计值 (从四元数推导) */
	vx = -1.0f * (q0q0 + q1q1 - q2q2 - q3q3);
	vy = -1.0f * (2.0f * (q1q2 - q0q3));
	vz = -1.0f * (2.0f * (q1q3 + q0q2));

	/* 加速度计与重力估计的叉积 → 误差 */
	ex = (ay * vz - az * vy);
	ey = (az * vx - ax * vz);
	ez = (ax * vy - ay * vx);

	/* 积分误差 */
	exInt = exInt + ex * Ki;
	eyInt = eyInt + ey * Ki;
	ezInt = ezInt + ez * Ki;

	/* PI 校正陀螺仪数据 */
	gx = gx + Kp * ex + exInt;
	gy = gy + Kp * ey + eyInt;
	gz = gz + Kp * ez + ezInt;

	/* 一阶龙格-库塔法更新四元数 */
	float q0temp = q0;
	float q1temp = q1;
	float q2temp = q2;
	float q3temp = q3;

	q0 = q0temp + (-q1temp * gx - q2temp * gy - q3temp * gz) * HALF_T;
	q1 = q1temp + ( q0temp * gx + q2temp * gz - q3temp * gy) * HALF_T;
	q2 = q2temp + ( q0temp * gy - q1temp * gz + q3temp * gx) * HALF_T;
	q3 = q3temp + ( q0temp * gz + q1temp * gy - q2temp * gx) * HALF_T;

	/* 四元数归一化 */
	norm = sqrtf_approx(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
	q0 = q0 / norm;
	q1 = q1 / norm;
	q2 = q2 / norm;
	q3 = q3 / norm;

	/* 四元数 → 欧拉角 (度) */
	Q_ANGLE_Z = atan2f_approx(2.0f * q1 * q2 - 2.0f * q0 * q3,
			   -2.0f * q2 * q2 - 2.0f * q3 * q3 + 1.0f) * 57.3f; /* yaw */
	Q_ANGLE_Y = -asinf_approx(2.0f * q1 * q3 + 2.0f * q0 * q2) * 57.3f;  /* pitch */
	Q_ANGLE_X = Q_ANGLE_X + gx * HALF_T * 2.0f * 57.3f;            /* roll */

	Gyroscope.Angle = Q_ANGLE_Y;

	/* 角度 → PWM 占空比
	 *
	 * 原工程: 12-bit DAC (0~4095), DAC_ALIGN_12B_R
	 * 新工程: SC7U22_PWM_SetDuty 使用百分比 (0~100)
	 *
	 * 映射关系 (与原工程完全一致):
	 *   Q_ANGLE_Y < 0       → 0%
	 *   Q_ANGLE_Y >= 79.98  → 100%
	 *   ax < 0              → 100%
	 *   else                → Q_ANGLE_Y / 79.98 * 100
	 */
	if (Q_ANGLE_Y < 0.0f) {
		PwmDuty = 0;
	} else if (Q_ANGLE_Y >= 79.98f) {
		PwmDuty = 4095;
	} else if (ax < 0.0f) {
		PwmDuty = 4095;
	} else {
		PwmDuty = (uint16_t)(Q_ANGLE_Y * 51.2f);
	}

	SC7U22_PWM_SetDuty((uint32_t)(PwmDuty * 100U / 4095U));


	int pitch_int = (int)Q_ANGLE_Y;
	int pitch_dec = (int)((Q_ANGLE_Y - (float)pitch_int) * 10.0f);
	if (pitch_dec < 0) pitch_dec = -pitch_dec;
	printk("GYRO: ax=%d ay=%d az=%d | gx=%d gy=%d gz=%d | "
		"pitch=%d.%d pwm=%u(%u%%)\n",
		reg_ax, reg_ay, reg_az,
		reg_gx, reg_gy, reg_gz,
		pitch_int, pitch_dec,
		PwmDuty, (uint32_t)(PwmDuty * 100U / 4095U));

}
