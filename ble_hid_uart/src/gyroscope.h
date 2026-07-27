#ifndef _GYROSCOPE_H_
#define _GYROSCOPE_H_

#include <stdint.h>

/*
 * 陀螺仪应用层 - 移植自原工程 User/inc/Gyroscope.h
 *
 * 依赖: sc7u22_spi.c (传感器驱动 + PWM输出)
 */

/* 角度输出结构体 (对应原工程 Gyroscope.h:33-37) */
typedef struct {
	float Angle; /* pitch 角度, 单位:度 */
} tsGyroscope;

/* 全局变量 (对应原工程 Gyroscope.h:49-52) */
extern tsGyroscope Gyroscope;
extern float Q_ANGLE_X, Q_ANGLE_Y, Q_ANGLE_Z;
extern uint8_t flag_gyro;

/* API (对应原工程 Gyroscope.h:53-55) */
void GyroscopeInit(void);
void GyrosCopeDataRead(void);

#endif /* _GYROSCOPE_H_ */
