#ifndef __SERVO_H__
#define __SERVO_H__

#include <Arduino.h>

#ifdef __cplusplus
extern "C" {
#endif

// 初始化两路舵机 PWM (LEDC)
void servo_init(void);

// 设置舵机角度  channel: 0=水平X, 1=俯仰Y, angle: 0~180
void servo_set_angle(uint8_t channel, double angle);

#ifdef __cplusplus
}
#endif

#endif
