#ifndef __MOTOR_H__
#define __MOTOR_H__

#include <Arduino.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    DIR_STOP = 0,
    DIR_FORWARD,
    DIR_BACKWARD,
    DIR_LEFT,
    DIR_RIGHT,
} dir_t;

// 初始化电机 PWM + 方向引脚
void motor_init(void);

// 底层: 控制单侧电机  side=0 左侧, side=1 右侧, forward=1前进, speed=0~255
void motor_ctrl(uint8_t side, uint8_t forward, uint32_t speed);

// 高层: 全向运动
void motor_move(dir_t dir, uint32_t speed);

// 停车
void motor_stop(void);

#ifdef __cplusplus
}
#endif

#endif
