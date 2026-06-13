#ifndef __PIN_CONFIG_H__
#define __PIN_CONFIG_H__

// ===== 电机驱动 (TB6612FNG, 每侧 1 个电机, 差速转向) =====
#define PIN_LPWM   4    // 左侧 PWM  (TB6612 PWMA)
#define PIN_LAB1   5    // 左侧方向1 (TB6612 AIN1)
#define PIN_LAB2   6    // 左侧方向2 (TB6612 AIN2)
#define PIN_RPWM   7    // 右侧 PWM  (TB6612 PWMB)
#define PIN_RAB1  15    // 右侧方向1 (TB6612 BIN1)
#define PIN_RAB2  16    // 右侧方向2 (TB6612 BIN2)

// ===== 云台舵机 (SG90) =====
#define PIN_SERVO_X  17   // 水平舵机
#define PIN_SERVO_Y  18   // 俯仰舵机

// ===== MaixCAM 串口 =====
#define PIN_CAM_RX  10   // ← MaixCAM TX
#define PIN_CAM_TX   9   // → MaixCAM RX

// ===== PWM 参数 =====
#define MOTOR_PWM_FREQ   20000   // 电机 20kHz (静音)
#define MOTOR_PWM_RES    8       // 8-bit: 0~255
#define SERVO_PWM_FREQ   50      // 舵机 50Hz
#define SERVO_PWM_RES    16      // 16-bit 精度

// ===== 串口 =====
#define CAM_BAUD   115200
#define DBG_BAUD   115200

#endif
