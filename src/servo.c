/* SG90 舵机驱动 — ESP32-S3 LEDC PWM (50Hz, 16-bit 精度)
 *
 * SG90 参数:
 *   0°   → 0.5ms 脉宽
 *   180° → 2.5ms 脉宽
 *   周期 20ms (50Hz)
 *
 * 16-bit 分辨率: 2^16 = 65536 个计数
 *   0.5ms → 65536 * 0.5/20  = 1638
 *   2.5ms → 65536 * 2.5/20  = 8192
 */

#include "servo.h"
#include "pin_config.h"

#define CH_SERVO_X  0   // LEDC 通道 0
#define CH_SERVO_Y  1   // LEDC 通道 1

#define SERVO_MIN_COUNT  1638   // 0.5ms @ 50Hz / 16-bit
#define SERVO_MAX_COUNT  8192   // 2.5ms @ 50Hz / 16-bit

void servo_init(void)
{
    // ledcSetup(channel, freq, resolution_bits)
    ledcSetup(CH_SERVO_X, SERVO_PWM_FREQ, SERVO_PWM_RES);
    ledcSetup(CH_SERVO_Y, SERVO_PWM_FREQ, SERVO_PWM_RES);

    // ledcAttachPin(pin, channel)
    ledcAttachPin(PIN_SERVO_X, CH_SERVO_X);
    ledcAttachPin(PIN_SERVO_Y, CH_SERVO_Y);
}

void servo_set_angle(uint8_t channel, double angle)
{
    // 限幅 [0, 180]
    if (angle > 180.0) angle = 180.0;
    if (angle < 0.0)   angle = 0.0;

    // 角度 → 脉宽计数
    uint32_t duty = (uint32_t)(SERVO_MIN_COUNT +
        (angle / 180.0) * (SERVO_MAX_COUNT - SERVO_MIN_COUNT));

    ledcWrite(channel == 0 ? CH_SERVO_X : CH_SERVO_Y, duty);
}
