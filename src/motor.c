/* 电机驱动 — TB6612FNG, ESP32-S3 LEDC PWM
 *
 * TB6612 真值表 (单通道):
 *   IN1=H  IN2=L  → 正转
 *   IN1=L  IN2=H  → 反转
 *   IN1=L  IN2=L  → 制动(滑行)
 *   IN1=H  IN2=H  → 制动(短路)
 *
 * 本板每侧 1 个电机 (差速转向), 共 2 个电机:
 *   左侧: LPWM(PWM) + LAB1/AIN1 + LAB2/AIN2
 *   右侧: RPWM(PWM) + RAB1/BIN1 + RAB2/BIN2
 */

#include "motor.h"
#include "pin_config.h"

#define CH_LPWM  2   // LEDC 通道 2
#define CH_RPWM  3   // LEDC 通道 3

void motor_init(void)
{
    // 方向引脚
    pinMode(PIN_LAB1, OUTPUT);
    pinMode(PIN_LAB2, OUTPUT);
    pinMode(PIN_RAB1, OUTPUT);
    pinMode(PIN_RAB2, OUTPUT);

    // LEDC PWM 通道初始化: ledcSetup(channel, freq, resolution)
    ledcSetup(CH_LPWM, MOTOR_PWM_FREQ, MOTOR_PWM_RES);
    ledcSetup(CH_RPWM, MOTOR_PWM_FREQ, MOTOR_PWM_RES);

    // LEDC 通道绑定引脚: ledcAttachPin(pin, channel)
    ledcAttachPin(PIN_LPWM, CH_LPWM);
    ledcAttachPin(PIN_RPWM, CH_RPWM);

    motor_stop();
}

/* side: 0=左侧, 1=右侧 */
void motor_ctrl(uint8_t side, uint8_t forward, uint32_t speed)
{
    if (speed > 255) speed = 255;

    if (side == 0) {
        // 左电机: AIN1=H, AIN2=L → 正转
        digitalWrite(PIN_LAB1, forward ? HIGH : LOW);
        digitalWrite(PIN_LAB2, forward ? LOW  : HIGH);
        ledcWrite(CH_LPWM, speed);
    } else {
        // 右电机: BIN1=H, BIN2=L → 正转
        digitalWrite(PIN_RAB1, forward ? HIGH : LOW);
        digitalWrite(PIN_RAB2, forward ? LOW  : HIGH);
        ledcWrite(CH_RPWM, speed);
    }
}

void motor_move(dir_t dir, uint32_t speed)
{
    if (speed > 255) speed = 255;

    switch (dir) {
    case DIR_FORWARD:
        motor_ctrl(0, 1, speed);  // 左前进
        motor_ctrl(1, 1, speed);  // 右前进
        break;
    case DIR_BACKWARD:
        motor_ctrl(0, 0, speed);  // 左后退
        motor_ctrl(1, 0, speed);  // 右后退
        break;
    case DIR_LEFT:
        motor_ctrl(0, 0, speed);  // 左后退
        motor_ctrl(1, 1, speed);  // 右前进
        break;
    case DIR_RIGHT:
        motor_ctrl(0, 1, speed);  // 左前进
        motor_ctrl(1, 0, speed);  // 右后退
        break;
    default:
        motor_stop();
        break;
    }
}

void motor_stop(void)
{
    ledcWrite(CH_LPWM, 0);
    ledcWrite(CH_RPWM, 0);
    digitalWrite(PIN_LAB1, LOW);
    digitalWrite(PIN_LAB2, LOW);
    digitalWrite(PIN_RAB1, LOW);
    digitalWrite(PIN_RAB2, LOW);
}
