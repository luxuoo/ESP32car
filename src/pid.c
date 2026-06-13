/* PID 控制器 — 移植自 STM32 方案, 无修改 */

#include "pid.h"

void pid_init(double kp, double ki, double kd, PID *pid)
{
    memset(pid, 0, sizeof(PID));
    pid->Kp = kp;
    pid->Ki = ki;
    pid->Kd = kd;
}

/* 位置式 PID */
double pid_position(PID *pid)
{
    pid->error = pid->setValue - pid->actualValue;
    pid->integral += pid->error;

    pid->P = pid->Kp * pid->error;
    pid->I = pid->Ki * pid->integral;
    pid->D = pid->Kd * (pid->error - pid->errorPre);
    double setVal = pid->P + pid->I + pid->D;

    pid->errorPrePre = pid->errorPre;
    pid->errorPre    = pid->error;

    return setVal;
}

/* 增量式 PID */
double pid_incremental(PID *pid)
{
    pid->error = pid->setValue - pid->actualValue;

    pid->P = pid->Kp * (pid->error - pid->errorPre);
    pid->I = pid->Ki * pid->error;
    pid->D = pid->Kd * (pid->error - 2 * pid->errorPre + pid->errorPrePre);
    double increment = pid->P + pid->I + pid->D;

    pid->errorPrePre = pid->errorPre;
    pid->errorPre    = pid->error;

    return increment;
}
