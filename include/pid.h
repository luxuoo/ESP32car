#ifndef __PID_H__
#define __PID_H__

#include <string.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct _PID {
    double actualValue;    // 实际值 (传感器/视觉采集)
    double setValue;       // 设定值 (目标)
    double Kp, Ki, Kd;    // 比例、积分、微分系数
    double P, I, D;        // 比例项、积分项、微分项
    double error;          // 当前误差
    double errorPre;       // E[k-1] 上次误差
    double errorPrePre;    // E[k-2] 上上次误差
    double integral;       // 积分值
} PID;

void   pid_init(double kp, double ki, double kd, PID *pid);
double pid_position(PID *pid);
double pid_incremental(PID *pid);

#ifdef __cplusplus
}
#endif

#endif
