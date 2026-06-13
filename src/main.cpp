/*
 * ESP32-S3 AI 视觉跟踪小车 — 人形跟随版
 * 移植自 STM32 方案, 增加自动跟随 + 已知 bug 修复
 *
 * 改进点:
 *   1. 修复 target_x/target_y 积分饱和 → 限幅到 [0, 180]
 *   2. 增加 motor_stop() (STM32 版缺失)
 *   3. 增加人形自动跟随: 云台水平偏角 → 转向, 框宽 → 前进/停止
 *   4. 丢失目标超时 → 自动停车
 */

#include <Arduino.h>
#include "pin_config.h"
#include "ringbuffer.h"
#include "comm_protocol.h"
#include "pid.h"
#include "motor.h"
#include "servo.h"

// ===== 图像参数 (与 MaixCAM 输出一致) =====
#define IMAGE_WIDTH   320
#define IMAGE_HEIGHT  224

// ===== 云台初始角度 =====
#define SERVO_X_INIT  90.0
#define SERVO_Y_INIT  80.0

// ===== 一阶低通滤波系数 (越小越平滑) =====
#define FILTER_FACTOR  0.01

// ===== 自动跟随参数 =====
#define FOLLOW_SPEED       120    // 跟随前进速度 (0~255)
#define TURN_SPEED         100    // 转向速度
#define PAN_DEADZONE       8.0    // 水平死区 (度), 中心 ± 此值内不转向
#define BBOX_STOP_WIDTH    80     // 框宽 >= 此值 → 目标很近, 停止前进
#define LOST_TIMEOUT_MS    500    // 丢失目标超时 (毫秒)

// ===== 串口 =====
#define CAM_SERIAL  Serial1  // ESP32-S3 UART1

// ===== 环形缓冲区 =====
static rbuffer_t rbuffer_camera;
static uint8_t   _rb_pool[2048];

// ===== PID 实例 =====
static PID pid_servo_x, pid_servo_y;

// ===== 状态机 =====
typedef enum {
    E_PACKET_GET,
    E_SERVO_CTRL,
    E_MOTOR_CTRL,
} task_state_t;

static task_state_t _task_state = E_PACKET_GET;

// ===== 当前目标坐标 =====
static int32_t _x = 0, _y = 0, _width = 0, _height = 0;
static int32_t _dir = 0, _speed = 0;

// ===== 跟随状态 =====
static unsigned long _last_detect_ms = 0;
static bool _target_visible = false;

static portMUX_TYPE _rb_mux = portMUX_INITIALIZER_UNLOCKED;

// ===== 串口接收回调 (onReceive 运行在 FreeRTOS 任务上下文) =====
static void cam_uart_rx_cb(void)
{
    portENTER_CRITICAL(&_rb_mux);
    while (CAM_SERIAL.available()) {
        uint8_t ch = CAM_SERIAL.read();
        rbuffer_putchar_force(&rbuffer_camera, ch);
    }
    portEXIT_CRITICAL(&_rb_mux);
}

// ===== 初始化 =====
void setup()
{
    // 调试串口
    Serial.begin(DBG_BAUD);
    Serial.println("[ESP32-S3] AI Tracking Car starting...");

    // 环形缓冲区
    rbuffer_init(&rbuffer_camera, _rb_pool, sizeof(_rb_pool));

    // MaixCAM 串口
    CAM_SERIAL.begin(CAM_BAUD, SERIAL_8N1, PIN_CAM_RX, PIN_CAM_TX);
    CAM_SERIAL.setRxFIFOFull(1);  // 收到 1 字节就触发回调
    CAM_SERIAL.onReceive(cam_uart_rx_cb);

    // 舵机初始化 → 初始位置
    servo_init();
    servo_set_angle(0, SERVO_X_INIT);
    servo_set_angle(1, SERVO_Y_INIT);

    // PID 初始化 (增量式)
    // X 轴: 图像坐标 → 舵机方向一致
    pid_init(0.102, 0.016, 0, &pid_servo_x);
    pid_servo_x.setValue = IMAGE_WIDTH / 2.0;

    // Y 轴: 图像 Y 向下, 舵机角度向上, 取负号反转
    pid_init(-0.102, -0.016, 0, &pid_servo_y);
    pid_servo_y.setValue = IMAGE_HEIGHT / 2.0;

    // 电机初始化
    motor_init();

    Serial.println("[ESP32-S3] Init done. Waiting for MaixCAM data...");
}

// ===== 协议解析任务 =====
static void task_packet_parse(void)
{
    portENTER_CRITICAL(&_rb_mux);
    uint32_t len = rbuffer_data_len(&rbuffer_camera);
    if (len == 0) {
        portEXIT_CRITICAL(&_rb_mux);
        return;
    }

    uint8_t  buffer[128];
    uint8_t  payload[32];
    uint32_t redundant = 0;

    len = (len <= sizeof(buffer)) ? len : sizeof(buffer);
    rbuffer_peek(&rbuffer_camera, buffer, len);
    portEXIT_CRITICAL(&_rb_mux);

    int32_t rc = packet_check_valid(buffer, len, &redundant);
    if (redundant > 0) {
        portENTER_CRITICAL(&_rb_mux);
        rbuffer_del(&rbuffer_camera, redundant);
        portEXIT_CRITICAL(&_rb_mux);
    }

    if (rc >= 0) {
        // 解码 payload
        packet_decode(&buffer[redundant], len - redundant, payload, sizeof(payload));

        int32_t cmd_type = 0;
        int32_t idx = 0;
        memcpy(&cmd_type, &payload[idx], 4);
        idx += 4;

        if (cmd_type == CMD_OBJECT) {
            memcpy(&_x,      &payload[idx], 4); idx += 4;
            memcpy(&_y,      &payload[idx], 4); idx += 4;
            memcpy(&_width,  &payload[idx], 4); idx += 4;
            memcpy(&_height, &payload[idx], 4); idx += 4;
            _task_state = E_SERVO_CTRL;
        }
        else if (cmd_type == CMD_MOTOR) {
            memcpy(&_dir,   &payload[idx], 4); idx += 4;
            memcpy(&_speed, &payload[idx], 4); idx += 4;
            _task_state = E_MOTOR_CTRL;
        }

        uint32_t pkt_len = packet_length(&buffer[redundant], len - redundant);
        portENTER_CRITICAL(&_rb_mux);
        rbuffer_del(&rbuffer_camera, pkt_len);
        portEXIT_CRITICAL(&_rb_mux);

    } else if (rc == -3 || rbuffer_status(&rbuffer_camera) == RB_FULL) {
        // 帧校验失败或缓冲区满 → 丢弃
        portENTER_CRITICAL(&_rb_mux);
        rbuffer_del(&rbuffer_camera, len);
        portEXIT_CRITICAL(&_rb_mux);
    }
}

// ===== 云台 PID 跟踪任务 =====
static void task_servo_track(void)
{
    static double target_x = SERVO_X_INIT;
    static double target_y = SERVO_Y_INIT;
    static double valx_last = IMAGE_WIDTH  / 2.0;
    static double valy_last = IMAGE_HEIGHT / 2.0;

    // 计算目标中心点
    double valx_now = _x + _width  / 2.0;
    double valy_now = _y + _height / 2.0;

    // 一阶低通滤波
    pid_servo_x.actualValue = valx_now * FILTER_FACTOR + valx_last * (1.0 - FILTER_FACTOR);
    valx_last = pid_servo_x.actualValue;

    pid_servo_y.actualValue = valy_now * FILTER_FACTOR + valy_last * (1.0 - FILTER_FACTOR);
    valy_last = pid_servo_y.actualValue;

    // 增量式 PID
    double inc_x = pid_incremental(&pid_servo_x);
    double inc_y = pid_incremental(&pid_servo_y);

    target_x += inc_x;
    target_y += inc_y;

    // ★ BUG 修复: 积分限幅, 防止饱和 ★
    if (target_x > 180.0) target_x = 180.0;
    if (target_x < 0.0)   target_x = 0.0;
    if (target_y > 180.0) target_y = 180.0;
    if (target_y < 0.0)   target_y = 0.0;

    // 设置舵机
    servo_set_angle(0, target_x);
    servo_set_angle(1, target_y);

    // 记录检测时间
    _last_detect_ms = millis();
    _target_visible = true;
}

// ===== 自动跟随电机控制 =====
static void task_follow_person(void)
{
    // 计算云台水平偏角 (中心 = 90°)
    // target_x 是 PID 累加值, 代表当前舵机角度
    // 我们用 _x + _width/2 相对于图像中心的偏差来判断
    double center_x = _x + _width / 2.0;
    double error_x  = center_x - IMAGE_WIDTH / 2.0;  // 正 = 目标在右边

    // 计算框宽 (代表距离)
    // 框宽越大 → 目标越近

    // 将角度死区近似转换为像素: IMAGE_WIDTH/FOV ≈ 320/180 px/deg
    if (fabs(error_x) > PAN_DEADZONE * (IMAGE_WIDTH / 180.0)) {
        // 目标偏离中心 → 转向
        if (error_x > 0) {
            // 目标在右边 → 右转 (云台向右看, 车向右转)
            motor_move(DIR_RIGHT, TURN_SPEED);
        } else {
            // 目标在左边 → 左转
            motor_move(DIR_LEFT, TURN_SPEED);
        }
    }
    else if (_width < BBOX_STOP_WIDTH) {
        // 目标在中心但较远 → 前进
        motor_move(DIR_FORWARD, FOLLOW_SPEED);
    }
    else {
        // 目标在中心且足够近 → 停止
        motor_stop();
    }
}

// ===== 主循环 =====
void loop()
{
    // 1. 协议解析
    task_packet_parse();

    // 2. 根据状态分发
    switch (_task_state) {
    case E_SERVO_CTRL:
        task_servo_track();
        task_follow_person();   // 跟踪的同时自动跟随
        _task_state = E_PACKET_GET;
        break;

    case E_MOTOR_CTRL:
        // 远程手动控制 (来自 MaixCAM TCP 命令)
        motor_move((dir_t)_dir, (uint32_t)_speed);
        _task_state = E_PACKET_GET;
        break;

    case E_PACKET_GET:
    default:
        break;
    }

    // 3. 丢失目标超时 → 停车
    if (_target_visible && (millis() - _last_detect_ms > LOST_TIMEOUT_MS)) {
        motor_stop();
        _target_visible = false;
        Serial.println("[FOLLOW] Target lost, stopping.");
    }

    // 让出 CPU 给 FreeRTOS 调度 (WiFi/BT/idle)
    vTaskDelay(1);
}
