/*
 * ESP32-S3 AI 视觉跟踪小车 — 串口调试版
 *
 * USB 串口命令 (115200 baud):
 *   S:<x_angle>:<y_angle>         设置舵机角度 (0~180)
 *   M:<dir>:<speed>               电机控制 (dir: 0停止 1前进 2后退 3左转 4右转)
 *   T:<x>:<y>:<w>:<h>            注入模拟目标坐标 (测试跟踪逻辑)
 *   P:<kp>:<ki>:<kd>             实时修改 PID 参数 (作用于 X 轴)
 *   D                              打印一次完整状态
 *   ?                              打印帮助
 *
 * 每 500ms 自动输出一行状态: 时间 | 舵机 | 电机 | 目标 | PID | 缓冲区
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

// ===== 调试输出间隔 =====
#define DBG_INTERVAL_MS  500

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

// ===== 调试: 当前舵机角度 & 电机状态 =====
static double _cur_servo_x = SERVO_X_INIT;
static double _cur_servo_y = SERVO_Y_INIT;
static dir_t  _cur_motor_dir = DIR_STOP;
static uint32_t _cur_motor_speed = 0;

// ===== 调试: 统计 =====
static uint32_t _pkt_count = 0;       // 收到的有效包数
static uint32_t _pkt_err_count = 0;   // 校验失败包数
static unsigned long _last_dbg_ms = 0;
static unsigned long _last_pkt_ms = 0; // 最后收到 MaixCAM 包的时间

static portMUX_TYPE _rb_mux = portMUX_INITIALIZER_UNLOCKED;

// ===== 串口接收回调 =====
static void cam_uart_rx_cb(void)
{
    portENTER_CRITICAL(&_rb_mux);
    while (CAM_SERIAL.available()) {
        uint8_t ch = CAM_SERIAL.read();
        rbuffer_putchar_force(&rbuffer_camera, ch);
    }
    portEXIT_CRITICAL(&_rb_mux);
}

// ===== 帮助信息 =====
static void print_help(void)
{
    Serial.println("=== ESP32-S3 Debug Commands ===");
    Serial.println("  S:<x>:<y>         Servo angle (0~180)");
    Serial.println("  M:<dir>:<speed>   Motor (0=stop 1=fwd 2=back 3=left 4=right)");
    Serial.println("  T:<x>:<y>:<w>:<h> Simulate target bbox");
    Serial.println("  P:<kp>:<ki>:<kd>  Set PID (X axis)");
    Serial.println("  D                  Dump status");
    Serial.println("  ?                  This help");
    Serial.println("===============================");
}

// ===== 格式化电机方向 =====
static const char* dir_str(dir_t d)
{
    switch (d) {
        case DIR_STOP:     return "STOP";
        case DIR_FORWARD:  return "FWD";
        case DIR_BACKWARD: return "BACK";
        case DIR_LEFT:     return "LEFT";
        case DIR_RIGHT:    return "RIGHT";
        default:           return "???";
    }
}

// ===== 输出一行状态 =====
static void print_status(void)
{
    uint32_t rb_used = 0;
    portENTER_CRITICAL(&_rb_mux);
    rb_used = rbuffer_data_len(&rbuffer_camera);
    portEXIT_CRITICAL(&_rb_mux);

    unsigned long now = millis();

    // 格式: [ms] SV:x,y MT:dir,speed | TGT:x,y,w,h vis:0/1 last_ms | PID:set,act,P,I,D | RB:used/total PKT:ok/err
    Serial.printf("[%lu] ", now);
    Serial.printf("SV:%.1f,%.1f ", _cur_servo_x, _cur_servo_y);
    Serial.printf("MT:%s,%lu ", dir_str(_cur_motor_dir), _cur_motor_speed);
    Serial.printf("| TGT:%ld,%ld,%ld,%ld vis:%d ", _x, _y, _width, _height, _target_visible ? 1 : 0);
    Serial.printf("| PID_X:set=%.1f,act=%.1f,P=%.3f,I=%.3f ", pid_servo_x.setValue, pid_servo_x.actualValue, pid_servo_x.P, pid_servo_x.I);
    Serial.printf("| RB:%lu/%d PKT:%lu/%lu", rb_used, (int)sizeof(_rb_pool), _pkt_count, _pkt_err_count);
    Serial.println();
}

// ===== 解析 USB 串口输入命令 =====
static void task_serial_cmd(void)
{
    static char cmd_buf[128];
    static uint8_t cmd_idx = 0;

    while (Serial.available()) {
        char ch = Serial.read();
        if (ch == '\n' || ch == '\r') {
            if (cmd_idx == 0) continue;
            cmd_buf[cmd_idx] = '\0';

            // 解析命令
            if (cmd_buf[0] == '?') {
                print_help();
            }
            else if (cmd_buf[0] == 'D') {
                print_status();
            }
            else if (cmd_buf[0] == 'S') {
                // S:<x>:<y>
                double sx, sy;
                if (sscanf(cmd_buf, "S:%lf:%lf", &sx, &sy) == 2) {
                    servo_set_angle(0, sx);
                    servo_set_angle(1, sy);
                    _cur_servo_x = sx;
                    _cur_servo_y = sy;
                    Serial.printf("[CMD] Servo -> %.1f, %.1f\n", sx, sy);
                } else {
                    Serial.println("[ERR] Format: S:<x>:<y>");
                }
            }
            else if (cmd_buf[0] == 'M') {
                // M:<dir>:<speed>
                int d, s;
                if (sscanf(cmd_buf, "M:%d:%d", &d, &s) == 2) {
                    motor_move((dir_t)d, (uint32_t)s);
                    _cur_motor_dir = (dir_t)d;
                    _cur_motor_speed = s;
                    Serial.printf("[CMD] Motor -> dir=%s speed=%d\n", dir_str((dir_t)d), s);
                } else {
                    Serial.println("[ERR] Format: M:<dir>:<speed>");
                }
            }
            else if (cmd_buf[0] == 'T') {
                // T:<x>:<y>:<w>:<h>
                int32_t tx, ty, tw, th;
                if (sscanf(cmd_buf, "T:%ld:%ld:%ld:%ld", &tx, &ty, &tw, &th) == 4) {
                    _x = tx; _y = ty; _width = tw; _height = th;
                    _task_state = E_SERVO_CTRL;
                    Serial.printf("[CMD] Target -> x=%ld y=%ld w=%ld h=%ld\n", tx, ty, tw, th);
                } else {
                    Serial.println("[ERR] Format: T:<x>:<y>:<w>:<h>");
                }
            }
            else if (cmd_buf[0] == 'P') {
                // P:<kp>:<ki>:<kd>
                double kp, ki, kd;
                if (sscanf(cmd_buf, "P:%lf:%lf:%lf", &kp, &ki, &kd) == 3) {
                    pid_servo_x.Kp = kp;
                    pid_servo_x.Ki = ki;
                    pid_servo_x.Kd = kd;
                    // Y 轴取反
                    pid_servo_y.Kp = -kp;
                    pid_servo_y.Ki = -ki;
                    pid_servo_y.Kd = -kd;
                    Serial.printf("[CMD] PID -> Kp=%.4f Ki=%.4f Kd=%.4f\n", kp, ki, kd);
                } else {
                    Serial.println("[ERR] Format: P:<kp>:<ki>:<kd>");
                }
            }
            else {
                Serial.printf("[ERR] Unknown: %s (type ? for help)\n", cmd_buf);
            }

            cmd_idx = 0;
        } else {
            if (cmd_idx < sizeof(cmd_buf) - 1)
                cmd_buf[cmd_idx++] = ch;
        }
    }
}

// ===== 初始化 =====
void setup()
{
    Serial.begin(DBG_BAUD);
    Serial.println("\n[ESP32-S3] AI Tracking Car — DEBUG MODE");
    Serial.println("[ESP32-S3] Type ? for commands");

    // 环形缓冲区
    rbuffer_init(&rbuffer_camera, _rb_pool, sizeof(_rb_pool));

    // MaixCAM 串口
    CAM_SERIAL.begin(CAM_BAUD, SERIAL_8N1, PIN_CAM_RX, PIN_CAM_TX);
    CAM_SERIAL.setRxFIFOFull(1);
    CAM_SERIAL.onReceive(cam_uart_rx_cb);

    // 舵机初始化
    servo_init();
    servo_set_angle(0, SERVO_X_INIT);
    servo_set_angle(1, SERVO_Y_INIT);

    // PID 初始化 (增量式)
    pid_init(0.102, 0.016, 0, &pid_servo_x);
    pid_servo_x.setValue = IMAGE_WIDTH / 2.0;

    pid_init(-0.102, -0.016, 0, &pid_servo_y);
    pid_servo_y.setValue = IMAGE_HEIGHT / 2.0;

    // 电机初始化
    motor_init();

    Serial.println("[ESP32-S3] Init done.");
    print_help();
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

        _pkt_count++;
        _last_pkt_ms = millis();

    } else if (rc == -3 || rbuffer_status(&rbuffer_camera) == RB_FULL) {
        portENTER_CRITICAL(&_rb_mux);
        rbuffer_del(&rbuffer_camera, len);
        portEXIT_CRITICAL(&_rb_mux);
        _pkt_err_count++;
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
    _cur_servo_x = target_x;
    _cur_servo_y = target_y;

    // 记录检测时间
    _last_detect_ms = millis();
    _target_visible = true;
}

// ===== 自动跟随电机控制 =====
static void task_follow_person(void)
{
    double center_x = _x + _width / 2.0;
    double error_x  = center_x - IMAGE_WIDTH / 2.0;

    // 将角度死区近似转换为像素: IMAGE_WIDTH/FOV ≈ 320/180 px/deg
    if (fabs(error_x) > PAN_DEADZONE * (IMAGE_WIDTH / 180.0)) {
        if (error_x > 0) {
            motor_move(DIR_RIGHT, TURN_SPEED);
            _cur_motor_dir = DIR_RIGHT;
        } else {
            motor_move(DIR_LEFT, TURN_SPEED);
            _cur_motor_dir = DIR_LEFT;
        }
        _cur_motor_speed = TURN_SPEED;
    }
    else if (_width < BBOX_STOP_WIDTH) {
        motor_move(DIR_FORWARD, FOLLOW_SPEED);
        _cur_motor_dir = DIR_FORWARD;
        _cur_motor_speed = FOLLOW_SPEED;
    }
    else {
        motor_stop();
        _cur_motor_dir = DIR_STOP;
        _cur_motor_speed = 0;
    }
}

// ===== 主循环 =====
void loop()
{
    // 0. 处理 USB 串口调试命令
    task_serial_cmd();

    // 1. 协议解析 (MaixCAM 数据)
    task_packet_parse();

    // 2. 根据状态分发
    switch (_task_state) {
    case E_SERVO_CTRL:
        task_servo_track();
        task_follow_person();
        _task_state = E_PACKET_GET;
        break;

    case E_MOTOR_CTRL:
        motor_move((dir_t)_dir, (uint32_t)_speed);
        _cur_motor_dir = (dir_t)_dir;
        _cur_motor_speed = (uint32_t)_speed;
        _task_state = E_PACKET_GET;
        break;

    case E_PACKET_GET:
    default:
        break;
    }

    // 3. 丢失目标超时 → 停车
    if (_target_visible && (millis() - _last_detect_ms > LOST_TIMEOUT_MS)) {
        motor_stop();
        _cur_motor_dir = DIR_STOP;
        _cur_motor_speed = 0;
        _target_visible = false;
        Serial.println("[FOLLOW] Target lost, stopping.");
    }

    // 4. 周期性状态输出
    if (millis() - _last_dbg_ms >= DBG_INTERVAL_MS) {
        _last_dbg_ms = millis();
        print_status();
    }

    vTaskDelay(1);
}
