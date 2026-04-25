#include "ti_msp_dl_config.h"
#include "headfile.h"
// ========== 外部变量引用 ==========
extern motor_config trackless_motor;
extern float turn_scale;
extern float speed_kp, speed_ki, speed_kd;
extern float speed_setup;
extern float gray_status[2];
extern _gray_state gray_state;
extern float turn_output;
extern LOCK_STATE unlock_flag;

// 灰度检测中增加的横向暂停线标志（在 gray_detection.h 中声明）
extern uint8_t pause_line_detected;

LOCK_STATE unlock_flag = UNLOCK;
volatile uint8_t start_flag = 0;

static uint16_t low_speed_timer = 0;      // 软启动计时（5ms单位）
static uint16_t lost_timer = 0;
static float last_turn_output = 0;

// 状态机
typedef enum {
    NORMAL,
    DELAY_STOP,      // 直角弯延迟前进（编码器里程）
    STOP_AND_TURN,   // 原地旋转
    PAUSE,           // B点暂停
} TurnState;
static TurnState turn_state = NORMAL;
static float target_angle = 0;
static float relative_angle = 0;

// 编码器里程计相关（直角弯延迟前进）
static int32_t start_encoder_pulse = 0;
static uint8_t encoder_recorded = 0;
#define TARGET_DISTANCE_CM  4.0f                     // 直角弯延迟前进距离（cm）
#define PULSE_PER_CM  (pulse_cnt_per_circle_default / (2.0f * 3.1415926f * tire_radius_cm_default))
#define TARGET_PULSE   ((int32_t)(TARGET_DISTANCE_CM * PULSE_PER_CM))

// B点辅助
static uint8_t first_turn_completed = 0;              // 是否已完成第一个直角弯
static int32_t start_total_pulse = 0;                 // 起点总脉冲（用于B点减速）
int32_t dist_to_b_pulse = 0;                          // B点对应的脉冲数（动态计算）
#define DIST_TO_B_CM  47.0f                           // 起点到B点的距离（cm）
// ========== 参数初始化 ==========
void trackless_params_init(void)
{
    trackless_motor.left_encoder_dir_config = left_motor_encoder_dir_default;
    trackless_motor.right_encoder_dir_config = right_motor_encoder_dir_default;
    trackless_motor.left_motion_dir_config = left_motion_dir_default;
    trackless_motor.right_motion_dir_config = right_motion_dir_default;
    trackless_motor.wheel_radius_cm = tire_radius_cm_default;
    trackless_motor.pulse_num_per_circle = pulse_cnt_per_circle_default;

    speed_kp = 8.0f;
    speed_ki = 0.3f;
    speed_kd = 0.1f;
    turn_scale = 0.20f;
}

void ctrl_params_init(void)
{
    pid_control_init(&seektrack_ctrl[0], 14.0f, 0.0f, 0.0f, 20, 0, 500, 1, 0, 0, 6);
}

// ========== 200Hz 主中断 ==========
void maple_duty_200hz(void)
{
static float start_yaw = 0;
static float filtered_delta = 0;
static uint8_t started = 0;
static uint8_t first_sample = 1;   // 新增

if (!started) {
    start_yaw = smartcar_imu.rpy_deg[_YAW];
    filtered_delta = 0;
    started = 1;
    first_sample = 1;              // 新增
}

float delta_yaw = smartcar_imu.rpy_deg[_YAW] - start_yaw;
if (delta_yaw > 180.0f) delta_yaw -= 360.0f;
if (delta_yaw < -180.0f) delta_yaw += 360.0f;

if (first_sample) {
    filtered_delta = delta_yaw;    // 第一次直接赋值
    first_sample = 0;
} else {
    filtered_delta = 0.95f * filtered_delta + 0.05f * delta_yaw;
}
    // 软启动
    if (low_speed_timer > 0) {
        low_speed_timer--;
        if (low_speed_timer == 0) {
            speed_setup = 20.0f;
        }
    }

    // 传感器更新
    get_wheel_speed();
    imu_data_sampling();
    trackless_ahrs_update();

    // 获取转向输出
    float turn_val;
    gray_turn_control_200hz(&turn_val);

    // ========== 状态机 ==========
    if (turn_state == NORMAL) {
        // 丢线保持
        if (gray_state.state == 0x0000) {
            if (lost_timer == 0) {
                last_turn_output = turn_val;
                lost_timer = 40;
            }
            if (lost_timer > 0) {
                lost_timer--;
                turn_val = last_turn_output;
            }
        } else {
            lost_timer = 0;
        }

        // 动态速度与差速
        float error_abs = ABS(gray_status[0]);
        float current_speed;
        float scale;
        if (error_abs >= 6) {
            current_speed = 12.0f;
            scale = 0.30f;
        } else if (error_abs >= 3) {
            current_speed = 16.0f;
            scale = 0.25f;
        } else {
            current_speed = speed_setup;
            scale = turn_scale;
        }
        speed_expect[0] = current_speed + turn_val * scale;
        speed_expect[1] = current_speed - turn_val * scale;

        // ---------- B点暂停检测（仅当第一个直角弯完成后）----------
        if (first_turn_completed && pause_line_detected) {
            turn_state = PAUSE;
            // 立即停车
            speed_expect[0] = 0;
            speed_expect[1] = 0;
            speed_control_100hz(1);
            motor_output(1);
            beep.times = 3;    // 响3声
            beep.reset = 1;
            delay_ms(1500);     // 等待蜂鸣器响完
            // 暂停1秒后继续循迹（B点过后继续行驶）
            delay_ms(3000);
            // 清除标志，避免重复触发
            pause_line_detected = 0;
            // 继续前往C点，不再检测B点
            first_turn_completed = 0;   // 可选：之后不再进入暂停
            turn_state = NORMAL;
            return;   // 本次中断不再执行后续速度控制
        }

        // ---------- 编码器里程计辅助减速（接近B点时降低速度）----------
        if (!first_turn_completed) {
            int32_t now_pulse = (NEncoder.left_motor_total_cnt + NEncoder.right_motor_total_cnt) / 2;
            int32_t traveled = now_pulse - start_total_pulse;
            if (traveled > dist_to_b_pulse - 200) {   // 提前200脉冲开始减速
                // 限制最大速度
                if (speed_setup > 12.0f) speed_setup = 12.0f;
                if (current_speed > 12.0f) current_speed = 12.0f;
                // 重新赋值
                speed_expect[0] = current_speed + turn_val * scale;
                speed_expect[1] = current_speed - turn_val * scale;
            }
        }

        // ---------- 直角弯检测（全白触发）----------
        if (gray_state.state == 0x0000) {
            turn_state = DELAY_STOP;
            encoder_recorded = 0;
            // 清除丢线保持计时
            lost_timer = 0;
        }
    }
    else if (turn_state == DELAY_STOP) {
        // 编码器里程计控制前进固定距离
        if (!encoder_recorded) {
            start_encoder_pulse = (NEncoder.left_motor_total_cnt + NEncoder.right_motor_total_cnt) / 2;
            encoder_recorded = 1;
        }
        int32_t now_pulse = (NEncoder.left_motor_total_cnt + NEncoder.right_motor_total_cnt) / 2;
        int32_t traveled_pulse = now_pulse - start_encoder_pulse;

        if (traveled_pulse >= TARGET_PULSE) {
            turn_state = STOP_AND_TURN;
            encoder_recorded = 0;
            // 停车
            speed_expect[0] = 0;
            speed_expect[1] = 0;
            speed_control_100hz(1);
            motor_output(1);
            delay_ms(500);

            // 根据进入弯道前的方向设定旋转目标
            if (last_turn_output > 0) {
                target_angle = -85.0f;   // 右转
            } else {
                target_angle = 85.0f;    // 左转
            }
            relative_angle = 0;
        } else {
            // 直线前进（可用IMU轻微修正，可选）
            float straight_speed = 5.0f;
            // 简单航向保持（可选）
            static float start_yaw_delay = 0;
            static uint8_t yaw_recorded = 0;
            if (!yaw_recorded) {
                start_yaw_delay = smartcar_imu.rpy_deg[_YAW];
                yaw_recorded = 1;
            }
            float yaw_err = smartcar_imu.rpy_deg[_YAW] - start_yaw_delay;
            if (yaw_err > 180) yaw_err -= 360;
            if (yaw_err < -180) yaw_err += 360;
            float correction = yaw_err * 0.3f;
            correction = constrain_float(correction, -3.0f, 3.0f);
            speed_expect[0] = straight_speed - correction;
            speed_expect[1] = straight_speed + correction;
        }
    }
else if (turn_state == STOP_AND_TURN) {
    static float start_yaw = 0;
    static float filtered_delta = 0;
    static uint8_t started = 0;
    static uint8_t first_sample = 1;

    if (!started) {
        start_yaw = smartcar_imu.rpy_deg[_YAW];
        filtered_delta = 0;
        started = 1;
        first_sample = 1;
    }

    float delta_yaw = smartcar_imu.rpy_deg[_YAW] - start_yaw;
    if (delta_yaw > 180.0f) delta_yaw -= 360.0f;
    if (delta_yaw < -180.0f) delta_yaw += 360.0f;

    if (first_sample) {
        filtered_delta = delta_yaw;
        first_sample = 0;
    } else {
        filtered_delta = 0.95f * filtered_delta + 0.05f * delta_yaw;
    }

    float remain = fabs(target_angle - filtered_delta);
    if (remain <= 12.0f) {
        relative_angle = filtered_delta;
        turn_state = NORMAL;
        started = 0;

        speed_expect[0] = 0;
        speed_expect[1] = 0;
        speed_control_100hz(1);
        motor_output(1);
        delay_ms(300);

        lost_timer = 0;
        last_turn_output = 0;
        speed_integral[0] = 0;
        speed_integral[1] = 0;
        speed_output[0] = 0;
        speed_output[1] = 0;

        beep.period = 100;
        beep.light_on_percent = 0.5f;
        beep.reset = 1;
        beep.times = 1;

        first_turn_completed = 1;

        speed_expect[0] = 8.0f;
        speed_expect[1] = 8.0f;
        speed_control_100hz(1);
        motor_output(1);
        delay_ms(300);
        if (low_speed_timer == 0) speed_setup = 20.0f;
        else speed_setup = 5.0f;
    } else {
        int16_t turn_pwm;
        if (remain > 30.0f)      turn_pwm = (target_angle > 0) ? 25 : -25;
        else if (remain > 15.0f) turn_pwm = (target_angle > 0) ? 15 : -15;
        else                     turn_pwm = (target_angle > 0) ? 8 : -8;
        speed_expect[0] = -turn_pwm;
        speed_expect[1] = turn_pwm;
    }
}
    else if (turn_state == PAUSE) {
        // 已在进入时处理，这里保留空分支
        // 实际上不会执行到这里，因为进入PAUSE后已经delay并跳转
    }

    // 速度闭环与电机输出
    speed_control_100hz(1);
    motor_output(1);

    // 蜂鸣器、OLED显示（保留原有布局）
    laser_light_work(&beep);
    static uint16_t disp_cnt = 0;
    if (++disp_cnt >= 20) {
        disp_cnt = 0;
        float avg_speed = (NEncoder.left_motor_speed_cmps + NEncoder.right_motor_speed_cmps) * 0.5f;
        write_6_8_number(0, 0, avg_speed);
        write_6_8_number(70, 0, speed_setup);
    }
    static uint32_t last_show = 0;
    if (millis() - last_show > 200) {
        last_show = millis();
        char buf[17];
        float avg_spd = (NEncoder.left_motor_speed_cmps + NEncoder.right_motor_speed_cmps) * 0.5f;
        sprintf(buf, "Spd:%4.1f/%4.1f", avg_spd, speed_setup);
        LCD_P6x8Str(0, 0, (unsigned char*)buf);
        sprintf(buf, "Err:%5.1f", gray_status[0]);
        LCD_P6x8Str(0, 1, (unsigned char*)buf);
        const char* state_str;
        switch (turn_state) {
            case NORMAL:       state_str = "NORMAL   "; break;
            case DELAY_STOP:   state_str = "DELAY    "; break;
            case STOP_AND_TURN:state_str = "TURN     "; break;
            case PAUSE:        state_str = "PAUSE    "; break;
            default:           state_str = "UNKNOWN  "; break;
        }
        sprintf(buf, "State:%s", state_str);
        LCD_P6x8Str(0, 2, (unsigned char*)buf);
        sprintf(buf, "Ang:%5.1f/%4.1f", relative_angle, target_angle);
        LCD_P6x8Str(0, 3, (unsigned char*)buf);
        sprintf(buf, "GyroZ:%+5.1f", smartcar_imu.gyro_dps.z);
        LCD_P6x8Str(0, 4, (unsigned char*)buf);
        LCD_clear_L(0, 5);
    }
}

// ========== 定时器回调 ==========
void duty_100hz(void) {}
void duty_10hz(void) {}
void duty_1000hz(void)
{
    gpio_input_check_channel_12_2024();   // 更新灰度，同时更新 pause_line_detected
}

// ========== 主函数 ==========
int main(void)
{
    SYSCFG_DL_init();
    ncontroller_set_priority();
    OLED_Init();
    nGPIO_Init();
    ctrl_params_init();
    trackless_params_init();
    Encoder_Init();
    Button_Init();
    while (ICM206xx_Init());

    // 陀螺仪校准
    LCD_P6x8Str(0, 5, (unsigned char*)"Calib Gyro...");
    float gyro_z_sum = 0;
    uint16_t calib_samples = 200;
    for (int i = 0; i < calib_samples; i++) {
        imu_data_sampling();
        gyro_z_sum += smartcar_imu._gyro_dps_raw.z;
        delay_ms(10);
    }
    smartcar_imu.gyro_offset.z = gyro_z_sum / calib_samples;
    LCD_clear_L(0, 5);
    char buf[17];
    sprintf(buf, "Zoff:%.2f   ", smartcar_imu.gyro_offset.z);
    LCD_P6x8Str(0, 5, (unsigned char*)buf);
    delay_ms(1000);
    LCD_clear_L(0, 5);
    // 动态计算 B 点脉冲数（避免硬编码）
    dist_to_b_pulse = (int32_t)(DIST_TO_B_CM * (pulse_cnt_per_circle_default / (2.0f * 3.1415926f * tire_radius_cm_default)));
    timer_irq_config();
    PPM_Init();

    // 提示就绪
    beep.period = 100;
    beep.light_on_percent = 0.5f;
    beep.reset = 1;
    beep.times = 1;

    // 记录起点编码器总脉冲（用于B点减速）
    start_total_pulse = (NEncoder.left_motor_total_cnt + NEncoder.right_motor_total_cnt) / 2;

    while (1) {
        read_button_state_all();
        if (_button.state[UP].press == SHORT_PRESS ||
            (_button.state[ME_3D].press == SHORT_PRESS)) {
            start_flag = 1;
            speed_setup = 5.0f;
            low_speed_timer = 200;
            break;
        }
        delay_ms(10);
    }

    beep.period = 200;
    beep.light_on_percent = 0.5f;
    beep.reset = 1;
    beep.times = 3;
    delay_ms(1000);

    while (1) {
        delay_ms(100);
    }
}