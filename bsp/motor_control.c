#include "headfile.h"
#include "motor_control.h"
#include "nqei.h"      // 添加编码器头文件
#include "user.h"
extern encoder NEncoder;        // 编码器结构体
extern LOCK_STATE unlock_flag;  // 解锁标志

float left_pwm, right_pwm;

motor_config trackless_motor = {
    .left_encoder_dir_config = left_motor_encoder_dir_default,
    .right_encoder_dir_config = right_motor_encoder_dir_default,
    .left_motion_dir_config = left_motion_dir_default,
    .right_motion_dir_config = right_motion_dir_default,
    .wheel_radius_cm = tire_radius_cm_default,
    .pulse_num_per_circle = pulse_cnt_per_circle_default,
    .servo_median_value1 = servo_median_value1_default,
    .servo_median_value2 = servo_median_value2_default,
};

uint8_t speed_ctrl_mode = 1;   // 1:两轮单独控制
uint8_t speed_pid_mode = 0;     // 位置式PID

#define speed_err_max          50.0f
#define speed_integral_max    600.0f
#define speed_ctrl_output_max 999

float speed_setup = 20;
float speed_kp = speed_kp_default, speed_ki = speed_ki_default, speed_kd = speed_kd_default;
float speed_error[2] = {0}, speed_expect[2] = {speed_expect_default, speed_expect_default};
float speed_feedback[2] = {0};
float speed_integral[2] = {0};
float speed_output[2] = {0};

// 速度控制（100Hz，只保留普通两轮控制）
void speed_control_100hz(uint8_t _speed_ctrl_mode)
{
    static uint16_t _cnt = 0;
    if (_speed_ctrl_mode != 1) return;   // 只处理模式1

    _cnt++;
    if (_cnt < 2) return;
    _cnt = 0;   // 10ms执行一次

    if (speed_pid_mode == 0)   // 位置式PID
    {
        // 左轮
        speed_feedback[0] = NEncoder.left_motor_speed_cmps;
        speed_error[0] = speed_expect[0] - speed_feedback[0];
        speed_error[0] = constrain_float(speed_error[0], -speed_err_max, speed_err_max);
        speed_integral[0] += speed_ki * speed_error[0];
        speed_integral[0] = constrain_float(speed_integral[0], -speed_integral_max, speed_integral_max);
        speed_output[0] = speed_integral[0] + speed_kp * speed_error[0];
        speed_output[0] = constrain_float(speed_output[0], -speed_ctrl_output_max, speed_ctrl_output_max);

        // 右轮
        speed_feedback[1] = NEncoder.right_motor_speed_cmps;
        speed_error[1] = speed_expect[1] - speed_feedback[1];
        speed_error[1] = constrain_float(speed_error[1], -speed_err_max, speed_err_max);
        speed_integral[1] += speed_ki * speed_error[1];
        speed_integral[1] = constrain_float(speed_integral[1], -speed_integral_max, speed_integral_max);
        speed_output[1] = speed_integral[1] + speed_kp * speed_error[1];
        speed_output[1] = constrain_float(speed_output[1], -speed_ctrl_output_max, speed_ctrl_output_max);
    }
    // 增量式PID部分可删除，此处省略
}

// PWM 通道定义
#define right_pwm_channel_1  GPIO_PWM_0_C3_IDX
#define right_pwm_channel_2  GPIO_PWM_0_C2_IDX
#define left_pwm_channel_1   GPIO_PWM_0_C1_IDX
#define left_pwm_channel_2   GPIO_PWM_0_C0_IDX

// 最终电机输出
void motor_output(uint8_t _speed_ctrl_mode)
{
    if (_speed_ctrl_mode == 0)
    {
        left_pwm = 0;
        right_pwm = 0;
    }
    else if (_speed_ctrl_mode == 1)
    {
        right_pwm = speed_output[1];
        left_pwm  = speed_output[0];
    }
    else
    {
        left_pwm = 0;
        right_pwm = 0;
    }

    if (unlock_flag == LOCK)
    {
        left_pwm = 0;
        right_pwm = 0;
    }

    left_pwm  = constrain_float(left_pwm, -999, 999);
    right_pwm = constrain_float(right_pwm, -999, 999);

    // 右边电机输出
    if (trackless_motor.right_motion_dir_config == 0)
    {
        if (right_pwm >= 0)
        {
            DL_TimerA_setCaptureCompareValue(PWM_0_INST, 0,         right_pwm_channel_1);
            DL_TimerA_setCaptureCompareValue(PWM_0_INST, right_pwm, right_pwm_channel_2);
        }
        else
        {
            DL_TimerA_setCaptureCompareValue(PWM_0_INST, ABS(right_pwm), right_pwm_channel_1);
            DL_TimerA_setCaptureCompareValue(PWM_0_INST, 0,              right_pwm_channel_2);
        }
    }
    else
    {
        if (right_pwm >= 0)
        {
            DL_TimerA_setCaptureCompareValue(PWM_0_INST, right_pwm, right_pwm_channel_1);
            DL_TimerA_setCaptureCompareValue(PWM_0_INST, 0,         right_pwm_channel_2);
        }
        else
        {
            DL_TimerA_setCaptureCompareValue(PWM_0_INST, 0,              right_pwm_channel_1);
            DL_TimerA_setCaptureCompareValue(PWM_0_INST, ABS(right_pwm), right_pwm_channel_2);
        }
    }

    // 左边电机输出
    if (trackless_motor.left_motion_dir_config == 0)
    {
        if (left_pwm >= 0)
        {
            DL_TimerA_setCaptureCompareValue(PWM_0_INST, left_pwm, left_pwm_channel_1);
            DL_TimerA_setCaptureCompareValue(PWM_0_INST, 0,        left_pwm_channel_2);
        }
        else
        {
            DL_TimerA_setCaptureCompareValue(PWM_0_INST, 0,             left_pwm_channel_1);
            DL_TimerA_setCaptureCompareValue(PWM_0_INST, ABS(left_pwm), left_pwm_channel_2);
        }
    }
    else
    {
        if (left_pwm >= 0)
        {
            DL_TimerA_setCaptureCompareValue(PWM_0_INST, 0,        left_pwm_channel_1);
            DL_TimerA_setCaptureCompareValue(PWM_0_INST, left_pwm, left_pwm_channel_2);
        }
        else
        {
            DL_TimerA_setCaptureCompareValue(PWM_0_INST, ABS(left_pwm), left_pwm_channel_1);
            DL_TimerA_setCaptureCompareValue(PWM_0_INST, 0,             left_pwm_channel_2);
        }
    }
}