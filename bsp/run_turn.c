/*======================== run_turn.c ========================*/
#include "run_turn.h"

/* 外部变量 */
extern float gray_status[2];
extern _gray_state gray_state;
extern float speed_setup;
extern float turn_scale;
extern float speed_expect[2];
extern float speed_integral[2];
extern float speed_output[2];

extern uint16_t low_speed_timer;
extern encoder NEncoder;
extern sensor smartcar_imu;
extern _laser_light beep;

/* 内部变量 */
static float last_turn_output = 0;
static uint16_t lost_timer = 0;

static int32_t start_encoder_pulse = 0;
static uint8_t encoder_recorded = 0;

static float target_angle = 0;
static uint32_t stop_start_time = 0;
static float relative_angle = 0;

/* 急弯状态变量 */
static SharpTurnState sharp_state = SHARP_DELAY;
static uint8_t turn_complete_count = 0;  // 转向完成次数计数
static uint8_t run_turn_enabled = 1;     // 是否启用转向逻辑（停机标志）

/* 参数 */
#define TARGET_DISTANCE_CM  4.0f
#define PULSE_PER_CM  (pulse_cnt_per_circle_default / (2.0f * 3.1415926f * tire_radius_cm_default))
#define TARGET_PULSE   ((int32_t)(TARGET_DISTANCE_CM * PULSE_PER_CM))


/*======================== 获取状态接口 ========================*/
SharpTurnState get_sharp_turn_state(void)
{
    return sharp_state;
}

float get_target_angle(void)
{
    return target_angle;
}

float get_relative_angle(void)
{
    return relative_angle;
}

uint8_t get_turn_complete_count(void)
{
    return turn_complete_count;
}

void reset_turn_complete_count(void)
{
    turn_complete_count = 0;
}

void set_run_turn_enabled(uint8_t enabled)
{
    run_turn_enabled = enabled;
}


/*======================== 初始化 ========================*/
void run_turn_init(void)
{
    lost_timer = 0;
    last_turn_output = 0;
}


/*======================== 直线 ========================*/
void run_straight(void)
{
    float turn_val;
    gray_turn_control_200hz(&turn_val);

    speed_expect[0] = speed_setup + turn_val * turn_scale;
    speed_expect[1] = speed_setup - turn_val * turn_scale;

    last_turn_output = turn_val;   // 保持转向方向记录，丢线时使用
}


/*======================== 普通弯道 ========================*/
void run_curve(void)
{
    float turn_val;
    gray_turn_control_200hz(&turn_val);

    // 丢线保持逻辑
    if (gray_state.state == 0x0000)
    {
        if (lost_timer == 0)
        {
            last_turn_output = turn_val;
            lost_timer = 40;
        }
        if (lost_timer > 0)
        {
            lost_timer--;
            turn_val = last_turn_output;
        }
    }
    else
    {
        lost_timer = 0;
    }

    float error_abs = ABS(gray_status[0]);

    float current_speed;
    float scale;

    if (error_abs >= 6)
    {
        current_speed = 12.0f;
        scale = 0.30f;
    }
    else
    {
        current_speed = 16.0f;
        scale = 0.25f;
    }

    speed_expect[0] = current_speed + turn_val * scale;
    speed_expect[1] = current_speed - turn_val * scale;

    last_turn_output = turn_val;   // 更新方向记录
}


/*======================== 急弯/直角弯 ========================*/
void run_sharp_turn(void)
{
    switch (sharp_state)
    {
        /*---------- 延迟前进 ----------*/
        case SHARP_DELAY:
        {
            static float start_yaw_delay = 0;
            static uint8_t yaw_recorded = 0;

            if (!encoder_recorded)
            {
                start_encoder_pulse =
                    (NEncoder.left_motor_total_cnt +
                     NEncoder.right_motor_total_cnt) / 2;
                encoder_recorded = 1;
                yaw_recorded = 0;   // 重置航向记录
            }

            int32_t now_pulse =
                (NEncoder.left_motor_total_cnt +
                 NEncoder.right_motor_total_cnt) / 2;
            int32_t traveled_pulse = now_pulse - start_encoder_pulse;

            if (traveled_pulse >= TARGET_PULSE)
            {
                // 前进距离足够，停车并准备转向
                speed_expect[0] = 0;
                speed_expect[1] = 0;

                stop_start_time = millis();

                if (last_turn_output > 0)
                    target_angle = -85.0f;
                else
                    target_angle = 85.0f;

                sharp_state = SHARP_WAIT;
                yaw_recorded = 0;   // 下一阶段不再需要
            }
            else
            {
                // 直线前进，带 IMU 航向保持
                float straight_speed = 5.0f;

                if (!yaw_recorded)
                {
                    start_yaw_delay = smartcar_imu.rpy_deg[_YAW];
                    yaw_recorded = 1;
                }

                float yaw_err = smartcar_imu.rpy_deg[_YAW] - start_yaw_delay;
                if (yaw_err > 180.0f) yaw_err -= 360.0f;
                if (yaw_err < -180.0f) yaw_err += 360.0f;

                float correction = yaw_err * 0.3f;
                correction = constrain_float(correction, -3.0f, 3.0f);

                speed_expect[0] = straight_speed - correction;
                speed_expect[1] = straight_speed + correction;
            }

            break;
        }

        /*---------- 停车等待 ----------*/
        case SHARP_WAIT:
        {
            speed_expect[0] = 0;
            speed_expect[1] = 0;

            if (millis() - stop_start_time >= 500)
            {
                sharp_state = SHARP_TURN;
            }

            break;
        }

        /*---------- 原地转向 ----------*/
        case SHARP_TURN:
        {
            static float start_yaw = 0;
            static float filtered_delta = 0;
            static uint8_t started = 0;
            static uint8_t first_sample = 1;

            if (!started)
            {
                start_yaw = smartcar_imu.rpy_deg[_YAW];
                filtered_delta = 0;
                started = 1;
                first_sample = 1;
            }

            float delta_yaw =
                smartcar_imu.rpy_deg[_YAW] - start_yaw;
            if (delta_yaw > 180.0f) delta_yaw -= 360.0f;
            if (delta_yaw < -180.0f) delta_yaw += 360.0f;

            if (first_sample)
            {
                filtered_delta = delta_yaw;
                first_sample = 0;
            }
            else
            {
                filtered_delta =
                    0.95f * filtered_delta + 0.05f * delta_yaw;
            }

            float remain = fabs(target_angle - filtered_delta);

            if (remain <= 12.0f)
            {
                // 转向完成，停车
                speed_expect[0] = 0;
                speed_expect[1] = 0;

                relative_angle = filtered_delta;
                started = 0;
                encoder_recorded = 0;
                sharp_state = SHARP_DELAY;
                turn_complete_count++;          // 一次转向完成计数

                // 重置各种状态
                lost_timer = 0;
                last_turn_output = 0;
                speed_integral[0] = 0;
                speed_integral[1] = 0;
                speed_output[0] = 0;
                speed_output[1] = 0;

                // 蜂鸣器提示
                beep.period = 100;
                beep.light_on_percent = 0.5f;
                beep.reset = 1;
                beep.times = 1;

                // 短暂直线前进，帮助脱轨
                speed_expect[0] = 8.0f;
                speed_expect[1] = 8.0f;
                speed_control_100hz(1);
                motor_output(1);
                delay_ms(300);

                // 恢复速度设定
                if (low_speed_timer == 0)
                    speed_setup = 20.0f;
                else
                    speed_setup = 5.0f;

                low_speed_timer = 60;    // 软启动计时
            }
            else
            {
                // 原地差速转向
                int16_t turn_pwm;
                if (remain > 30.0f)
                    turn_pwm = (target_angle > 0) ? 25 : -25;
                else if (remain > 15.0f)
                    turn_pwm = (target_angle > 0) ? 15 : -15;
                else
                    turn_pwm = (target_angle > 0) ? 8 : -8;

                speed_expect[0] = -turn_pwm;
                speed_expect[1] = turn_pwm;
            }

            break;
        }

        default:
            sharp_state = SHARP_DELAY;
            break;
    }
}


/*======================== 总调度 ========================*/
void run_turn_logic_200hz(void)
{
    /* 如果禁用转向逻辑，直接返回 */
    if (!run_turn_enabled)
    {
        return;
    }

    float error_abs = ABS(gray_status[0]);

    /* 全白：进入急弯 */
    if (gray_state.state == 0x0000)
    {
        // 只在急弯刚开始时记录转向方向，避免在急弯过程中被覆盖
        if (sharp_state == SHARP_DELAY && !encoder_recorded)
        {
            // 此时还在正常巡线，获取一次转向值作为方向记录
            float turn_val;
            gray_turn_control_200hz(&turn_val);
            last_turn_output = turn_val;
        }

        lost_timer = 0;   // 进入急弯后停止丢线计时
        run_sharp_turn();
        return;
    }

    /* 小误差：直线 */
    if (error_abs < 3)
    {
        run_straight();
    }
    /* 中误差：普通弯 */
    else
    {
        run_curve();
    }
}