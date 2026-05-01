#include "ti_msp_dl_config.h"
#include "headfile.h"
#include "run_turn.h"
// ========== 外部变量引用 ==========
extern motor_config trackless_motor;
extern float turn_scale;
extern float speed_kp, speed_ki, speed_kd;
extern float speed_setup;
extern float gray_status[2];
extern _gray_state gray_state;
extern LOCK_STATE unlock_flag;

LOCK_STATE unlock_flag = UNLOCK;
volatile uint8_t start_flag = 0;

uint16_t low_speed_timer = 0;      // 软启动计时（5ms单位）
uint8_t mission_complete = 0;       // 任务完成标志（四转停下）
// ========== 参数初始化 ==========
void trackless_params_init(void)
{
    trackless_motor.left_encoder_dir_config = left_motor_encoder_dir_default;
    trackless_motor.right_encoder_dir_config = right_motor_encoder_dir_default;
    trackless_motor.left_motion_dir_config = left_motion_dir_default;
    trackless_motor.right_motion_dir_config = right_motion_dir_default;
    trackless_motor.wheel_radius_cm = tire_radius_cm_default;
    trackless_motor.pulse_num_per_circle = pulse_cnt_per_circle_default;

    speed_kp = 6.0f;
    speed_ki = 0.2f;
    speed_kd = 0.08f;
    turn_scale = 0.08f;
}

void ctrl_params_init(void)
{
    pid_control_init(&seektrack_ctrl[0], 14.0f, 0.0f, 0.0f, 20, 0, 500, 1, 0, 0, 6);
}

// ========== 200Hz 主中断 ==========

void maple_duty_200hz(void)
{
    // 未启动时不执行
    if (!start_flag)
    {
        return;
    }

    // 任务完成判断：转向4次后停机
    uint8_t turn_count = get_turn_complete_count();
    if (turn_count >= 4 && !mission_complete)
    {
        mission_complete = 1;
        speed_setup = 0;
        set_run_turn_enabled(0);  // 禁用转向逻辑
    }

    // 软启动
    if (low_speed_timer > 0) 
    {
        low_speed_timer--;
        if (low_speed_timer == 0) 
        {
            speed_setup = 50.0f;
        }
    }

    // 传感器更新
    get_wheel_speed();
    imu_data_sampling();
    trackless_ahrs_update();

    // 转向逻辑（已拆分到run_turn.c）
    run_turn_logic_200hz();

    speed_control_100hz(1);
    motor_output(1);

    // 蜂鸣器、OLED显示（保留原有布局）
    laser_light_work(&beep);
    static uint16_t disp_cnt = 0;
    if (++disp_cnt >= 20) 
    {
        disp_cnt = 0;
        float avg_speed = (NEncoder.left_motor_speed_cmps + NEncoder.right_motor_speed_cmps) * 0.5f;
        write_6_8_number(0, 0, avg_speed);
        write_6_8_number(70, 0, speed_setup);
    }
    // OLED 显示（200ms 一次）
    static uint32_t last_show = 0;
    if (millis() - last_show > 200)
    {
        last_show = millis();
        char buf[17];
        float avg_spd = (NEncoder.left_motor_speed_cmps + NEncoder.right_motor_speed_cmps) * 0.5f;
        sprintf(buf, "Spd:%4.1f/%4.1f", avg_spd, speed_setup);
        LCD_P6x8Str(0, 0, (unsigned char*)buf);
        sprintf(buf, "Err:%5.1f", gray_status[0]);
        LCD_P6x8Str(0, 1, (unsigned char*)buf);

        // 使用 run_turn 接口获取状态
        const char* state_str;
        SharpTurnState turn_state = get_sharp_turn_state();
        switch (turn_state)
        {
            case SHARP_DELAY:  state_str = "DELAY    "; break;
            case SHARP_WAIT:   state_str = "WAIT     "; break;
            case SHARP_TURN:   state_str = "TURN     "; break;
            default:           state_str = "NORMAL   "; break;
        }
        sprintf(buf, "State:%s", state_str);
        LCD_P6x8Str(0, 2, (unsigned char*)buf);

        float relative = get_relative_angle();
        float target = get_target_angle();
        sprintf(buf, "Ang:%5.1f/%4.1f", relative, target);
        LCD_P6x8Str(0, 3, (unsigned char*)buf);

        sprintf(buf, "GyroZ:%+5.1f", smartcar_imu.gyro_dps.z);
        LCD_P6x8Str(0, 4, (unsigned char*)buf);
        
        // 可选：显示转向计数
        sprintf(buf, "Turn:%d", get_turn_complete_count());
        LCD_P6x8Str(0, 5, (unsigned char*)buf);
    }
}

// ========== 定时器回调 ==========

void duty_1000hz(void)
{
    gpio_input_check_channel_12_2024();   // 更新灰度
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
    timer_irq_config();
    PPM_Init();

    // 提示就绪
    beep.period = 100;
    beep.light_on_percent = 0.5f;
    beep.reset = 1;
    beep.times = 1;

    while (1) {
        read_button_state_all();
        if (_button.state[UP].press == SHORT_PRESS ||
            (_button.state[ME_3D].press == SHORT_PRESS)) {
            start_flag = 1;
						speed_integral[0] = 0;
						speed_integral[1] = 0;
						speed_output[0] = 0;
						speed_output[1] = 0;
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