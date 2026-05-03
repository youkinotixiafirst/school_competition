#include "ti_msp_dl_config.h"
#include "headfile.h"
#include "run_turn.h"
#include "mission2.h"
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
uint8_t mission_mode = 0; 
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


    // 任务完成判断
		
    // 软启动
		if (low_speed_timer > 0) 
    {
        low_speed_timer--;
        if (low_speed_timer == 0) 
        {
            speed_setup = 60.0f;
        }
    }

    // 传感器更新
    get_wheel_speed();
    imu_data_sampling();
    trackless_ahrs_update();

    // 转向逻辑（已拆分到run_turn.c）
		if (mission_mode == 1)
    {			  
        run_turn_logic1_200hz();
			  mission1_show();
			  stop1();
    }  
 
   
		if (mission_mode == 2)
    {
				run_turn_logic2_200hz();  
				mission2_show();
			  stop2();   
    }  

    speed_control_100hz(1);
    motor_output(1);

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

    while (1) 
		{
        read_button_state_all();
        if (_button.state[UP].press == SHORT_PRESS ) 
				{
            start_flag = 1;
					  mission_mode = 1;
						speed_integral[0] = 0;
						speed_integral[1] = 0;
						speed_output[0] = 0;
						speed_output[1] = 0;
            speed_setup = 10.0f;
            low_speed_timer=200;
\
            last_turn_finish_pulse = (NEncoder.left_motor_total_cnt + NEncoder.right_motor_total_cnt) / 2;
							
            break;
        }
        
    
		    else if (_button.state[DOWN].press == SHORT_PRESS) 
				{
            start_flag = 1;
					  mission_mode = 2;
						speed_integral[0] = 0;
						speed_integral[1] = 0;
						speed_output[0] = 0;
						speed_output[1] = 0;
            speed_setup = 10.0f;
            low_speed_timer=200;
					
					  last_turn_finish_pulse = (NEncoder.left_motor_total_cnt + NEncoder.right_motor_total_cnt) / 2;
					
            break;
        }
				delay_ms(10);
    }
		
    beep.period = 200;
    beep.light_on_percent = 0.5f;
    beep.reset = 1;
    beep.times = 3;
    //delay_ms(1000);

    while (1) {
        delay_ms(10);
    }
}