/*======================== run_turn.c ========================*/
#include "run_turn.h"
/* 参数 */
#define TARGET_DISTANCE_CM1  14.0f
#define TARGET_DISTANCE_CM2  4.0f
#define PULSE_PER_CM  (pulse_cnt_per_circle_default / (2.0f * 3.1415926f * tire_radius_cm_default))
#define TARGET_PULSE1   ((int32_t)(TARGET_DISTANCE_CM1 * PULSE_PER_CM))
#define TARGET_PULSE2   ((int32_t)(TARGET_DISTANCE_CM2 * PULSE_PER_CM))
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
 float current_delta_yaw ;  // 当前转过的角度（实时显示用）

/* 急弯状态变量 */
static SharpTurnState sharp_state = SHARP_DELAY;
static uint8_t white_state_count = 0;  // 转向完成次数计数
static uint8_t run_turn_enabled1 = 1;     // 是否启用转向逻辑（停机标志）
static uint16_t stabilize_timer = 0;     // 脱轨稳定缓冲计时（200Hz计数）
int32_t last_turn_finish_pulse = 0; 
uint8_t mission1_complete = 0; 
static float global_turn_dir = 0;
/*======================== 获取状态接口 ========================*/
SharpTurnState get_sharp_turn_state(void)
{
    return sharp_state;
}

float get_target_angle(void)
{
    return target_angle;
}



uint8_t get_white_state_count(void)
{
    return white_state_count;
}

float get_current_delta_yaw(void)
{
    return current_delta_yaw;
}

void reset_white_state_count(void)
{
    white_state_count = 0;
}

void set_run_turn_enabled1(uint8_t enabled)
{
    run_turn_enabled1 = enabled;
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

    // 陀螺仪阻尼：用Z轴角速度抵抗偏航趋势
    float gyro_z = smartcar_imu.gyro_dps.z;
    float gyro_damping = -gyro_z * 0.25f;
    gyro_damping = constrain_float(gyro_damping, -5.0f, 5.0f);

    speed_expect[0] = speed_setup + turn_val * turn_scale ;
    speed_expect[1] = speed_setup - turn_val * turn_scale ;

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
        current_speed = 40.0f;
        scale = 0.15f;
    }
    else
    {
        current_speed = 40.0f;
			
        scale = 0.10f;
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
						
						
						if (white_state_count == 0)
						{						
            if (traveled_pulse >= TARGET_PULSE1)
            {
                // 前进距离足够，停车并准备转向
                speed_expect[0] = 0;
                speed_expect[1] = 0;
								speed_integral[0] = 0;
							  speed_integral[1] = 0;
								speed_output[0] = 0;
                speed_output[1] = 0;							
                stop_start_time = millis();
								if (last_turn_output > 0) 
								{
										global_turn_dir = 1.0f;  // 全局锁定为右转
										target_angle = -90.0f;
								} 
								else 
								{
										global_turn_dir = -1.0f; // 全局锁定为左转
										target_angle = 90.0f;
							  }
                sharp_state = SHARP_WAIT;
                yaw_recorded = 0;   // 下一阶段不再需要
            }
            else
            {
                // 直线前进，带 IMU 航向保持
                float straight_speed = 10.0f;

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
					  }
						else
						{
            if (traveled_pulse >= TARGET_PULSE2)
            {
                // 前进距离足够，停车并准备转向
                speed_expect[0] = 0;
                speed_expect[1] = 0;
                if (global_turn_dir > 0) 
                target_angle = -90.0f; // 或 180.0f，根据你陀螺仪的偏好
                else 
                target_angle = 90.0f;
                stop_start_time = millis();

                sharp_state = SHARP_WAIT;
                yaw_recorded = 0;   // 下一阶段不再需要
            }
            else
            {
                // 直线前进，带 IMU 航向保持
                float straight_speed = 10.0f;

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
						}
            break;
        }
			

        /*---------- 停车等待 ----------*/
        case SHARP_WAIT:
        {
            speed_expect[0] = 0;
            speed_expect[1] = 0;

            if (white_state_count == 0)
						{							
								if (millis() - stop_start_time >= 500)
								{
										sharp_state = SHARP_TURN;
									  white_state_count++;
								}
						}
						else
						{
								if (millis() - stop_start_time >= 30)
								{
										sharp_state = SHARP_TURN;
									  white_state_count++;
								}
						}
            break;
        }

        /*---------- 原地转向（PD闭环） ----------*/
        case SHARP_TURN:
        {
            static float start_yaw = 0;
            static uint8_t started = 0;

            if (!started)
            {
                start_yaw = smartcar_imu.rpy_deg[_YAW];
                started = 1;
            }

            float delta_yaw =
                smartcar_imu.rpy_deg[_YAW] - start_yaw;
            if (delta_yaw > 180.0f) delta_yaw -= 360.0f;
            if (delta_yaw < -180.0f) delta_yaw += 360.0f;
                current_delta_yaw = delta_yaw;
            float yaw_err = target_angle - delta_yaw;
            if (yaw_err > 180.0f) yaw_err -= 360.0f;
            if (yaw_err < -180.0f) yaw_err += 360.0f;

            float remain = fabs(yaw_err);

            if (remain <= 5.0f)
            {
                // 转向完成，进入脱轨前进阶段
                speed_expect[0] = 0;
                speed_expect[1] = 0;


                started = 0;
                encoder_recorded = 0;
                          // 一次转向完成计数

                // 重置各种状态
                lost_timer = 0;
                // last_turn_output 不重置，保持前一个有效值
                speed_integral[0] = 0;
                speed_integral[1] = 0;
                speed_output[0] = 0;
                speed_output[1] = 0;

                // 蜂鸣器提示
                beep.period = 100;
                beep.light_on_percent = 0.5f;
                beep.reset = 1;
                beep.times = 1;

                // 进入脱轨前进阶段
                stop_start_time = millis();
                sharp_state = SHARP_RELEASE;
            }
            else
            {
                // PD闭环控制：P=角度误差，D=角速度阻尼
                float p_out = yaw_err * 1.4f;
                float d_out = -smartcar_imu.gyro_dps.z * 0.2f;
                float turn_pwm = p_out + d_out;
                turn_pwm = constrain_float(turn_pwm, -100.0f, 100.0f);

                speed_expect[0] = -turn_pwm;
                speed_expect[1] = turn_pwm;
            }

            break;
        }

        /*---------- 脱轨前进 ----------*/
        case SHARP_RELEASE:
        {
            // 短暂直线前进，帮助脱轨（300ms）
            speed_expect[0] = 8.0f;
            speed_expect[1] = 8.0f;

            if (millis() - stop_start_time >= 30)
            {
                // 【新增】此时转弯彻底结束，记录此时的编码器平均值，作为下一段直道的起点
                last_turn_finish_pulse = (NEncoder.left_motor_total_cnt + NEncoder.right_motor_total_cnt) / 2; 							
                // 300ms后进入稳定缓冲期（防止巡线算法猛转）
                sharp_state = SHARP_DELAY;
                stabilize_timer = 20;  // 缓冲200ms（20×10ms）
                last_turn_output = 0;   // 重置转向记录，进入正常巡线
                if (low_speed_timer == 0)
                    speed_setup = 60.0f;
                else
                    speed_setup = 10.0f;

                low_speed_timer = 60;    // 软启动计时
            }

            break;
        }

        default:
            sharp_state = SHARP_DELAY;
            break;
    }
}


/*======================== 总调度 ========================*/
void run_turn_logic1_200hz(void)
{
    /* 如果禁用转向逻辑，直接返回 */
    if (!run_turn_enabled1)
    {
        return;
    }

    float error_abs = ABS(gray_status[0]);

    /* 全白或正在转弯过程中：进入/继续急弯（禁用巡线） */
    if (gray_state.state == 0x0000 || encoder_recorded)
    {
        // 只在急弯刚开始时记录转向方向，避免在急弯过程中被覆盖
        if (sharp_state == SHARP_DELAY && !encoder_recorded)
        {
            uint16_t pattern = last_valid_gray_state;
    
            // 左侧传感器变黑（高位有1）→ 黑线在左边 → 车需左转
            if (pattern & 0x0F00)   // bit11 ~ bit8 或更宽，可根据传感器布局缩小范围
            {
                last_turn_output = -1.0f;   // 左转（对应 target_angle = -90°）
            }
            // 右侧传感器变黑（低位有1）→ 黑线在右边 → 车需右转
            else if (pattern & 0x000F)  // bit7 ~ bit0
            {
                last_turn_output = 1.0f;    // 右转（对应 target_angle = 90°）
            }
        }

        stabilize_timer = 0;  // 检测到全白，重置稳定缓冲
        run_sharp_turn();
        return;
    }

    /* 脱轨稳定缓冲期内，保持直线前进，不执行巡线 */
    if (stabilize_timer > 0)
    {
        stabilize_timer--;
        speed_expect[0] = speed_setup;
        speed_expect[1] = speed_setup;
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
void stop1 (void)
{
		uint8_t turn_count = get_white_state_count();

		if (turn_count >= 5 && mission1_complete == 0)
		{
				mission1_complete = 1;      // 进入第一阶段：主动刹车/倒车
				set_run_turn_enabled1(0);   // 禁用巡线转向逻辑
				stop_state_timer = 0;      // 清零计时器

				// 【关键优化】瞬间清空之前的速度积分，防止PID的I项（积分）"顽固"地让车往前冲
				speed_integral[0] = 0;
				speed_integral[1] = 0;
		}

// 倒车与停车状态机（不阻塞主循环）
		if (mission1_complete == 1)
		{
				stop_state_timer++;
				// 200Hz下，1个周期5ms。设置 60个周期 = 300ms 的倒车时间（可根据实际测试稍微增减）
				if (stop_state_timer <= 60) 
				{
						speed_expect[0] = -50.0f;  // 给一个较小的负向期望，让车轮反转制动
						speed_expect[1] = -50.0f;
				}
				else
				{
						mission1_complete = 2; // 倒车结束，进入彻底停止阶段
				}
		}
		else if (mission1_complete == 2)
		{		
				// 彻底停车锁死
				speed_expect[0] = 0;
				speed_expect[1] = 0;
    
				// 直接把输出抹零，切断动力
				speed_output[0] = 0;
				speed_output[1] = 0;
				speed_setup = 0;
		}
    else if (mission1_complete == 0) 
    {
        // 1. 获取当前平均脉冲位置
        int32_t current_avg_pulse = (NEncoder.left_motor_total_cnt + NEncoder.right_motor_total_cnt) / 2;
        
        // 2. 计算距离上一次转弯结束跑了多少脉冲
        int32_t distance_pulse = current_avg_pulse - last_turn_finish_pulse;

        // 3. 距离判断：跑够65cm立刻降速备战直角弯
				if (low_speed_timer == 0) 
				{
						if (distance_pulse >= DECEL_THRESHOLD_PULSE)
								speed_setup = 40.0f;
						else
								speed_setup = 60.0f;
				}
    }
}
void mission1_show (void)
{
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

        float relative = get_current_delta_yaw();  // 显示当前转过的角度
        float target = get_target_angle();
        sprintf(buf, "Ang:%5.1f/%4.1f", relative, target);
        LCD_P6x8Str(0, 3, (unsigned char*)buf);

        sprintf(buf, "GyroZ:%+5.1f", smartcar_imu.gyro_dps.z);
        LCD_P6x8Str(0, 4, (unsigned char*)buf);
        
        // 可选：显示转向计数
        sprintf(buf, "Turn:%d", get_white_state_count());
        LCD_P6x8Str(0, 5, (unsigned char*)buf);
    }

}