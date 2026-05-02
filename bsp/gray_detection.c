#include "headfile.h"
#include "gray_detection.h"
#include "user.h"

#define steer_deadzone 15   // 转向控制死区
static uint16_t line_lost_timer = 0;      // 丢线计时器（用于安全超时，可选）
static const uint16_t LOST_TIMEOUT = 800; // 安全超时：4秒（800*5ms=4000ms），几乎不会触发
// 增量式PID相关变量
static float inc_pid_last_error = 0;      // 上一次偏差
static float inc_pid_last_output = 0;     // 上一次输出
static float inc_pid_kp = 18.0f;          // 比例系数（可调）
static float inc_pid_ki = 0.0f;           // 积分系数（小值，用于消除静差）
static float inc_pid_kd = 70.0f;          // 微分系数（可调）
_gray_state gray_state;
float gray_status[2] = {0}, gray_status_backup[2][20] = {0};
float turn_output = 0, turn_output_last = 0;
float turn_scale = turn_scale_default;
uint32_t gray_status_worse = 0;
controller seektrack_ctrl[2];
uint8_t pause_line_detected = 0;
// 外部变量声明（在 main.c 或 motor_control.c 中定义）
extern LOCK_STATE unlock_flag;
uint16_t last_valid_gray_state = 0;

#define read_gray_bit1   ((PORTA_PORT->DIN31_0 & PORTA_GRAY_BIT0_PIN ) ? 0x01 : 0x00)
#define read_gray_bit2   ((PORTA_PORT->DIN31_0 & PORTA_GRAY_BIT1_PIN ) ? 0x01 : 0x00)
#define read_gray_bit3   ((PORTA_PORT->DIN31_0 & PORTA_GRAY_BIT2_PIN ) ? 0x01 : 0x00)
#define read_gray_bit4   ((PORTA_PORT->DIN31_0 & PORTA_GRAY_BIT3_PIN ) ? 0x01 : 0x00)
#define read_gray_bit5   ((PORTA_PORT->DIN31_0 & PORTA_GRAY_BIT4_PIN ) ? 0x01 : 0x00)
#define read_gray_bit6   ((PORTA_PORT->DIN31_0 & PORTA_GRAY_BIT5_PIN ) ? 0x01 : 0x00)
#define read_gray_bit7   ((PORTB_PORT->DIN31_0 & PORTB_GRAY_BIT6_PIN ) ? 0x01 : 0x00)
#define read_gray_bit8   ((PORTB_PORT->DIN31_0 & PORTB_GRAY_BIT7_PIN ) ? 0x01 : 0x00)
#define read_gray_bit9   ((PORTB_PORT->DIN31_0 & PORTB_GRAY_BIT8_PIN ) ? 0x01 : 0x00)
#define read_gray_bit10  ((PORTB_PORT->DIN31_0 & PORTB_GRAY_BIT9_PIN ) ? 0x01 : 0x00)
#define read_gray_bit11  ((PORTA_PORT->DIN31_0 & PORTA_GRAY_BIT10_PIN) ? 0x01 : 0x00)
#define read_gray_bit12  ((PORTB_PORT->DIN31_0 & PORTB_GRAY_BIT11_PIN) ? 0x01 : 0x00)

// 12路灰度检测（用于循迹）
void gpio_input_check_channel_12_2024(void)
{
    gray_state.gray_bit[0] = read_gray_bit1;
    gray_state.gray_bit[1] = read_gray_bit2;
    gray_state.gray_bit[2] = read_gray_bit3;
    gray_state.gray_bit[3] = read_gray_bit4;
    gray_state.gray_bit[4] = read_gray_bit5;
    gray_state.gray_bit[5] = read_gray_bit6;
    gray_state.gray_bit[6] = read_gray_bit7;
    gray_state.gray_bit[7] = read_gray_bit8;
    gray_state.gray_bit[8] = read_gray_bit9;
    gray_state.gray_bit[9] = read_gray_bit10;
    gray_state.gray_bit[10] = read_gray_bit11;
    gray_state.gray_bit[11] = read_gray_bit12;
    gray_state.state = 0x0000;
    for (uint16_t i = 0; i < 12; i++)
    {
        gray_state.state |= gray_state.gray_bit[i] << i;
    }
		
    for (uint16_t i = 19; i > 0; i--)
    {
        gray_status_backup[0][i] = gray_status_backup[0][i - 1];
    }
    gray_status_backup[0][0] = gray_status[0];

    switch (gray_state.state)
    {
        case 0x0001: gray_status[0] = -11; break;
        case 0x0003: gray_status[0] = -10; break;
        case 0x0002: gray_status[0] = -9;  break;
        case 0x0007: gray_status[0] = -9;  break;
        case 0x0006: gray_status[0] = -8;  break;
        case 0x0004: gray_status[0] = -7;  break;
        case 0x000E: gray_status[0] = -7;  break;
        case 0x000C: gray_status[0] = -6;  break;
        case 0x0008: gray_status[0] = -5;  break;
        case 0x001C: gray_status[0] = -5;  break;
        case 0x0018: gray_status[0] = -4;  break;
        case 0x0010: gray_status[0] = -3;  break;
        case 0x0038: gray_status[0] = -3;  break;
        case 0x0030: gray_status[0] = -2;  break;
        case 0x0020: gray_status[0] = -1;  break;
        case 0x0070: gray_status[0] = -1;  break;
        case 0x0060: gray_status[0] =  0;  break;
        case 0x0040: gray_status[0] =  1;  break;
        case 0x00E0: gray_status[0] =  1;  break;
        case 0x00C0: gray_status[0] =  2;  break;
        case 0x0080: gray_status[0] =  3;  break;
        case 0x01C0: gray_status[0] =  3;  break;
        case 0x0180: gray_status[0] =  4;  break;
        case 0x0100: gray_status[0] =  5;  break;
        case 0x0380: gray_status[0] =  5;  break;
        case 0x0300: gray_status[0] =  6;  break;
        case 0x0200: gray_status[0] =  7;  break;
        case 0x0700: gray_status[0] =  7;  break;
        case 0x0600: gray_status[0] =  8;  break;
        case 0x0400: gray_status[0] =  9;  break;
        case 0x0E00: gray_status[0] =  9;  break;
        case 0x0C00: gray_status[0] = 10;  break;
        case 0x0800: gray_status[0] = 11;  break;
        case 0x0000:
            gray_status[0] = 0;
            break;
        default:
            break;
    }
    // 保存最近一次有效的灰度状态（用于急弯方向判断）
    static uint16_t last_valid_gray_state = 0;
    if (gray_state.state != 0x0000) 
    {
        last_valid_gray_state = gray_state.state;
    }
// 将 last_valid_gray_state 声明为可被 run_turn.c 引用的全局变量
// 简单起见，直接在 gray_detection.c 里定义，并在 run_turn.h 中声明外部引用。
// 在 gpio_input_check_channel_12_2024() 函数末尾，计算完 gray_status[0] 后添加：

    
}

// 增量式PID计算，返回增量 Δu
static float incremental_pid(float error, float kp, float ki, float kd)
{
    static float last_error = 0;
    static float last_last_error = 0;
    
    float delta = kp * (error - last_error) 
                + ki * error 
                + kd * (error - 2*last_error + last_last_error);
    
    last_last_error = last_error;
    last_error = error;
    
    return delta;
}

// 在文件开头添加静态变量（用于平滑滤波）
static float smooth_turn = 0;

// 在 gray_detection.c 中，使用动态 kp
void gray_turn_control_200hz(float *output)
{
    if (gray_status_worse >= 1000) unlock_flag = LOCK;
    turn_output_last = turn_output;
    
    if (gray_state.state == 0x0000) {
        // 丢线保持（不变）
    } else {
        float error = -gray_status[0];
        float error_abs = ABS(gray_status[0]);
        
        // 动态比例系数：偏差越大，kp 越大
        float kp;
        if (error_abs >= 5)      kp = 24.0f;   // 弯道强转向
        else                     kp = 14.0f;   // 直道弱转向
        
        float raw_output = kp * error;
        // 死区（减小）
        if (raw_output > 0) raw_output += 8;
        if (raw_output < 0) raw_output -= 8;
        raw_output = constrain_float(raw_output, -500, 500);
        
        // 平滑滤波（防止突变）
        static float smooth = 0;
        smooth = 0.6f * smooth + 0.4f * raw_output;
        turn_output = smooth;
    }
    *output = 0.8f * turn_output + 0.2f * turn_output_last;
}
// 以下函数已删除（视觉相关）：
// gpio_input_check_from_vision()
// vision_turn_control_50hz()
// gpio_input_check_channel_7()
// gpio_input_check_channel_12()
// gpio_input_check_channel_12_with_handle()
// 如需其他函数可自行添加