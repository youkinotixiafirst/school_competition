/*======================== run_turn.h ========================*/
#ifndef __MISSION3_H
#define __MISSION3_H

#include "headfile.h"

/* 急弯状态枚举 */
typedef enum
{
    SHARP_DELAY3,
    SHARP_WAIT3,
    SHARP_TURN3,
    SHARP_RELEASE3   // 转向完成后的脱轨前进
} SharpTurnState3;

/* 获取状态信息的接口 */
SharpTurnState3 get_sharp_turn_state3(void);
float get_target_angle3(void);
float get_relative_angle3(void);
float get_current_delta_yaw3(void);  // 获取当前转过的角度（实时显示用）
extern int32_t last_turn_finish_pulse3;
#define PULSE_PER_CM  (pulse_cnt_per_circle_default / (2.0f * 3.1415926f * tire_radius_cm_default))
#define DECEL_THRESHOLD_PULSE ((int32_t)(65.0f * PULSE_PER_CM))
/* 转向计数接口 */
uint8_t get_white_state_count3(void);
void reset_white_state_count3(void);
void set_run_turn_enabled3(uint8_t enabled);

/* 主调度 */
void run_turn_logic3_200hz(void);

/* 功能拆分 */
void run_straight3(void);      // 直线
void run_curve3(void);         // 普通弯道
void run_sharp_turn3(void);    // 急弯/直角弯

/* 初始化 */
void run_turn_init3(void);
void mission3_show(void);
extern uint8_t mission3_complete ;       // 任务完成标志（四转停下）
static uint16_t stop_state_timer3 = 0; 
void stop3 (void);
#endif