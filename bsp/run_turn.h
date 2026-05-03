/*======================== run_turn.h ========================*/
#ifndef __RUN_TURN_H
#define __RUN_TURN_H

#include "headfile.h"

/* 急弯状态枚举 */
typedef enum
{
    SHARP_DELAY,
    SHARP_WAIT,
    SHARP_TURN,
    SHARP_RELEASE   // 转向完成后的脱轨前进
} SharpTurnState;

/* 获取状态信息的接口 */
SharpTurnState get_sharp_turn_state(void);
float get_target_angle(void);
float get_relative_angle(void);
float get_current_delta_yaw(void);  // 获取当前转过的角度（实时显示用）
extern int32_t last_turn_finish_pulse;
#define PULSE_PER_CM  (pulse_cnt_per_circle_default / (2.0f * 3.1415926f * tire_radius_cm_default))
#define DECEL_THRESHOLD_PULSE ((int32_t)(65.0f * PULSE_PER_CM))
/* 转向计数接口 */
uint8_t get_white_state_count(void);
void reset_white_state_count(void);
void set_run_turn_enabled1(uint8_t enabled);

/* 主调度 */
void run_turn_logic1_200hz(void);

/* 功能拆分 */
void run_straight(void);      // 直线
void run_curve(void);         // 普通弯道
void run_sharp_turn(void);    // 急弯/直角弯
void stop1(void);
/* 初始化 */
void run_turn_init(void);
void mission1_show(void);
extern uint8_t mission1_complete ;       // 任务完成标志（四转停下）
static uint16_t stop_state_timer = 0; 
#endif