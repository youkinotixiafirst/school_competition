/*======================== run_turn.h ========================*/
#ifndef __MISSION2_H
#define __MISSION2_H

#include "headfile.h"

/* 急弯状态枚举 */
typedef enum
{
    SHARP_DELAY2,
    SHARP_WAIT2,
    SHARP_TURN2,
    SHARP_RELEASE2   // 转向完成后的脱轨前进
} SharpTurnState2;

/* 获取状态信息的接口 */
SharpTurnState2 get_sharp_turn_state2(void);
float get_target_angle2(void);
float get_relative_angle2(void);
float get_current_delta_yaw2(void);  // 获取当前转过的角度（实时显示用）
extern int32_t last_turn_finish_pulse2;
#define PULSE_PER_CM  (pulse_cnt_per_circle_default / (2.0f * 3.1415926f * tire_radius_cm_default))
#define DECEL_THRESHOLD_PULSE ((int32_t)(65.0f * PULSE_PER_CM))
/* 转向计数接口 */
uint8_t get_white_state_count2(void);
void reset_white_state_count2(void);
void set_run_turn_enabled2(uint8_t enabled);

/* 主调度 */
void run_turn_logic2_200hz(void);

/* 功能拆分 */
void run_straight2(void);      // 直线
void run_curve2(void);         // 普通弯道
void run_sharp_turn2(void);    // 急弯/直角弯

/* 初始化 */
void run_turn_init2(void);
void mission2_show(void);
extern uint8_t mission2_complete ;       // 任务完成标志（四转停下）
static uint16_t stop_state_timer2 = 0; 
void stop2 (void);
#endif