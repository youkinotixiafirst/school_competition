/*======================== run_turn.h ========================*/
#ifndef __RUN_TURN_H
#define __RUN_TURN_H

#include "headfile.h"

/* 急弯状态枚举 */
typedef enum
{
    SHARP_DELAY,
    SHARP_WAIT,
    SHARP_TURN
} SharpTurnState;

/* 获取状态信息的接口 */
SharpTurnState get_sharp_turn_state(void);
float get_target_angle(void);
float get_relative_angle(void);

/* 转向计数接口 */
uint8_t get_turn_complete_count(void);
void reset_turn_complete_count(void);
void set_run_turn_enabled(uint8_t enabled);

/* 主调度 */
void run_turn_logic_200hz(void);

/* 功能拆分 */
void run_straight(void);      // 直线
void run_curve(void);         // 普通弯道
void run_sharp_turn(void);    // 急弯/直角弯

/* 初始化 */
void run_turn_init(void);

#endif