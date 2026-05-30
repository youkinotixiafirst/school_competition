/* ======================== mission1.h ========================
 * 基础部分第一问: A→B 直线循迹 100cm, 编码器测距停止
 * ========================================================== */

#ifndef __MISSION1_H
#define __MISSION1_H

#include "headfile.h"

#define TOTAL_DISTANCE_CM   100.0f  /* A→B 距离                        */
#define CRUISE_SPEED        20.0f   /* 巡航速度 cm/s                   */

void run_turn_logic1_200hz(void);
void mission1_show(void);
void stop1(void);

#endif
