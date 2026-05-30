/* ======================== mission2.h ========================
 * 车载倒立摆-基础部分第二问: 正弦起摆 + LQR 平衡 v2.0
 *
 * 物理参数: M_cart≈200g, m_pend≈30g, m_rod≈15g, L≈30cm, l_cm≈17cm
 * 摆杆固有频率 ω_n≈4.88rad/s, 周期 T_n≈1.29s
 * 状态: x=[pos(cm), vel(cm/s), angle(deg), ang_vel(deg/s)]
 * 控制: u(cm/s) = -(K1*x_err + K2*v + K3*θ_err + K4*ω) + K5*∫θ_err
 * ========================================================== */

#ifndef __MISSION2_H
#define __MISSION2_H

#include "headfile.h"

/* ---- 目标 ---- */
#define BALANCE_TARGET       70.0f   /* 竖直角度 (potentiometer reading)   */
#define BALANCE_RANGE        4.0f    /* 进入平衡的偏差 ±°                  */
#define BALANCE_ENTER_RANGE  3.0f    /* 确认进入平衡的角度阈值 ±°          */
#define FALL_RECOVER_RANGE   10.0f   /* 倒下阈值: 超此重回起摆             */
#define FALL_HOLD_COUNT      40      /* 连续超出倒下阈值次数(200Hz: 200ms) */
#define BALANCE_SPEED_LIMIT  30.0f   /* 平衡最大车速 cm/s                  */
#define CART_POS_LIMIT_CM    80.0f   /* 小车位置软限位 cm (相对起点)       */

/* ---- LQR 增益 K = [K_POS, K_VEL, K_ANGLE, K_ANGLE_VEL] ---- */
/*
 * 调参法则:
 *   倒了扶不起来   → 加大 K_ANGLE
 *   来回震荡       → 加大 K_ANGLE_VEL, 减小 K_ANGLE
 *   车越跑越远     → 加大 K_POS, 加大 K_INT_ANGLE
 *   刹车太猛       → 减小 K_VEL
 *   有稳态偏差     → 加大 K_INT_ANGLE
 *
 * 起始值基于线性化模型 LQR 近似解, 需实测微调:
 *   A=[0 1 0 0;0 0 -31.9 0;0 0 0 1;0 0 1022 0]
 *   B=[0;4.3;0;7.4]  Q=diag(0.1,0.01,15,0.1)  R=1.0
 */
#define LQR_K_POS         0.015f  /* 位置 cm/s per cm                  */
#define LQR_K_VEL         0.08f   /* 速度阻尼                          */
#define LQR_K_ANGLE       0.55f   /* 角度 cm/s per deg                 */
#define LQR_K_ANGLE_VEL   0.10f   /* 角速度 cm/s per deg/s             */
#define LQR_K_INT_ANGLE   0.008f  /* 角度积分 cm/s per deg·s           */
#define LQR_INT_MAX        8.0f   /* 积分限幅                          */

/* ---- 起摆参数 (正弦起摆 + 能量检测) ---- */
#define SWING_PERIOD_MS    1290    /* 振荡周期 ms (匹配摆杆固有频率)     */
#define SWING_SPEED_START  25.0f   /* 起摆起始速度 cm/s                  */
#define SWING_SPEED_MAX    45.0f   /* 起摆最大速度 cm/s                  */
#define SWING_RAMP_TIME_S  4.0f    /* 爬升时间 s                        */
#define SWING_MIN_ENERGY_HOLD 30   /* 能量足够后稳定计数(200Hz)         */

/* ---- 角速度滤波器 ---- */
#define ANGVEL_LPF_ALPHA   0.15f   /* 角速度低通滤波系数(0<α≤1, 越小越平滑) */

/* ---- 外部接口 ---- */
void run_turn_logic2_200hz(void);
void mission2_show(void);
void stop2(void);

#endif
