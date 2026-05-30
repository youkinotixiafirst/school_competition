/* ======================== mission2.c ========================
 * 车载倒立摆-基础部分第二问 v2.0
 * 状态机: IDLE → SWING_UP(正弦起摆+能量检测) → BALANCE(LQR全状态反馈+积分)
 *
 * 起摆: 正弦激励 @ 摆杆固有频率, 幅度线性爬升
 *       能量检测: 当摆角连续进入平衡区后切换到LQR
 * 平衡: LQR五状态反馈 [pos, vel, angle, angvel, ∫angle_err]
 *       倒下 → 自动重新起摆
 *       位置软限位防止跑出轨道
 * ========================================================== */

#include "mission2.h"

#define PULSE_PER_CM  (pulse_cnt_per_circle_default / (2.0f * 3.1415926f * tire_radius_cm_default))

/* ---- 外部变量 ---- */
extern float speed_expect[2];
extern float speed_output[2];
extern encoder NEncoder;

/* ---- 状态枚举 ---- */
typedef enum {
    PEND_IDLE = 0,
    PEND_SWING_UP,
    PEND_BALANCE,
} PendState;

/* ---- 内部状态 ---- */
static PendState  g_state         = PEND_IDLE;
static uint32_t   g_tick          = 0;
static float      g_angle         = 0.0f;    /* 当前角度 (经过滤波)       */
static float      g_angle_raw     = 0.0f;    /* 原始角度                  */
static float      g_last_angle    = 0.0f;
static float      g_angle_vel     = 0.0f;    /* 角速度 deg/s (LPF滤波)    */
static float      g_angle_vel_raw = 0.0f;    /* 原始角速度估计            */
static float      g_cart_pos      = 0.0f;    /* 小车位置 cm (相对起点)    */
static float      g_cart_vel      = 0.0f;    /* 小车速度 cm/s             */
static float      g_angle_int     = 0.0f;    /* 角度误差积分              */
static int32_t    g_pos_ref       = 0;       /* 编码器参考零点            */
static uint8_t    g_inited        = 0;
static uint16_t   g_fall_count    = 0;       /* 倒下计数                  */
static uint16_t   g_energy_hold   = 0;       /* 能量足够保持计数          */

/* ==================== 内部函数声明 ==================== */
static void pend_go_swing_up(void);
static void pend_do_swing_up(void);
static void pend_do_lqr_balance(void);
static void pend_limit_cart_position(float *u);

/* ==================== 200Hz 主控制循环 ==================== */
void run_turn_logic2_200hz(void)
{
    /* ---- 首次初始化 ---- */
    if (!g_inited) {
        WDD35D4_Init();
        g_pos_ref = (NEncoder.left_motor_total_cnt +
                     NEncoder.right_motor_total_cnt) / 2;
        g_angle         = 0.0f;
        g_angle_raw     = 0.0f;
        g_last_angle    = 0.0f;
        g_angle_vel     = 0.0f;
        g_angle_vel_raw = 0.0f;
        g_cart_pos      = 0.0f;
        g_cart_vel      = 0.0f;
        g_angle_int     = 0.0f;
        g_tick          = 0;
        g_fall_count    = 0;
        g_energy_hold   = 0;
        g_state         = PEND_SWING_UP;
        g_inited        = 1;
    }

    /* ---- 读取传感器 ---- */
    WDD35D4_StateMachine();
    g_angle_raw  = WDD35D4_GetAngle();

    /* 角度低通滤波 (减少ADC噪声) */
    g_angle = g_angle * 0.85f + g_angle_raw * 0.15f;

    /* 角速度: 差分 + 低通滤波 (用滤波后的角度做差分更平滑) */
    g_angle_vel_raw = (g_angle - g_last_angle) * 200.0f;   /* 200Hz 差分 */
    g_angle_vel = g_angle_vel * (1.0f - ANGVEL_LPF_ALPHA)
                + g_angle_vel_raw * ANGVEL_LPF_ALPHA;
    g_last_angle = g_angle;

    /* 编码器里程: 位置 + 速度 */
    {
        int32_t now = (NEncoder.left_motor_total_cnt +
                       NEncoder.right_motor_total_cnt) / 2;
        g_cart_pos = (float)(now - g_pos_ref) / PULSE_PER_CM;
        g_cart_vel = (NEncoder.left_motor_speed_cmps +
                      NEncoder.right_motor_speed_cmps) * 0.5f;
    }
    g_tick++;

    /* ---- 状态机 ---- */
    switch (g_state) {
    case PEND_SWING_UP:
        pend_do_swing_up();
        break;
    case PEND_BALANCE:
        pend_do_lqr_balance();
        break;
    default:
        break;
    }
}

/* ==================== 起摆 ==================== */
static void pend_go_swing_up(void)
{
    g_tick        = 0;
    g_energy_hold = 0;
    g_fall_count  = 0;
    g_angle_int   = 0.0f;
    g_state       = PEND_SWING_UP;

    /* 重新标定位置零点 */
    g_pos_ref = (NEncoder.left_motor_total_cnt +
                 NEncoder.right_motor_total_cnt) / 2;
    g_cart_pos = 0.0f;
}

static void pend_do_swing_up(void)
{
    float error = g_angle - BALANCE_TARGET;

    /* ---- 能量足够检测: 连续多个周期角度在平衡区内 ---- */
    if (fabsf(error) <= BALANCE_ENTER_RANGE) {
        g_energy_hold++;
        if (g_energy_hold >= SWING_MIN_ENERGY_HOLD) {
            /* 切换到平衡模式 */
            g_state       = PEND_BALANCE;
            g_angle_int   = 0.0f;
            g_fall_count  = 0;
            g_energy_hold = 0;
            speed_expect[0] = 0.0f;
            speed_expect[1] = 0.0f;
            return;
        }
    } else {
        /* 短暂进入后又出去, 重置计数 (带一点迟滞) */
        if (g_energy_hold > 0 && fabsf(error) > BALANCE_RANGE) {
            g_energy_hold = 0;
        }
    }

    /* ---- 正弦起摆: 频率匹配摆杆固有频率, 幅度线性爬升 ---- */
    uint32_t period_ticks = (uint32_t)(SWING_PERIOD_MS / 5.0f);   /* 5ms per tick @ 200Hz */
    if (period_ticks < 10) period_ticks = 10;

    uint32_t ramp_ticks = (uint32_t)(SWING_RAMP_TIME_S * 200.0f);

    /* 相位: 用 sin 使速度从零开始渐变, 避免突变 */
    float phase = (float)(g_tick % period_ticks) / (float)period_ticks
                * 2.0f * 3.14159265f;

    /* 幅度爬升: 线性从 START → MAX */
    float ramp = (float)g_tick / (float)ramp_ticks;
    if (ramp > 1.0f) ramp = 1.0f;

    float amplitude = SWING_SPEED_START
                    + (SWING_SPEED_MAX - SWING_SPEED_START) * ramp;
    float speed = amplitude * sinf(phase);

    speed_expect[0] = speed;
    speed_expect[1] = speed;
}

/* ==================== 平衡: LQR 全状态反馈 + 积分 ==================== */
static void pend_do_lqr_balance(void)
{
    float theta_err = g_angle - BALANCE_TARGET;

    /* ---- 倒下检测 (带连续确认, 防止误触发) ---- */
    if (fabsf(theta_err) > FALL_RECOVER_RANGE) {
        g_fall_count++;
        if (g_fall_count >= FALL_HOLD_COUNT) {
            pend_go_swing_up();
            return;
        }
    } else {
        g_fall_count = 0;
    }

    /* ---- 角度积分 (抗饱和) ---- */
    g_angle_int += theta_err * 0.005f;   /* dt = 1/200 = 0.005s */
    if (g_angle_int >  LQR_INT_MAX) g_angle_int =  LQR_INT_MAX;
    if (g_angle_int < -LQR_INT_MAX) g_angle_int = -LQR_INT_MAX;

    /* 进入平衡区时清除积分, 防止起摆阶段的累积 */
    if (fabsf(theta_err) > BALANCE_RANGE) {
        g_angle_int *= 0.95f;   /* 缓慢泄漏 */
    }

    /* ---- LQR 全状态反馈 ---- */
    /* u = -(K_pos * x + K_vel * v + K_angle * θ_err + K_angvel * ω)
     *     + K_int * ∫θ_err
     *
     * 注意: 位置项用误差(当前位置-目标位置=0), 让小车回到原点附近
     */
    float u = -(LQR_K_POS       * g_cart_pos
              + LQR_K_VEL       * g_cart_vel
              + LQR_K_ANGLE     * theta_err
              + LQR_K_ANGLE_VEL * g_angle_vel)
              + LQR_K_INT_ANGLE * g_angle_int;

    /* ---- 限幅 ---- */
    if (u >  BALANCE_SPEED_LIMIT) u =  BALANCE_SPEED_LIMIT;
    if (u < -BALANCE_SPEED_LIMIT) u = -BALANCE_SPEED_LIMIT;

    /* ---- 位置软限位: 防止小车跑出轨道 ---- */
    pend_limit_cart_position(&u);

    speed_expect[0] = u;
    speed_expect[1] = u;
}

/* ---- 位置软限位 ---- */
static void pend_limit_cart_position(float *u)
{
    float margin = 10.0f;   /* 限位缓冲区 cm */

    if (g_cart_pos > CART_POS_LIMIT_CM - margin) {
        /* 太靠右 → 强制向左 */
        float brake = -15.0f - (g_cart_pos - (CART_POS_LIMIT_CM - margin)) * 2.0f;
        if (brake < *u) *u = brake;   /* 取更负的值 (更强制左转) */
    } else if (g_cart_pos > CART_POS_LIMIT_CM) {
        /* 超出硬限位 → 强力回拉 */
        *u = -30.0f;
    }

    if (g_cart_pos < -(CART_POS_LIMIT_CM - margin)) {
        /* 太靠左 → 强制向右 */
        float brake = 15.0f + ((-(CART_POS_LIMIT_CM - margin)) - g_cart_pos) * 2.0f;
        if (brake > *u) *u = brake;
    } else if (g_cart_pos < -CART_POS_LIMIT_CM) {
        /* 超出硬限位 → 强力回拉 */
        *u = 30.0f;
    }
}

/* ==================== LCD 显示 (200ms 刷新) ==================== */
void mission2_show(void)
{
    static uint32_t last_show = 0;
    if (millis() - last_show < 200) return;
    last_show = millis();

    char buf[17];

    LCD_P6x8Str(0, 0, (unsigned char*)"M2 LQR Balance v2");

    const char *st;
    switch (g_state) {
    case PEND_SWING_UP: st = "SWING UP"; break;
    case PEND_BALANCE:  st = "BALANCE "; break;
    default:            st = "IDLE    "; break;
    }
    sprintf(buf, "St:%s E:%u", st, g_energy_hold);
    LCD_P6x8Str(0, 1, (unsigned char*)buf);

    sprintf(buf, "Ang:%5.1f /%.0f", g_angle, BALANCE_TARGET);
    LCD_P6x8Str(0, 2, (unsigned char*)buf);

    sprintf(buf, "dA:%+5.0f E:%+4.1f", g_angle_vel,
            g_angle - BALANCE_TARGET);
    LCD_P6x8Str(0, 3, (unsigned char*)buf);

    float out = (speed_expect[0] + speed_expect[1]) * 0.5f;
    sprintf(buf, "U:%+4.0f X:%+4.0f", out, g_cart_pos);
    LCD_P6x8Str(0, 4, (unsigned char*)buf);

    sprintf(buf, "V:%+4.0f I:%+4.1f", g_cart_vel, g_angle_int);
    LCD_P6x8Str(0, 5, (unsigned char*)buf);

    sprintf(buf, "KA:%.2f KV:%.3f", LQR_K_ANGLE, LQR_K_ANGLE_VEL);
    LCD_P6x8Str(0, 6, (unsigned char*)buf);

    uint16_t raw = WDD35D4_ReadRaw();
    sprintf(buf, "ADC:%5u FC:%u", raw, g_fall_count);
    LCD_P6x8Str(0, 7, (unsigned char*)buf);
}

/* ==================== 停止逻辑 ==================== */
void stop2(void)
{
    /* 预留: 任务完成后的停止处理 */
    /* 当前由主循环的 start_flag 控制启停 */
}
