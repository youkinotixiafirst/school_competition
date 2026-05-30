/* ======================== mission1.c ========================
 * 基础部分第一问: A→B 直线循迹 100cm, 编码器测距停止
 * ========================================================== */

#include "mission1.h"

#define PULSE_PER_CM  (pulse_cnt_per_circle_default / \
                      (2.0f * 3.1415926f * tire_radius_cm_default))

extern float       gray_status[2];
extern float       turn_scale;
extern float       speed_setup;
extern float       speed_expect[2];
extern float       speed_output[2];
extern float       speed_integral[2];
extern encoder     NEncoder;

static uint8_t  g_done        = 0;
static int32_t  g_start_pulse = 0;
static uint8_t  g_inited      = 0;

void run_turn_logic1_200hz(void)
{
    if (g_done) return;

    if (!g_inited) {
        g_start_pulse = (NEncoder.left_motor_total_cnt +
                         NEncoder.right_motor_total_cnt) / 2;
        speed_setup = CRUISE_SPEED;
        g_inited = 1;
    }

    speed_setup = CRUISE_SPEED;

    float turn_val;
    gray_turn_control_200hz(&turn_val);
    float err = ABS(gray_status[0]);

    if (err < 3.0f) {
        speed_expect[0] = speed_setup + turn_val * turn_scale;
        speed_expect[1] = speed_setup - turn_val * turn_scale;
    } else {
        float scale = (err >= 6.0f) ? 0.15f : 0.10f;
        speed_expect[0] = speed_setup + turn_val * scale;
        speed_expect[1] = speed_setup - turn_val * scale;
    }
}

void stop1(void)
{
    if (g_done) {
        speed_expect[0]  = 0.0f;
        speed_expect[1]  = 0.0f;
        speed_output[0]  = 0.0f;
        speed_output[1]  = 0.0f;
        speed_setup      = 0.0f;
        return;
    }

    int32_t now  = (NEncoder.left_motor_total_cnt +
                    NEncoder.right_motor_total_cnt) / 2;
    float   dist = (float)(now - g_start_pulse) / PULSE_PER_CM;

    if (dist >= TOTAL_DISTANCE_CM) {
        g_done = 1;
        speed_expect[0]  = 0.0f;
        speed_expect[1]  = 0.0f;
        speed_output[0]  = 0.0f;
        speed_output[1]  = 0.0f;
        speed_integral[0] = 0.0f;
        speed_integral[1] = 0.0f;
        speed_setup      = 0.0f;
    }
}

void mission1_show(void)
{
    static uint32_t last = 0;
    uint32_t now_ms = millis();
    if (now_ms - last < 200) return;
    last = now_ms;

    char buf[17];
    int32_t now  = (NEncoder.left_motor_total_cnt +
                    NEncoder.right_motor_total_cnt) / 2;
    float   dist = (float)(now - g_start_pulse) / PULSE_PER_CM;
    float   spd  = (NEncoder.left_motor_speed_cmps +
                    NEncoder.right_motor_speed_cmps) * 0.5f;

    sprintf(buf, "M1 A->B %s", g_done ? "DONE" : "RUN ");
    LCD_P6x8Str(0, 0, (unsigned char*)buf);

    sprintf(buf, "Spd:%4.1f/%4.1f", spd, speed_setup);
    LCD_P6x8Str(0, 1, (unsigned char*)buf);

    sprintf(buf, "Dist:%5.1f/%.0f", dist, TOTAL_DISTANCE_CM);
    LCD_P6x8Str(0, 2, (unsigned char*)buf);

    sprintf(buf, "Err:%+5.1f", gray_status[0]);
    LCD_P6x8Str(0, 3, (unsigned char*)buf);

    sprintf(buf, "L:%4.1f R:%4.1f",
            NEncoder.left_motor_speed_cmps,
            NEncoder.right_motor_speed_cmps);
    LCD_P6x8Str(0, 4, (unsigned char*)buf);
}
