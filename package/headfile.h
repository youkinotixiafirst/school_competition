#ifndef __HEADFILE_H
#define __HEADFILE_H

#include "ti_msp_dl_config.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <stdbool.h>
#include <math.h>
#include "user.h"
#include "datatype.h"
#include "system.h"
#include "ntimer.h"
#include "npwm.h"
#include "nqei.h"
#include "motor_control.h"
#include "gray_detection.h"
#include "pid.h"
#include "wp_math.h"
#include "sensor.h"
#include "icm20608.h"
#include "ni2c.h"
#include "oled.h"
#include "ngpio.h"
#include "nbutton.h"
#include "filter.h"
#include "nppm.h"
// 默认参数（可根据需要修改）
#define left_motor_encoder_dir_default    0
#define right_motor_encoder_dir_default   0
#define left_motion_dir_default           0
#define right_motion_dir_default          0
#define tire_radius_cm_default            2.40f
#define pulse_cnt_per_circle_default      1060

#define speed_kp_default                  8.0f
#define speed_ki_default                  0.6f
#define speed_kd_default                  0.0f






#endif