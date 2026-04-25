#include "headfile.h"
#include "sensor.h"
#include "Fusion.h"

// 不使用温度控制和磁力计，简化配置
lpf_param accel_lpf_param, gyro_lpf_param;
lpf_buf gyro_filter_buf[3], accel_filter_buf[3];

sensor smartcar_imu;
FusionAhrs ahrs;
FusionOffset offset;
#define sampling_frequent 200

// 向量减法（保留）
void vector3f_sub(vector3f a, vector3f b, vector3f *c)
{
    c->x = a.x - b.x;
    c->y = a.y - b.y;
    c->z = a.z - b.z;
}

// 欧拉角转四元数（保留，但可能不需要）
void euler_to_quaternion(float *rpy, float *q)
{
    float sPitch2, cPitch2, sRoll2, cRoll2, sYaw2, cYaw2;
    FastSinCos(0.5f * rpy[0] * DEG2RAD, &sRoll2, &cRoll2);
    FastSinCos(0.5f * rpy[1] * DEG2RAD, &sPitch2, &cPitch2);
    FastSinCos(0.5f * rpy[2] * DEG2RAD, &sYaw2, &cYaw2);
    q[0] = cPitch2 * cRoll2 * cYaw2 + sPitch2 * sRoll2 * sYaw2;
    q[1] = sPitch2 * cRoll2 * cYaw2 - cPitch2 * sRoll2 * sYaw2;
    q[2] = cPitch2 * sRoll2 * cYaw2 + sPitch2 * cRoll2 * sYaw2;
    q[3] = cPitch2 * cRoll2 * sYaw2 - sPitch2 * sRoll2 * cYaw2;
    float recipNorm = invSqrt(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]);
    q[0] *= recipNorm; q[1] *= recipNorm; q[2] *= recipNorm; q[3] *= recipNorm;
}

// 四元数转方向余弦矩阵（保留，可能不需要）
void quaternion_to_cb2n(float *q, float *cb2n)
{
    float a = q[0], b = q[1], c = q[2], d = q[3];
    float bc = b * c, ad = a * d, bd = b * d, ac = a * c, cd = c * d, ab = a * b;
    float a2 = a * a, b2 = b * b, c2 = c * c, d2 = d * d;
    cb2n[0] = a2 + b2 - c2 - d2;
    cb2n[1] = 2 * (bc - ad);
    cb2n[2] = 2 * (bd + ac);
    cb2n[3] = 2 * (bc + ad);
    cb2n[4] = a2 - b2 + c2 - d2;
    cb2n[5] = 2 * (cd - ab);
    cb2n[6] = 2 * (bd - ac);
    cb2n[7] = 2 * (cd + ab);
    cb2n[8] = a2 - b2 - c2 + d2;
}

// 通过加速度计和磁力计计算初始四元数（删除磁力计，仅用加速度计估算）
void calculate_quaternion_init(vector3f a, vector3f m, float *q)
{
    // 忽略磁力计，只用加速度计计算初始俯仰和横滚，偏航设为0
    (void)m;  // 避免未使用警告
    float ax = a.x, ay = a.y, az = a.z;
    float rpy_obs_deg[3];
    rpy_obs_deg[0] = -57.3f * atan2f(ax, sqrtf(ay * ay + az * az));
    rpy_obs_deg[1] = 57.3f * atan2f(ay, sqrtf(ax * ax + az * az));
    rpy_obs_deg[2] = 0.0f;  // 初始偏航角设为0
    euler_to_quaternion(rpy_obs_deg, q);
}

// 简化的卡尔曼滤波（可选，保留用于俯仰角，但循迹不需要）
float kalman_filter(float angle, float gyro)
{
    static uint8_t init = 0;
    static float x = 0, P = 0.000001, Q = 0.000001, R = 0.35, k = 0;
    if (init == 0) { x = angle; init = 1; }
    x = x + gyro * 0.005f;
    P = P + Q;
    k = P / (P + R);
    x = x + k * (angle - x);
    P = (1 - k) * P;
    return x;
}

// IMU数据采样（删除温度控制、电池电压、磁力计相关）
void imu_data_sampling(void)
{
    if (smartcar_imu.lpf_init == 0)
    {
        set_cutoff_frequency(200, 50, &gyro_lpf_param);
        set_cutoff_frequency(200, 30, &accel_lpf_param);
        smartcar_imu.lpf_init = 1;
    }
    // 读取原始数据
    ICM206xx_Read_Data(&smartcar_imu._gyro_dps_raw, &smartcar_imu._accel_g_raw, &smartcar_imu.temperature_raw);
    // 低通滤波
    smartcar_imu.gyro_dps_raw.x = LPButterworth(smartcar_imu._gyro_dps_raw.x, &gyro_filter_buf[0], &gyro_lpf_param);
    smartcar_imu.gyro_dps_raw.y = LPButterworth(smartcar_imu._gyro_dps_raw.y, &gyro_filter_buf[1], &gyro_lpf_param);
    smartcar_imu.gyro_dps_raw.z = LPButterworth(smartcar_imu._gyro_dps_raw.z, &gyro_filter_buf[2], &gyro_lpf_param);
    smartcar_imu.accel_g_raw.x = LPButterworth(smartcar_imu._accel_g_raw.x, &accel_filter_buf[0], &accel_lpf_param);
    smartcar_imu.accel_g_raw.y = LPButterworth(smartcar_imu._accel_g_raw.y, &accel_filter_buf[1], &accel_lpf_param);
    smartcar_imu.accel_g_raw.z = LPButterworth(smartcar_imu._accel_g_raw.z, &accel_filter_buf[2], &accel_lpf_param);
    // 温度滤波（简单一阶）
    smartcar_imu.temperature_filter = 0.75f * smartcar_imu.temperature_raw + 0.25f * smartcar_imu.temperature_filter;
    // 校准后的数据
    vector3f_sub(smartcar_imu.gyro_dps_raw, smartcar_imu.gyro_offset, &smartcar_imu.gyro_dps);
    smartcar_imu.accel_g.x = smartcar_imu.accel_scale.x * smartcar_imu.accel_g_raw.x - smartcar_imu.accel_offset.x;
    smartcar_imu.accel_g.y = smartcar_imu.accel_scale.y * smartcar_imu.accel_g_raw.y - smartcar_imu.accel_offset.y;
    smartcar_imu.accel_g.z = smartcar_imu.accel_scale.z * smartcar_imu.accel_g_raw.z - smartcar_imu.accel_offset.z;
    // 自动校准（如果尚未校准）
    imu_calibration(&smartcar_imu.gyro_dps_raw, &smartcar_imu.accel_g_raw);
    // 计算观测角度（仅用于卡尔曼滤波，可不使用）
    float ax = smartcar_imu.accel_g.x, ay = smartcar_imu.accel_g.y, az = smartcar_imu.accel_g.z;
    smartcar_imu.rpy_obs_deg[0] = -57.3f * atan2f(ax, sqrtf(ay * ay + az * az));
    smartcar_imu.rpy_obs_deg[1] = 57.3f * atan2f(ay, sqrtf(ax * ax + az * az));
    smartcar_imu.rpy_kalman_deg[1] = kalman_filter(smartcar_imu.rpy_obs_deg[1], smartcar_imu.gyro_dps.x);
}

// 姿态更新（使用Fusion库，只使用加速度计和陀螺仪，无磁力计）
void trackless_ahrs_update(void)
{
    FusionVector gyroscope = {{0, 0, 0}};
    FusionVector accelerometer = {{0, 0, 1.0f}};
    gyroscope.axis.x = smartcar_imu.gyro_dps.x;
    gyroscope.axis.y = smartcar_imu.gyro_dps.y;
    gyroscope.axis.z = smartcar_imu.gyro_dps.z;
    accelerometer.axis.x = smartcar_imu.accel_g.x;
    accelerometer.axis.y = smartcar_imu.accel_g.y;
    accelerometer.axis.z = smartcar_imu.accel_g.z;

    if (smartcar_imu.quaternion_init_ok == 0)
    {
        // 温度稳定标志简化：直接认为就绪
        smartcar_imu.temperature_stable_flag = 1;
        if (smartcar_imu.temperature_stable_flag == 1)
        {
            // 初始四元数估算（不使用磁力计，偏航初始为0）
            vector3f zero_mag = {0, 0, 0};
            calculate_quaternion_init(smartcar_imu.accel_g, zero_mag, smartcar_imu.quaternion_init);
            smartcar_imu.quaternion_init_ok = 1;
            FusionOffsetInitialise(&offset, sampling_frequent);
            FusionAhrsInitialise(&ahrs);
            const FusionAhrsSettings settings = {
                .gain = 0.5f,
                .accelerationRejection = 10.0f,
                .magneticRejection = 20.0f,
                .rejectionTimeout = 5 * sampling_frequent,
            };
            FusionAhrsSetSettings(&ahrs, &settings);
        }
    }

    if (smartcar_imu.quaternion_init_ok == 1)
    {
        gyroscope = FusionOffsetUpdate(&offset, gyroscope);
        // 无磁力计更新
        FusionAhrsUpdateNoMagnetometer(&ahrs, gyroscope, accelerometer, 0.005f);
        FusionEuler euler = FusionQuaternionToEuler(FusionAhrsGetQuaternion(&ahrs));
        smartcar_imu.rpy_deg[_ROL] = euler.angle.pitch;
        smartcar_imu.rpy_deg[_PIT] = euler.angle.roll;
        smartcar_imu.rpy_deg[_YAW] = euler.angle.yaw;

        // 收敛标志简化
        if (smartcar_imu.imu_convergence_cnt < 200 * 5)
            smartcar_imu.imu_convergence_cnt++;
        else
            smartcar_imu.imu_convergence_flag = 1;
    }

    // 保存角速度（单位 deg/s）
    smartcar_imu.rpy_gyro_dps[_PIT] = smartcar_imu.gyro_dps.x;
    smartcar_imu.rpy_gyro_dps[_ROL] = smartcar_imu.gyro_dps.y;
    smartcar_imu.rpy_gyro_dps[_YAW] = smartcar_imu.gyro_dps.z;

    // 计算三角函数（可能用于其他模块，保留）
    smartcar_imu.sin_rpy[_PIT] = FastSin(smartcar_imu.rpy_deg[_PIT] * DEG2RAD);
    smartcar_imu.cos_rpy[_PIT] = FastCos(smartcar_imu.rpy_deg[_PIT] * DEG2RAD);
    smartcar_imu.sin_rpy[_ROL] = FastSin(smartcar_imu.rpy_deg[_ROL] * DEG2RAD);
    smartcar_imu.cos_rpy[_ROL] = FastCos(smartcar_imu.rpy_deg[_ROL] * DEG2RAD);
    smartcar_imu.sin_rpy[_YAW] = FastSin(smartcar_imu.rpy_deg[_YAW] * DEG2RAD);
    smartcar_imu.cos_rpy[_YAW] = FastCos(smartcar_imu.rpy_deg[_YAW] * DEG2RAD);

    // 提取四元数
    smartcar_imu.quaternion[0] = ahrs.quaternion.element.w;
    smartcar_imu.quaternion[1] = ahrs.quaternion.element.x;
    smartcar_imu.quaternion[2] = ahrs.quaternion.element.y;
    smartcar_imu.quaternion[3] = ahrs.quaternion.element.z;

    // 计算方向余弦矩阵（可选）
    quaternion_to_cb2n(smartcar_imu.quaternion, smartcar_imu.cb2n);

    // 计算导航系下的偏航角速度（用于转向控制，可保留）
    smartcar_imu.yaw_gyro_enu = -smartcar_imu.sin_rpy[_ROL] * smartcar_imu.gyro_dps.x
                                + smartcar_imu.cos_rpy[_ROL] * smartcar_imu.sin_rpy[_PIT] * smartcar_imu.gyro_dps.y
                                + smartcar_imu.cos_rpy[_PIT] * smartcar_imu.cos_rpy[_ROL] * smartcar_imu.gyro_dps.z;

    // 删除电池电压和加热器相关代码
}

// 删除 imu_temperature_ctrl, simulation_pwm_init, simulation_pwm_output, temperature_state_get, temperature_state_check
// 因为这些依赖 heater_port, heater_pin 和 get_battery_voltage，且循迹不需要

// 里程计系统（如果需要，可保留；但循迹不需要，注释掉）
// void get_wheel_speed(void) { ... }