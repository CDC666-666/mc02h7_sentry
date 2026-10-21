#include "cmsis_os.h"
#include "FreeRTOS.h"
#include "task.h"
#include "INS_Task.h"
#include "Bmi088.h"
#include "Kalman_Filter.h"
#include "Quaternion.h"
#include "bsp_pwm.h"
#include "config.h"
#include "pid.h"
#include "robot_param.h"

#include <math.h>

#define IMU_UPDATE_PERIOD_S (0.001f)

INS_Info_Typedef INS_Info;
volatile uint8_t INS_ImuReady;
volatile uint16_t INS_GyroBiasSampleCount;
volatile float INS_GyroRaw[3];
volatile float INS_GyroBias[3];

/* This is the raw BMI088-to-IMU rotation used by StandardRobotpp. The second
 * matrix is profile-owned because it depends on how the H723 board is mounted. */
static const float bmi088_sensor_matrix[3][3] = {
    {0.0f, 1.0f, 0.0f},
    {-1.0f, 0.0f, 0.0f},
    {0.0f, 0.0f, 1.0f},
};
static const float board_install_matrix[3][3] = {__BOARD_INSTALL_SPIN_MATRIX};

static KalmanFilter_Info_TypeDef gravity_kf;
static float gravity_vector[3];
static const float gravity_kf_f[9] = {
    1.0f, 0.0f, 0.0f,
    0.0f, 1.0f, 0.0f,
    0.0f, 0.0f, 1.0f,
};
static const float gravity_kf_p[9] = {
    100.0f, 0.1f, 0.1f,
    0.1f, 100.0f, 0.1f,
    0.1f, 0.1f, 100.0f,
};
static const float gravity_kf_q[9] = {
    1.0f, 0.0f, 0.0f,
    0.0f, 1.0f, 0.0f,
    0.0f, 0.0f, 1.0f,
};
static const float gravity_kf_r[9] = {
    2000.0f, 0.0f, 0.0f,
    0.0f, 2000.0f, 0.0f,
    0.0f, 0.0f, 2000.0f,
};
static const float gravity_kf_h[9] = {
    1.0f, 0.0f, 0.0f,
    0.0f, 1.0f, 0.0f,
    0.0f, 0.0f, 1.0f,
};

static float quaternion_ekf_a[36] = {
    1.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f,
    0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 0.0f,
    0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f,
    0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f,
    0.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f,
    0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 1.0f,
};
static float quaternion_ekf_p[36] = {
    100000.0f, 0.1f, 0.1f, 0.1f, 0.1f, 0.1f,
    0.1f, 100000.0f, 0.1f, 0.1f, 0.1f, 0.1f,
    0.1f, 0.1f, 100000.0f, 0.1f, 0.1f, 0.1f,
    0.1f, 0.1f, 0.1f, 100000.0f, 0.1f, 0.1f,
    0.1f, 0.1f, 0.1f, 0.1f, 10000.0f, 0.1f,
    0.1f, 0.1f, 0.1f, 0.1f, 0.1f, 10000.0f,
};

static float temp_pid_parameters[PID_PARAMETER_NUM] = {
    __IMU_TEMP_PID_KP, __IMU_TEMP_PID_KI, __IMU_TEMP_PID_KD,
    0.0f, 0.0f, __IMU_TEMP_PID_MAX_IOUT, __IMU_TEMP_PID_MAX_OUT,
};
static PID_Info_TypeDef temp_pid;
static uint8_t imu_temp_stable;
static uint16_t imu_temp_stable_count;
static uint8_t imu_gyro_bias_captured;
static uint16_t imu_attitude_warmup_count;
static float imu_gyro_bias_sum[3];

static void INS_Task_Init(void);
static void IMU_Rotate(const float source[3], float destination[3]);
static void GravityEKF_Update(const float gyro[3], const float accel[3]);
static void BMI088_Temp_Control(float temperature);
static void INS_GyroBiasUpdate(const float gyro[3], const float accel[3]);
static uint8_t INS_IsStationary(const float gyro[3], const float accel[3]);

uint8_t INS_IsImuReady(void)
{
    return INS_ImuReady;
}

void INS_Task(void const *argument)
{
    (void)argument;
    /* 强实现覆盖了 freertos.c 的弱任务，必须在此初始化 Type-C USB CDC。 */
    INS_Task_Init();
    BMI088_DMA_Start((BMI088_TaskHandle_t)xTaskGetCurrentTaskHandle());

    for (;;) {
        float corrected_gyro[3];

        if (ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(1U)) == 0U) {
            /* Keep the IMU alive even when the board's DRDY EXTI is not wired
             * or is held at the wrong polarity. The normal EXTI path remains
             * preferred and this request is ignored while DMA is busy. */
            BMI088_DMA_RequestGyro();
            continue;
        }

        /* The DMA callback has completed and decoded the latest BMI088 frame. */
        IMU_Rotate(BMI088_Info.Gyro, INS_Info.Gyro);
        IMU_Rotate(BMI088_Info.Accel, INS_Info.Accel);
        memcpy((void *)INS_GyroRaw, INS_Info.Gyro, sizeof(INS_GyroRaw));
        BMI088_Temp_Control(BMI088_Info.Temperature);
        INS_GyroBiasUpdate(INS_Info.Gyro, INS_Info.Accel);

        if (imu_gyro_bias_captured == 0U) {
            continue;
        }

        corrected_gyro[0] = INS_Info.Gyro[0] - INS_GyroBias[0];
        corrected_gyro[1] = INS_Info.Gyro[1] - INS_GyroBias[1];
        corrected_gyro[2] = INS_Info.Gyro[2] - INS_GyroBias[2];
        memcpy(INS_Info.Gyro, corrected_gyro, sizeof(corrected_gyro));

        GravityEKF_Update(corrected_gyro, INS_Info.Accel);
        if (Quaternion_Info.QuaternionEKF.Initialized) {
            QuaternionEKF_Update(&Quaternion_Info, corrected_gyro, gravity_vector, IMU_UPDATE_PERIOD_S);
        }

        memcpy(INS_Info.Angle, Quaternion_Info.EulerAngle, sizeof(INS_Info.Angle));
        INS_Info.Pitch_Angle = INS_Info.Angle[IMU_ANGLE_INDEX_PITCH] * RadiansToDegrees;
        INS_Info.Yaw_Angle = INS_Info.Angle[IMU_ANGLE_INDEX_YAW] * RadiansToDegrees;
        INS_Info.Roll_Angle = INS_Info.Angle[IMU_ANGLE_INDEX_ROLL] * RadiansToDegrees;

        if ((INS_Info.Yaw_Angle - INS_Info.Last_Yaw_Angle) < -180.0f) {
            INS_Info.YawRoundCount++;
        } else if ((INS_Info.Yaw_Angle - INS_Info.Last_Yaw_Angle) > 180.0f) {
            INS_Info.YawRoundCount--;
        }
        INS_Info.Last_Yaw_Angle = INS_Info.Yaw_Angle;
        INS_Info.Yaw_TolAngle = INS_Info.Yaw_Angle + INS_Info.YawRoundCount * 360.0f;

        INS_Info.Pitch_Gyro = INS_Info.Gyro[IMU_GYRO_INDEX_PITCH] * RadiansToDegrees;
        INS_Info.Yaw_Gyro = INS_Info.Gyro[IMU_GYRO_INDEX_YAW] * RadiansToDegrees;
        INS_Info.Roll_Gyro = INS_Info.Gyro[IMU_GYRO_INDEX_ROLL] * RadiansToDegrees;

        if (imu_attitude_warmup_count < __IMU_ATTITUDE_WARMUP_SAMPLES) {
            imu_attitude_warmup_count++;
            if (imu_attitude_warmup_count >= __IMU_ATTITUDE_WARMUP_SAMPLES) {
                INS_ImuReady = 1U;
            }
        }
    }
}

static void INS_Task_Init(void)
{
    Kalman_Filter_Init(&gravity_kf, 3U, 0U, 3U);
    if (gravity_kf.Initialized) {
        memcpy(gravity_kf.Data.A, gravity_kf_f, sizeof(gravity_kf_f));
        memcpy(gravity_kf.Data.P, gravity_kf_p, sizeof(gravity_kf_p));
        memcpy(gravity_kf.Data.Q, gravity_kf_q, sizeof(gravity_kf_q));
        memcpy(gravity_kf.Data.R, gravity_kf_r, sizeof(gravity_kf_r));
        memcpy(gravity_kf.Data.H, gravity_kf_h, sizeof(gravity_kf_h));
    }

    PID_Init(&temp_pid, PID_POSITION, temp_pid_parameters);
    QuaternionEKF_Init(&Quaternion_Info, 10.0f, 0.001f, 1000000.0f,
                       quaternion_ekf_a, quaternion_ekf_p);

    INS_ImuReady = 0U;
    INS_GyroBiasSampleCount = 0U;
    INS_GyroRaw[0] = 0.0f;
    INS_GyroRaw[1] = 0.0f;
    INS_GyroRaw[2] = 0.0f;
    INS_GyroBias[0] = 0.0f;
    INS_GyroBias[1] = 0.0f;
    INS_GyroBias[2] = 0.0f;
    imu_temp_stable = 0U;
    imu_temp_stable_count = 0U;
    imu_gyro_bias_captured = 0U;
    imu_attitude_warmup_count = 0U;
    memset(imu_gyro_bias_sum, 0, sizeof(imu_gyro_bias_sum));
}

static void IMU_Rotate(const float source[3], float destination[3])
{
    float sensor_frame[3];
    uint8_t row;
    uint8_t column;

    for (row = 0U; row < 3U; row++) {
        sensor_frame[row] = 0.0f;
        for (column = 0U; column < 3U; column++) {
            sensor_frame[row] += bmi088_sensor_matrix[row][column] * source[column];
        }
    }

    for (row = 0U; row < 3U; row++) {
        destination[row] = 0.0f;
        for (column = 0U; column < 3U; column++) {
            destination[row] += board_install_matrix[row][column] * sensor_frame[column];
        }
    }
}

static void GravityEKF_Update(const float gyro[3], const float accel[3])
{
	if (!gravity_kf.Initialized) {
		gravity_vector[0] = accel[0];
		gravity_vector[1] = accel[1];
		gravity_vector[2] = accel[2];
		return;
	}
    float gxdt = gyro[0] * IMU_UPDATE_PERIOD_S;
    float gydt = gyro[1] * IMU_UPDATE_PERIOD_S;
    float gzdt = gyro[2] * IMU_UPDATE_PERIOD_S;

    gravity_kf.Data.A[1] = gzdt;
    gravity_kf.Data.A[2] = -gydt;
    gravity_kf.Data.A[3] = -gzdt;
    gravity_kf.Data.A[5] = gxdt;
    gravity_kf.Data.A[6] = gydt;
    gravity_kf.Data.A[7] = -gxdt;

    gravity_kf.MeasuredVector[0] = accel[0];
    gravity_kf.MeasuredVector[1] = accel[1];
    gravity_kf.MeasuredVector[2] = accel[2];
    Kalman_Filter_Update(&gravity_kf);

    gravity_vector[0] = gravity_kf.Output[0];
    gravity_vector[1] = gravity_kf.Output[1];
    gravity_vector[2] = gravity_kf.Output[2];
}

static void BMI088_Temp_Control(float temperature)
{
    if (__HEAT_IMU == 0U) {
        Heat_Power_Control(0U);
        return;
    }

    if (temperature < (__IMU_CONTROL_TEMPERATURE - 1.0f)) {
        Heat_Power_Control((uint16_t)(__IMU_TEMP_PID_MAX_OUT - 1.0f));
    } else {
        PID_Calculate(&temp_pid, __IMU_CONTROL_TEMPERATURE, temperature);
        if (temp_pid.Output < 0.0f) {
            temp_pid.Output = 0.0f;
        }
        Heat_Power_Control((uint16_t)temp_pid.Output);
    }

    /* This board can be heated above the setpoint by the H7 itself. There is
     * no active cooling, so a temperature at or above 40 C is ready to use. */
    if (temperature >= __IMU_CONTROL_TEMPERATURE) {
        imu_temp_stable_count = __IMU_TEMP_STABLE_SAMPLES;
        imu_temp_stable = 1U;
    } else {
        imu_temp_stable_count = 0U;
        imu_temp_stable = 0U;
    }
}

static void INS_GyroBiasUpdate(const float gyro[3], const float accel[3])
{
    if (imu_gyro_bias_captured != 0U) {
        return;
    }

#if (__IMU_GYRO_USE_FIXED_BIAS != 0U)
    (void)gyro;
    (void)accel;

    if (imu_temp_stable == 0U) {
        INS_GyroBiasSampleCount = 0U;
        return;
    }

    INS_GyroBias[0] = __IMU_GYRO_FIXED_BIAS_X;
    INS_GyroBias[1] = __IMU_GYRO_FIXED_BIAS_Y;
    INS_GyroBias[2] = __IMU_GYRO_FIXED_BIAS_Z;
    INS_GyroBiasSampleCount = __IMU_GYRO_BIAS_SAMPLES;
    imu_gyro_bias_captured = 1U;
    imu_attitude_warmup_count = 0U;
#else
    if ((imu_temp_stable == 0U) || (INS_IsStationary(gyro, accel) == 0U)) {
        INS_GyroBiasSampleCount = 0U;
        memset(imu_gyro_bias_sum, 0, sizeof(imu_gyro_bias_sum));
        return;
    }

    imu_gyro_bias_sum[0] += gyro[0];
    imu_gyro_bias_sum[1] += gyro[1];
    imu_gyro_bias_sum[2] += gyro[2];
    INS_GyroBiasSampleCount++;

    if (INS_GyroBiasSampleCount >= __IMU_GYRO_BIAS_SAMPLES) {
        INS_GyroBias[0] = imu_gyro_bias_sum[0] / (float)INS_GyroBiasSampleCount;
        INS_GyroBias[1] = imu_gyro_bias_sum[1] / (float)INS_GyroBiasSampleCount;
        INS_GyroBias[2] = imu_gyro_bias_sum[2] / (float)INS_GyroBiasSampleCount;
        imu_gyro_bias_captured = 1U;
        imu_attitude_warmup_count = 0U;
    }
#endif
}

static uint8_t INS_IsStationary(const float gyro[3], const float accel[3])
{
    float gyro_norm = sqrtf(gyro[0] * gyro[0] + gyro[1] * gyro[1] + gyro[2] * gyro[2]);
    float accel_norm = sqrtf(accel[0] * accel[0] + accel[1] * accel[1] + accel[2] * accel[2]);

    return (gyro_norm <= __IMU_GYRO_BIAS_MAX_RATE &&
            fabsf(accel_norm - GravityAccel) <= __IMU_GYRO_BIAS_ACCEL_TOL) ? 1U : 0U;
}
