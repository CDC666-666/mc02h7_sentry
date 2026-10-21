#ifndef CHASSIS_POWER_H
#define CHASSIS_POWER_H

#include <stdint.h>

#define CHASSIS_POWER_MOTOR_COUNT 4U

/* 单个电机的经验功率模型：P = K0 + K1*I + K2*w + K3*I*w + K4*I^2 + K5*w^2。 */
typedef struct {
    float k0;
    float k1;
    float k2;
    float k3;
    float k4;
    float k5;
    float current_conversion;
} ChassisPowerModel_t;

/* 仅保存遥测和调试数据，单位均为 W；attenuation 为本帧电流保留比例。 */
typedef struct {
    uint8_t enabled;
    float requested_limit_w;
    float budget_w;
    float predicted_before_w;
    float predicted_after_w;
    float motor_limit_w[CHASSIS_POWER_MOTOR_COUNT];
    float motor_before_w[CHASSIS_POWER_MOTOR_COUNT];
    float motor_after_w[CHASSIS_POWER_MOTOR_COUNT];
    float attenuation[CHASSIS_POWER_MOTOR_COUNT];
} ChassisPowerManager_t;

void ChassisPower_Init(ChassisPowerManager_t *manager);
void ChassisPower_Limit(ChassisPowerManager_t *manager,
                        const ChassisPowerModel_t *model,
                        float desired_current[CHASSIS_POWER_MOTOR_COUNT],
                        const float target_motor_rad_s[CHASSIS_POWER_MOTOR_COUNT],
                        const float feedback_motor_rad_s[CHASSIS_POWER_MOTOR_COUNT],
                        float total_limit_w,
                        float buffer_attenuation,
                        float compensation,
                        float error_equal_rpm,
                        float reserved_border_w,
                        float reserved_per_motor_w);

#endif /* CHASSIS_POWER_H */
