#include "chassis_power.h"

#include <math.h>
#include <string.h>

#define CHASSIS_POWER_RAD_S_TO_RPM (9.549296586f)
#define CHASSIS_POWER_EPSILON      (0.0001f)

static float ChassisPower_Clamp(float value, float minimum, float maximum)
{
    if (value < minimum) return minimum;
    if (value > maximum) return maximum;
    return value;
}

static float ChassisPower_Predict(const ChassisPowerModel_t *model,
                                  float command_current,
                                  float motor_rad_s)
{
    float current;
    float speed_rpm;

    if (model == 0 || model->current_conversion <= CHASSIS_POWER_EPSILON) return 0.0f;
    current = fabsf(command_current) / model->current_conversion;
    speed_rpm = fabsf(motor_rad_s) * CHASSIS_POWER_RAD_S_TO_RPM;
    return model->k0 + model->k1 * current + model->k2 * speed_rpm +
           model->k3 * current * speed_rpm + model->k4 * current * current +
           model->k5 * speed_rpm * speed_rpm;
}

static float ChassisPower_FindAttenuation(const ChassisPowerModel_t *model,
                                          float desired_current,
                                          float motor_rad_s,
                                          float motor_limit_w)
{
    float current;
    float speed_rpm;
    float a;
    float b;
    float c;
    float discriminant;
    float candidates[4];
    uint8_t candidate_count = 0U;
    float best = 0.0f;

    if (model == 0 || model->current_conversion <= CHASSIS_POWER_EPSILON ||
        motor_limit_w <= 0.0f) return 0.0f;
    if (ChassisPower_Predict(model, desired_current, motor_rad_s) <= motor_limit_w) return 1.0f;
    if (fabsf(desired_current) <= CHASSIS_POWER_EPSILON) return 1.0f;

    current = fabsf(desired_current) / model->current_conversion;
    speed_rpm = fabsf(motor_rad_s) * CHASSIS_POWER_RAD_S_TO_RPM;
    a = model->k4 * current * current;
    b = (model->k1 + model->k3 * speed_rpm) * current;
    c = model->k0 + model->k2 * speed_rpm + model->k5 * speed_rpm * speed_rpm - motor_limit_w;

    candidates[candidate_count++] = 0.0f;
    candidates[candidate_count++] = 1.0f;
    if (fabsf(a) <= CHASSIS_POWER_EPSILON) {
        if (fabsf(b) > CHASSIS_POWER_EPSILON) candidates[candidate_count++] = -c / b;
    } else {
        discriminant = b * b - 4.0f * a * c;
        if (discriminant >= 0.0f) {
            float root = sqrtf(discriminant);
            candidates[candidate_count++] = (-b - root) / (2.0f * a);
            candidates[candidate_count++] = (-b + root) / (2.0f * a);
        }
    }

    for (uint8_t i = 0U; i < candidate_count; i++) {
        float attenuation = candidates[i];
        float predicted;
        if (attenuation < 0.0f || attenuation > 1.0f) continue;
        predicted = ChassisPower_Predict(model, desired_current * attenuation, motor_rad_s);
        if (predicted <= motor_limit_w + CHASSIS_POWER_EPSILON && attenuation > best) {
            best = attenuation;
        }
    }
    return best;
}

void ChassisPower_Init(ChassisPowerManager_t *manager)
{
    if (manager != 0) memset(manager, 0, sizeof(*manager));
}

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
                        float reserved_per_motor_w)
{
    float errors[CHASSIS_POWER_MOTOR_COUNT];
    float total_error = 0.0f;
    float budget;

    if (manager == 0 || model == 0 || desired_current == 0 ||
        target_motor_rad_s == 0 || feedback_motor_rad_s == 0) return;

    ChassisPower_Init(manager);
    manager->enabled = 1U;
    manager->requested_limit_w = total_limit_w;
    budget = total_limit_w * (1.0f - ChassisPower_Clamp(compensation, 0.0f, 0.99f)) *
             ChassisPower_Clamp(buffer_attenuation, 0.0f, 1.0f);
    manager->budget_w = budget;

    for (uint8_t i = 0U; i < CHASSIS_POWER_MOTOR_COUNT; i++) {
        errors[i] = fabsf(target_motor_rad_s[i] - feedback_motor_rad_s[i]) *
                    CHASSIS_POWER_RAD_S_TO_RPM;
        total_error += errors[i];
        manager->motor_before_w[i] = ChassisPower_Predict(model, desired_current[i],
                                                            feedback_motor_rad_s[i]);
        manager->predicted_before_w += manager->motor_before_w[i];
    }

    if (budget > 0.0f && total_error <= error_equal_rpm) {
        for (uint8_t i = 0U; i < CHASSIS_POWER_MOTOR_COUNT; i++) {
            manager->motor_limit_w[i] = budget / (float)CHASSIS_POWER_MOTOR_COUNT;
        }
    } else if (budget > 0.0f && total_error > CHASSIS_POWER_EPSILON) {
        float shared_budget = budget;
        float base_power = 0.0f;
        if (budget >= reserved_border_w) {
            base_power = reserved_per_motor_w;
            shared_budget -= base_power * (float)CHASSIS_POWER_MOTOR_COUNT;
            if (shared_budget < 0.0f) shared_budget = 0.0f;
        }
        for (uint8_t i = 0U; i < CHASSIS_POWER_MOTOR_COUNT; i++) {
            manager->motor_limit_w[i] = base_power + shared_budget * errors[i] / total_error;
        }
    }

    for (uint8_t i = 0U; i < CHASSIS_POWER_MOTOR_COUNT; i++) {
        manager->attenuation[i] = ChassisPower_FindAttenuation(model, desired_current[i],
                                                                 feedback_motor_rad_s[i],
                                                                 manager->motor_limit_w[i]);
        desired_current[i] *= manager->attenuation[i];
        manager->motor_after_w[i] = ChassisPower_Predict(model, desired_current[i],
                                                           feedback_motor_rad_s[i]);
        manager->predicted_after_w += manager->motor_after_w[i];
    }
}
