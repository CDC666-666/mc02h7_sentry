#ifndef EXTRA_IMU_H
#define EXTRA_IMU_H

#include <stdbool.h>
#include <stdint.h>

/*
 * 顶部额外 IMU 的统一状态。
 * 本模块仅保存 DM-IMU-L1 的 CAN 数据，不向云台、电机或 INS 写入控制量。
 */
typedef struct {
    float acceleration_mps2[3];
    float gyro_rad_s[3];
    float pitch_deg;
    float yaw_deg;
    float roll_deg;
    float quaternion[4];
    float temperature_c;
    uint32_t last_update_ms;
    uint32_t last_gyro_update_ms;
    uint32_t last_euler_update_ms;
    uint32_t last_identifier;
    uint32_t valid_frames;
    uint32_t invalid_frames;
    uint8_t frame_mask;
    uint8_t last_data_type;
    uint8_t last_data[8];
} ExtraImu_t;

extern volatile ExtraImu_t extra_imu;
extern volatile uint32_t ExtraImu_RequestCount;
extern volatile uint32_t ExtraImu_RequestRejectedCount;

/* CAN 接收总入口调用：只有匹配当前额外 IMU 配置的帧才会被消耗。 */
bool ExtraImu_CanReceive(uint8_t can_index, uint32_t identifier, const uint8_t data[8]);
bool ExtraImu_IsOnline(void);
bool ExtraImu_RequestNext(void);
bool ExtraImu_HasYawReference(void);
float ExtraImu_GetYawRateCompensated(void);
float ExtraImu_GetYawBias(void);

#endif /* EXTRA_IMU_H */
