#ifndef SLAM_H
#define SLAM_H

#include <stdint.h>

#define SLAM_RAW_FRAME_MAX_SIZE 15U

/* SLAM 模块输出的局部位姿。位置单位为米，姿态单位为度。 */
typedef struct {
    /* 保留模块输出的原始位置计数，供专用调试网页核对实际比例与坐标轴。 */
    int16_t x_count;
    int16_t y_count;
    int16_t z_count;
    int16_t roll_count;
    int16_t pitch_count;
    int16_t yaw_count;
    float x_m;
    float y_m;
    float z_m;
    float roll_deg;
    float pitch_deg;
    float yaw_deg;
    uint32_t last_update_ms;
    uint32_t valid_frames;
    uint32_t format_errors;
    uint32_t uart_errors;
    uint32_t rx_bytes;
    uint32_t rx_overflow;
    uint8_t raw_frame[SLAM_RAW_FRAME_MAX_SIZE];
    uint8_t raw_length;
    uint8_t online;
} SlamPose_t;

/* 初始化解析器与接收队列。 */
void Slam_Init(void);

/* UART7 接收中断调用：仅将原始字节写入队列，不参与控制。 */
void Slam_OnUartRx(const uint8_t *data, uint16_t length);

/* UART7 发生硬件接收错误时记录一次。 */
void Slam_OnUartError(void);

/* Slam 任务调用：解析队列中的固定 14 字节位姿帧。 */
void Slam_ProcessRx(void);

/* 更新在线状态；超过超时时间未收到完整帧即离线。 */
void Slam_Update(void);

/* 原子复制当前位姿，供遥测或后续导航模块读取。 */
void Slam_GetPose(SlamPose_t *pose);

#endif /* SLAM_H */
