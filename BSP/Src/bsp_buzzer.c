#include "bsp_buzzer.h"

#include "tim.h"

#define BUZZER_TIMER_CLOCK_HZ 2000000U
#define BUZZER_DEFAULT_DUTY_PERCENT 50U

typedef enum {
    BUZZER_SEQUENCE_IDLE = 0,
    BUZZER_SEQUENCE_STARTUP_FIRST_TONE,
    BUZZER_SEQUENCE_STARTUP_GAP,
    BUZZER_SEQUENCE_STARTUP_SECOND_TONE,
} BuzzerSequence_t;

static uint32_t buzzer_deadline_ms;
static BuzzerSequence_t buzzer_sequence;

static uint8_t BuzzerDeadlineReached(uint32_t now, uint32_t deadline)
{
    return (uint8_t)((int32_t)(now - deadline) >= 0);
}

void BSP_Buzzer_Init(void)
{
    (void)HAL_TIM_PWM_Start(&htim12, TIM_CHANNEL_2);
    BSP_Buzzer_Off();
    buzzer_sequence = BUZZER_SEQUENCE_IDLE;
}

void BSP_Buzzer_Off(void)
{
    /* 保持 TIM12 运行，仅将占空比置零；该板子的 PWM 输出需要这样才能
       在下一次提示音时正常恢复。 */
    __HAL_TIM_SET_COMPARE(&htim12, TIM_CHANNEL_2, 0U);
    buzzer_deadline_ms = 0U;
    buzzer_sequence = BUZZER_SEQUENCE_IDLE;
}

void BSP_Buzzer_PlayTone(uint16_t frequency_hz, uint16_t duration_ms)
{
    uint32_t period;

    if (frequency_hz == 0U) {
        BSP_Buzzer_Off();
        return;
    }

    /* A direct prompt interrupts any previous prompt sequence. */
    buzzer_sequence = BUZZER_SEQUENCE_IDLE;

    period = BUZZER_TIMER_CLOCK_HZ / frequency_hz;
    if (period < 2U) period = 2U;

    __HAL_TIM_DISABLE(&htim12);
    __HAL_TIM_SET_AUTORELOAD(&htim12, period - 1U);
    __HAL_TIM_SET_COMPARE(&htim12, TIM_CHANNEL_2,
                          (period * BUZZER_DEFAULT_DUTY_PERCENT) / 100U);
    __HAL_TIM_SET_COUNTER(&htim12, 0U);
    __HAL_TIM_ENABLE(&htim12);

    buzzer_deadline_ms = HAL_GetTick() + duration_ms;
}

void BSP_Buzzer_PlayStartup(void)
{
    BSP_Buzzer_Off();
    buzzer_sequence = BUZZER_SEQUENCE_STARTUP_FIRST_TONE;
    buzzer_deadline_ms = HAL_GetTick();
}

void BSP_Buzzer_Process(void)
{
    uint32_t now = HAL_GetTick();

    if (!BuzzerDeadlineReached(now, buzzer_deadline_ms)) return;

    switch (buzzer_sequence) {
        case BUZZER_SEQUENCE_STARTUP_FIRST_TONE:
            BSP_Buzzer_PlayTone(3000U, 100U);
            buzzer_sequence = BUZZER_SEQUENCE_STARTUP_GAP;
            break;
        case BUZZER_SEQUENCE_STARTUP_GAP:
            BSP_Buzzer_Off();
            buzzer_deadline_ms = now + 80U;
            buzzer_sequence = BUZZER_SEQUENCE_STARTUP_SECOND_TONE;
            break;
        case BUZZER_SEQUENCE_STARTUP_SECOND_TONE:
            BSP_Buzzer_PlayTone(3600U, 160U);
            buzzer_sequence = BUZZER_SEQUENCE_IDLE;
            break;
        case BUZZER_SEQUENCE_IDLE:
        default:
            BSP_Buzzer_Off();
            break;
    }
}
