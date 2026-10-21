#ifndef BSP_BUZZER_H
#define BSP_BUZZER_H

#include <stdint.h>

void BSP_Buzzer_Init(void);
void BSP_Buzzer_PlayTone(uint16_t frequency_hz, uint16_t duration_ms);
void BSP_Buzzer_PlayStartup(void);
void BSP_Buzzer_Process(void);
void BSP_Buzzer_Off(void);

#endif /* BSP_BUZZER_H */
