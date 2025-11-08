#ifndef BSP_BUZZER_H
#define BSP_BUZZER_H
#include "struct_typedef.h"
#define MAX_PSC                 1000

#define MAX_BUZZER_PWM      20000
#define MIN_BUZZER_PWM      10000


// 设定用于映射的系统时钟（单位：Hz）
#define TIMER_CLK 84000000
#define BUZZER_PERIOD 20999  // 固定ARR = 1000（由pwm设置）

// 简单音符频率表（C大调，单位Hz）
typedef struct {
    uint16_t freq;
    uint16_t duration; // ms
} Note;



#define MELODY_LEN (sizeof(melody_max) / sizeof(Note))
extern void buzzer_on(uint16_t psc, uint16_t pwm);
extern void buzzer_off(void);
extern uint16_t freq_to_psc(uint16_t freq);
extern void play_max(void);
#endif
