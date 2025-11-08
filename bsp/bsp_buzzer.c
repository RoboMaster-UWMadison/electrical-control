#include "bsp_buzzer.h"
#include "cmsis_os.h"
#include "main.h"
extern TIM_HandleTypeDef htim4;


Note melody_max[] = {
    {110, 300}, // 6 = La (A3)
    {110, 300}, // 6 = La (A3)
    {110, 200}, // 6 = La (A3) 
    {165, 450}, // 3 = Mi (E2)
    {0,  300},
    {165,  400}, // 3 = Mi (E2)
    {165,  200}, // 3 = Mi (E2)
    {131, 360},  // 1 = Do (C3)
    {123, 360}, // 7 = Si (B2)
    
};
uint16_t freq_to_psc(uint16_t freq)
{
    return (TIMER_CLK / (BUZZER_PERIOD * freq)) - 1;
}
void buzzer_on(uint16_t psc, uint16_t pwm)
{
    __HAL_TIM_PRESCALER(&htim4, psc);
    __HAL_TIM_SetCompare(&htim4, TIM_CHANNEL_3, pwm);

}
void buzzer_off(void)
{
    __HAL_TIM_SetCompare(&htim4, TIM_CHANNEL_3, 0);
}

void play_max(void)
{
  uint16_t psc = 0;
  uint16_t pwm = MIN_BUZZER_PWM;
  for(int j = 0 ;j<1;j++)
  {
    for (int i = 0; i < MELODY_LEN; i++)
{
    if (melody_max[i].freq == 0)
    {
        buzzer_off();                         // 停顿音：直接关闭蜂鸣器
        HAL_Delay(melody_max[i].duration);        // 停顿时间
    }
    else
    {
        psc = freq_to_psc(melody_max[i].freq*2);    // 只有在 freq != 0 时才调用
        pwm = BUZZER_PERIOD / 4;
        buzzer_on(psc, pwm);
        HAL_Delay(melody_max[i].duration);
        buzzer_off();
        osDelay(30);                        // 音符间断音
    }
}
buzzer_off();
  }
}
