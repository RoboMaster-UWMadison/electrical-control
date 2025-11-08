#include "FreeRTOS.h"
#include "task.h"
#include "shoot.h"
#include "main.h"
#include "tim.h"
#include "cmsis_os.h"
#include "arm_math.h"
static fp32 speed_fliter_1[3] = {0.0f,0.0f,0.0f};
static fp32 speed_fliter_2[3] = {0.0f,0.0f,0.0f};
static fp32 speed_fliter_3[3] = {0.0f,0.0f,0.0f};
static int theta,output ;
static fp32 trigger_speed = TRIGGER_SPEED_slow;
static const fp32 fliter_num[3] = {1.725709860247969f, -0.75594777109163436f, 0.030237910843665373f};
/**
  * @brief          射击状态机设置，遥控器上拨一次开启，再上拨关闭，下拨1次发射1颗，一直处在下，则持续发射，用于3min准备时间清理子弹
  * @param[in]      void
  * @retval         void
  */
static void shoot_set_mode(void);
/**
  * @brief          射击数据更新
  * @param[in]      void
  * @retval         void
  */
static void shoot_feedback_update(void);

/**
  * @brief          堵转倒转处理
  * @param[in]      void
  * @retval         void
  */
static void trigger_motor_turn_back(void);
/**
  * @brief          开关弹舱
  * @param[in]      void
  * @retval         void
  */

shoot_control_t shoot_control;          //射击数据
int8_t load,last_load,load_flag;
int8_t preheat,last_preheat,preheat_flag;
void shoot_init(void)
{
  shoot_control.shoot_rc = get_remote_control_point();
  shoot_control.turret_motor_measure[2] = get_trigger_motor_measure_point();
  shoot_control.turret_motor_measure[1] = get_fric_motor_r_measure_point();
  shoot_control.turret_motor_measure[0] = get_fric_motor_l_measure_point();
  PID_Init(&shoot_control.trigger_motor_pid,9000.0f, 500.0f,0.0f,  TRIGGER_RPM_KP,TRIGGER_RPM_KI,TRIGGER_RPM_KD,  0.0f,0.0f,0.0f,0.0f, 0);
  PID_Init(&shoot_control.fric_motor_pid_l,30000.0f, 20000.0f,0.0f,  FRIC_RPM_KP,FRIC_RPM_KI,FRIC_RPM_KD,  0.0f,0.0f,0.0f,0.0f, 0);
  PID_Init(&shoot_control.fric_motor_pid_r,30000.0f, 20000.0f,0.0f,  FRIC_RPM_KP,FRIC_RPM_KI,FRIC_RPM_KD,  0.0f,0.0f,0.0f,0.0f, 0);
  shoot_feedback_update();
  for(uint8_t i=0;i<=2;i++)
  {
  shoot_control.given_current[i] = 0;
  shoot_control.speed[i] = 0.0f;
  shoot_control.speed_set[i] = 0.0f;
  }
  
  load = 0;
  load_flag = 1;
  preheat = 0;
  preheat_flag = 1;
}

void shoot_task(void const *pvParameters)
{
  osDelay(5);
  shoot_init();
  int16_t* current_turret;
  
  while(1)
  {
    current_turret = shoot_control_loop();
    CAN_cmd_TURRET(current_turret[0],current_turret[1],current_turret[2]);
    last_load = load;
    load = shoot_control.shoot_rc->key.r;

    if(shoot_control.shoot_rc->rc.s[1] == 2 &&  shoot_control.shoot_rc->rc.ch[4] <= -500)
    {
      theta = 90;
      output = 500 - (theta - 180)* 2000/180 ;
      __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_1, output);
    }
    else
    {
      theta = 0;
      output = 500 - (theta - 180)* 2000/180 ;
      __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_1, output);
    }
      if(load  - last_load == 1)
    {
      load_flag*=-1;

      
    }
    if( shoot_control.shoot_rc->rc.s[1] != 2)
    {
    if(load_flag == -1)
     {
        theta = 90;
        output = 500 - (theta - 180)* 2000/180 ;
        __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_1, output);
     }
    else
    {
      theta = 0;
      output = 500 - (theta - 180)* 2000/180 ;
      __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_1, output);
    }
    }
    osDelay(2);
  }

}
/**
  * @brief          射击循环
  * @param[in]      void
  * @retval         返回can控制值
  */
int16_t* shoot_control_loop(void)
{

    shoot_set_mode();        //设置状态机
    shoot_feedback_update(); //更新数据
    if(shoot_control.shoot_state == SHOOT_LAUNCH)
    {
      shoot_control.speed_set[2] = trigger_speed;
      trigger_motor_turn_back();      
      shoot_control.given_current[2] = PID_Calculate_Delta(&shoot_control.trigger_motor_pid,shoot_control.speed[2],shoot_control.speed_set[2]);
      volt.Volt_Data.Data[0] = shoot_control.speed[2];
      volt.Volt_Data.Data[1] = shoot_control.speed_set[2];        
      Volt_Sendware();
      osDelay(1);
      shoot_control.speed_set[0] = -FRIC_SPEED;
      shoot_control.given_current[0] = PID_Calculate_Delta(&shoot_control.fric_motor_pid_l,shoot_control.speed[0],shoot_control.speed_set[0]);      
      shoot_control.speed_set[1] = FRIC_SPEED;      
      shoot_control.given_current[1] = PID_Calculate_Delta(&shoot_control.fric_motor_pid_r,shoot_control.speed[1],shoot_control.speed_set[1]);
    }
    else if(shoot_control.shoot_state == SHOOT_READY)
    {
      shoot_control.given_current[2] = 0.0f;
      shoot_control.speed_set[0] = -FRIC_SPEED;
      shoot_control.given_current[0] = PID_Calculate_Delta(&shoot_control.fric_motor_pid_l,shoot_control.speed[0],shoot_control.speed_set[0]);      
      shoot_control.speed_set[1] = FRIC_SPEED;      
      shoot_control.given_current[1] = PID_Calculate_Delta(&shoot_control.fric_motor_pid_r,shoot_control.speed[1],shoot_control.speed_set[1]);
      
//      
      
    }
    else if(shoot_control.shoot_state == SHOOT_STOP)
    {
      shoot_control.given_current[2] = 0.0f;
       shoot_control.speed_set[0] = 0.0f;
      shoot_control.given_current[0] = PID_Calculate_Delta(&shoot_control.fric_motor_pid_l,shoot_control.speed[0],shoot_control.speed_set[0]);      
      shoot_control.speed_set[1] = 0.0f;      
      shoot_control.given_current[1] = PID_Calculate_Delta(&shoot_control.fric_motor_pid_r,shoot_control.speed[1],shoot_control.speed_set[1]);
//   
    }
    
    return shoot_control.given_current;
}

static void shoot_feedback_update(void)
{
  //二阶低通滤波
    for(uint8_t i=0;i<=2;i++)
    {
      speed_fliter_1[i] = speed_fliter_2[i];
      speed_fliter_2[i] = speed_fliter_3[i];
      speed_fliter_3[i] = speed_fliter_2[i] * fliter_num[0] + 
                          speed_fliter_1[i] * fliter_num[1] + 
                         (shoot_control.turret_motor_measure[i]->speed_rpm * MOTOR_RPM_TO_SPEED) * fliter_num[2];
      shoot_control.speed[i] = speed_fliter_3[i]; 
    }  
}

static void shoot_set_mode(void)
{

    last_preheat = preheat;
  preheat = shoot_control.shoot_rc->key.c;
  if(preheat  - last_preheat == 1)
    {

      preheat_flag*=-1;
    }  
  if(preheat_flag == 1)
   {
      shoot_control.shoot_state= SHOOT_STOP;
    }      
  else
    {
      shoot_control.shoot_state= SHOOT_READY;
    }
    
      if(shoot_control.shoot_rc->rc.s[0] == 1)
  {
      shoot_control.shoot_state= SHOOT_LAUNCH;

  }
  else if(shoot_control.shoot_rc->rc.s[0] == 3)
  {
      shoot_control.shoot_state= SHOOT_READY;
        if(shoot_control.shoot_rc->mouse.press_l != 0 )
    {
      trigger_speed = TRIGGER_SPEED_slow;
      shoot_control.shoot_state= SHOOT_LAUNCH;
    }
    else if(shoot_control.shoot_rc->mouse.press_r != 0 )
    {
      trigger_speed = TRIGGER_SPEED_fast;
      shoot_control.shoot_state= SHOOT_LAUNCH;
    }
  }
  else
  {
       shoot_control.shoot_state= SHOOT_STOP;
  }
  
  
}

static void trigger_motor_turn_back(void)
{
    if( shoot_control.block_time < BLOCK_TIME)
    {
        shoot_control.speed_set[2] = shoot_control.speed_set[2];
    }
    else
    {
        shoot_control.speed_set[2] = 3.0f;
    }

    if(fabs(shoot_control.speed[2]) < BLOCK_TRIGGER_SPEED && shoot_control.block_time < BLOCK_TIME)
    {
        shoot_control.block_time++;
        shoot_control.reverse_time = 0;
    }
    else if (shoot_control.block_time == BLOCK_TIME && shoot_control.reverse_time < REVERSE_TIME)
    {
        shoot_control.reverse_time++;
    }
    else
    {
        shoot_control.block_time = 0;
    }
}






















