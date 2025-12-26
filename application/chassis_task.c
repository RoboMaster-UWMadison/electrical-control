#include "arm_math.h"
#include "chassis_task.h"
#include "chassis_behaviour.h"
#include "cmsis_os.h"
#include "detect_task.h"
#include "INS_task.h"
#include "motor_CAN.h"
#include "pid.h"
#include "pid_dji.h"
#include "remote_control.h"
#include "volt.h"

#define rc_deadband_limit(input, output, dealine)        \
    {                                                    \
        if ((input) > (dealine) || (input) < -(dealine)) \
        {                                                \
            (output) = (input);                          \
        }                                                \
        else                                             \
        {                                                \
            (output) = 0;                                \
        }                                                \
    }

/**
  * @brief         	Initialize chassis_move, including the initialization of pid, remote control pointer, 
  * 				3508 chassis motor pointer, and INS_angle pointer.
  * 				初始化"chassis_move"变量，包括pid初始化， 遥控器指针初始化，3508底盘电机指针初始化，陀螺仪角度指针初始化
  * @param[out]     chassis_move_init: Output chassis_move_t struct pointer initialized
  */
static void chassis_init(chassis_move_t *chassis_move_init);

/**
  * @brief          Set chassis control mode by changing "chassis_bahaviour_mode_set" variable.
  *                 设置底盘控制模式，主要在'chassis_behaviour_mode_set'函数中改变
  * @param[out]     chassis_move_mode: pointer to the "chassis_move" structure.
  *                 "chassis_move"变量指针.
  * @retval         none
  */
static void chassis_set_mode(chassis_move_t *chassis_move_mode);
/**
  * @brief          Change necessary parameters after chassis mode changes, such as setting the yaw
  *                 setpoint to current yaw angle.
  *                 底盘模式改变，有些参数需要改变，例如底盘控制yaw角度设定值应该变成当前底盘yaw角度
  * @param[out]     chassis_move_transit: pointer to the "chassis_move" structure.
  *                 "chassis_move"变量指针.
  * @retval         none
  */
void chassis_mode_change_control_transit(chassis_move_t *chassis_move_transit);
/**
  * @brief          Update chassis feedback data, including motor speeds, Euler angles, and robot velocity.
  *                 底盘测量数据更新，包括电机速度，欧拉角度，机器人速度
  * @param[out]     chassis_move_update: pointer to the "chassis_move" structure.
  *                 "chassis_move"变量指针.
  * @retval         none
  */
static void chassis_feedback_update(chassis_move_t *chassis_move_update);
/**
  * @brief          ?
  * @param[out]     chassis_move_update: pointer to the "chassis_move" structure.
  *                 "chassis_move"变量指针.
  * @retval         none
  */
static void chassis_set_control(chassis_move_t *chassis_move_control);
/**
  * @brief          Chassis control loop, calculate motor current based on control setpoints.
  *                 控制循环，根据控制设定值，计算电机电流值，进行控制
  * @param[out]     chassis_move_control_loop: pointer to the "chassis_move" structure.
  *                 "chassis_move"变量指针.
  * @retval         none
  */
static void chassis_control_loop(chassis_move_t *chassis_move_control_loop);

/**
  * @brief          Global pointer to chassis current data.
  * @note           Initialized in chassis_init().
  */
chassis_move_t chassis_move;
//麦轮赋值矩阵
/**
  * @brief          Macanum wheel coupling matrix.
  * @note           Translate target movement to individual wheel speeds with 
  *                 Wheel_Speed = This_Matrix(4x3) X Chassis_Vector(Vx,Vy,angular_velocity).
  * @note           The matrix is stored as transpose so each row corresponds to an action.
  */
static fp32 M[3][4] = {{   -1.0f,   -1.0f,    1.0f,    1.0f},
                       {   -1.0f,    1.0f,    1.0f,   -1.0f},
                       {    2.5f,    2.5f,    2.5f,    2.5f}};

/** @brief          ?
  */
fp32 yaw_offset;

/**
  * @brief          ?
  */
static void vector_ground_convert(fp32 *vx_set, fp32 *vy_set, fp32 *angle);

/**
  * @brief          ?
  * @param[in]      ecd: ?
  * @param[in]      offset_ecd: ?
  * @retval         ?
  */
static fp32 motor_ecd_to_angle_change(uint16_t ecd, uint16_t offset_ecd);

static int8_t  q,                /**< boolean for chassis rotate (on/off) */
	           last_q,           /**< last chassis rotate mode */
	           q_flag;           /**< ? */
// TODO: move these to chassis_task.h?
// TODO: change to enum?
static fp32    gear_xy,          /**< Index in gear_xylevel for upper speed limit of translational movement. */
               gear_z;           /**< Index in gear_zlevel for upper speed limit of orientational movement. */
// TODO: set values for now, can change to dynamic based on competition API later
static fp32    gear_xylevel[3],  /**< Stores corresponding translational speed limit for each of 3 mode. */
               gear_zlevel[3];   /**< Stores corresponding orientational speed limit for each of 3 mode. */

/**
  * @brief          Chassis task, runs at an interval of CHASSIS_CONTROL_TIME_MS (2ms).
  *                 底盘任务，间隔 CHASSIS_CONTROL_TIME_MS 2ms
  * @param[in]      pvParameters: None
  * @retval         none
  */
void chassis_task(void const *pvParameters)
{
    //空闲一段时间
    vTaskDelay(CHASSIS_TASK_INIT_TIME);
    //底盘初始化
    chassis_init(&chassis_move);
    while (1)
    {
      //设置底盘控制模式
       chassis_set_mode(&chassis_move);
      //底盘数据更新
       chassis_feedback_update(&chassis_move);
      //设置控制量
       chassis_set_control(&chassis_move);
      //底盘控制PID计算
       chassis_control_loop(&chassis_move);
       CAN_cmd_chassis(chassis_move.motor_chassis[0].give_current, chassis_move.motor_chassis[1].give_current,
                       chassis_move.motor_chassis[2].give_current, chassis_move.motor_chassis[3].give_current);
       vTaskDelay(1);
      
    }    
}
/**
  * @brief         	Initialize chassis_move, including the initialization of pid, remote control pointer, 
	* 												3508 chassis motor pointer, INS_angle
	* 								初始化"chassis_move"变量，包括pid初始化， 遥控器指针初始化，3508底盘电机指针初始化，陀螺仪角度指针初始化
  * @param[out]     chassis_move_init Output chassis_move_t struct pointer initialized
  */
static void chassis_init(chassis_move_t *chassis_move_init)
{
    if (chassis_move_init == NULL)
    {
        return;
    }
    // chassis motor speed pid
    const static fp32 motor_speed_pid[3] = {M3505_MOTOR_SPEED_PID_KP, M3505_MOTOR_SPEED_PID_KI, M3505_MOTOR_SPEED_PID_KD};    
    const static fp32 motor_current_pid[3] = {M3505_MOTOR_CURRENT_PID_KP,M3505_MOTOR_CURRENT_PID_KI, M3505_MOTOR_CURRENT_PID_KD};  
    const static fp32 chassis_x_order_filter[1] = {CHASSIS_ACCEL_X_NUM};
    const static fp32 chassis_y_order_filter[1] = {CHASSIS_ACCEL_Y_NUM};
    uint8_t i;
    chassis_move_init->chassis_mode = CHASSIS_ZERO_FORCE;
    // get chassis remote control pointer
    chassis_move_init->chassis_RC = get_remote_control_point();
    // get gyroscrope attitude angle pointer as the gimbal yaw angle
    chassis_move_init->chassis_INS_angle = get_yaw_gimbal_motor_measure_point();   
    // get chassis motor data pointer and initialize its pid
    for (i = 0; i < 4; i++)
    {
      chassis_move_init->motor_chassis[i].chassis_motor_measure = get_chassis_motor_measure_point(i);          
      PID_Init(&chassis_move_init->motor_speed_pid[i],M3505_MOTOR_SPEED_PID_MAX_OUT,M3505_MOTOR_SPEED_PID_MAX_IOUT,
      0.0f,motor_speed_pid[0],motor_speed_pid[1], motor_speed_pid[2],0.0f,0.0f,0.0f,0.0f, 0);
      PID_init(&chassis_move_init->motor_current_pid[i],0,motor_current_pid,30000.0f,5000.0f);
    }
    //初始化角度PID
    
    //用一阶滤波代替斜波函数生成
    first_order_filter_init(&chassis_move_init->chassis_cmd_slow_set_vx, CHASSIS_CONTROL_TIME, chassis_x_order_filter);
    first_order_filter_init(&chassis_move_init->chassis_cmd_slow_set_vy, CHASSIS_CONTROL_TIME, chassis_y_order_filter);
    //最大 最小速度
    chassis_move_init->vx_max_speed = NORMAL_MAX_CHASSIS_SPEED_X;
    chassis_move_init->vx_min_speed = -NORMAL_MAX_CHASSIS_SPEED_X;

    chassis_move_init->vy_max_speed = NORMAL_MAX_CHASSIS_SPEED_Y;
    chassis_move_init->vy_min_speed = -NORMAL_MAX_CHASSIS_SPEED_Y;
    //更新一下数据
    q = 0;
    q_flag = 1;
    gear_zlevel[0] = 0.3f;
    gear_zlevel[1] = 0.5f;
    gear_zlevel[2] = 0.6f;
    gear_z = gear_zlevel[0];
    gear_xylevel[0] = 1.0f;
    gear_xylevel[1] = 1.25f;
    gear_xylevel[2] = 1.5f;
    gear_xy = gear_xylevel[0];
    chassis_feedback_update(chassis_move_init);
}
/**
  * @brief          底盘测量数据更新，包括电机速度，欧拉角度，机器人速度
  * @param[out]     chassis_move_update:"chassis_move"变量指针.
  * @retval         none
  */
static void chassis_feedback_update(chassis_move_t *chassis_move_update)
{
    if (chassis_move_update == NULL)
    {
        return;
    }

    uint8_t i = 0;
    for (i = 0; i < 4; i++)
    {
        //更新电机速度，加速度是速度的PID微分
        chassis_move_update->motor_chassis[i].speed = CHASSIS_MOTOR_RPM_TO_VECTOR_SEN * chassis_move_update->motor_chassis[i].chassis_motor_measure->speed_rpm;
        chassis_move_update->motor_chassis[i].accel = chassis_move_update->motor_speed_pid[i].Dbuf[0] * CHASSIS_CONTROL_FREQUENCE;
        chassis_move_update->motor_chassis[i].give_current = chassis_move_update->motor_chassis[i].chassis_motor_measure->given_current;
    }
    chassis_move_update->vx = (-chassis_move_update->motor_chassis[0].speed - chassis_move_update->motor_chassis[1].speed + chassis_move_update->motor_chassis[2].speed + chassis_move_update->motor_chassis[3].speed) * MOTOR_SPEED_TO_CHASSIS_SPEED_VX;
    chassis_move_update->vy =  (chassis_move_update->motor_chassis[0].speed - chassis_move_update->motor_chassis[1].speed - chassis_move_update->motor_chassis[2].speed + chassis_move_update->motor_chassis[3].speed) * MOTOR_SPEED_TO_CHASSIS_SPEED_VY;
    chassis_move_update->wz =  (chassis_move_update->motor_chassis[0].speed + chassis_move_update->motor_chassis[1].speed + chassis_move_update->motor_chassis[2].speed + chassis_move_update->motor_chassis[3].speed) * MOTOR_SPEED_TO_CHASSIS_SPEED_WZ / MOTOR_DISTANCE_TO_CENTER;
      
    chassis_move_update->chassis_absolute_angle = motor_ecd_to_angle_change(chassis_move_update->chassis_INS_angle->ecd, 8191-6488);   
    
}
/**
  * @brief          设置底盘控制模式，主要在'chassis_behaviour_mode_set'函数中改变
  * @param[out]     chassis_move_mode:"chassis_move"变量指针.
  * @retval         none
  */
static void chassis_set_mode(chassis_move_t *chassis_move_mode)
{
    if (chassis_move_mode == NULL)
    {
        return;
    }
    chassis_behaviour_mode_set(chassis_move_mode);
}

static void chassis_set_control(chassis_move_t *chassis_move_control)
{

    if (chassis_move_control == NULL)
    {
        return;
    }
    fp32 vx_set = 0.0f, vy_set = 0.0f, wz_set = 0.0f;
    //获取三个控制设置值
    chassis_rc_to_control_vector(&vx_set, &vy_set, chassis_move_control);
    last_q = q;
    q = chassis_move_control->chassis_RC->key.q;
    if(q  - last_q == 1)
    {

      q_flag*=-1;
    }     
        
  if(q_flag == 1 && chassis_move_control->chassis_RC->rc.ch[4] == 0)
   {
      wz_set = 0.0f;
     if(chassis_move_control->chassis_RC->key.f == 1)
     {
       gear_z = gear_zlevel[0];
       gear_xy = gear_xylevel[0];
     }
     else if(chassis_move_control->chassis_RC->key.g == 1)
     {
        gear_z = gear_zlevel[1];
        gear_xy = gear_xylevel[1];
     }
     else if(chassis_move_control->chassis_RC->key.v == 1)
     {
        gear_z = gear_zlevel[2];
        gear_xy = gear_xylevel[2];
     }
    }      
   else if(q_flag == -1 && chassis_move_control->chassis_RC->rc.ch[4] == 0)
   {
      wz_set = gear_z;
     //无水平运动则增加转速
     if((chassis_move_control->chassis_RC->key.s || chassis_move_control->chassis_RC->key.a || chassis_move_control->chassis_RC->key.w || chassis_move_control->chassis_RC->key.d) == 0)
       {
        wz_set *= 2;
        }
   }
   else
   {
      wz_set = chassis_move_control->chassis_RC->rc.ch[4]*-0.0025f;
   }

    chassis_move_control->wz_set = wz_set;
    if (chassis_move_control->chassis_mode == CHASSIS_FOLLOW_GIMBAL)
    {
        vector_ground_convert(&vx_set, &vy_set, &chassis_move_control->chassis_absolute_angle);  

        chassis_move_control->vx_set = vx_set;
        chassis_move_control->vy_set = vy_set;
        if(chassis_move_control->vx_set != 0.0f && chassis_move_control->vy_set !=0.0f)
        {
          chassis_move_control->vx_set *= 0.7f;
          chassis_move_control->vy_set *= 0.7f;
        }
        chassis_move_control->vx_set = fp32_constrain(chassis_move_control->vx_set, chassis_move_control->vx_min_speed, chassis_move_control->vx_max_speed);
        chassis_move_control->vy_set = fp32_constrain(chassis_move_control->vy_set, chassis_move_control->vy_min_speed, chassis_move_control->vy_max_speed);
    }
    else if (chassis_move_control->chassis_mode == GIMBAL_FOLLOW_CHASSIS)
      
    {   
        chassis_move_control->vx_set = vx_set;
        chassis_move_control->vy_set = vy_set;
      if(chassis_move_control->vx_set != 0.0f && chassis_move_control->vy_set !=0.0f)
        {
          chassis_move_control->vx_set *= 0.7f;
          chassis_move_control->vy_set *= 0.7f;
        }
        chassis_move_control->vx_set = fp32_constrain(vx_set, chassis_move_control->vx_min_speed, chassis_move_control->vx_max_speed);
        chassis_move_control->vy_set = fp32_constrain(vy_set, chassis_move_control->vy_min_speed, chassis_move_control->vy_max_speed);
    }
    else if (chassis_move_control->chassis_mode == CHASSIS_ZERO_FORCE)
    {
        chassis_move_control->vx_set = 0.0;
        chassis_move_control->vy_set = 0.0;
        chassis_move_control->wz_set = 0.0;
    }
}
static void chassis_vector_to_mecanum_wheel_speed(const fp32 vx_set, const fp32 vy_set, const fp32 wz_set, fp32 wheel_speed[4])
{
    //旋转的时候， 由于云台靠前，所以是前面两轮 0 ，1 旋转的速度变慢， 后面两轮 2,3 旋转的速度变快
    fp32 vector[3] = {vx_set, vy_set,wz_set};
    for(uint8_t i=0;i<=3;i++)
    {
      for(uint8_t j=0;j<=2;j++)
      {
        wheel_speed[i] += vector[j] * M[j][i];
      }
    }
}
/**
  * @brief          控制循环，根据控制设定值，计算电机电流值，进行控制
  * @param[out]     chassis_move_control_loop:"chassis_move"变量指针.
  * @retval         none
  */
static void chassis_control_loop(chassis_move_t *chassis_move_control_loop)
{
    fp32 wheel_speed[4] = {0.0f, 0.0f, 0.0f, 0.0f};
    //麦轮运动分解
    chassis_vector_to_mecanum_wheel_speed(chassis_move_control_loop->vx_set,
                                          chassis_move_control_loop->vy_set, 
                                          chassis_move_control_loop->wz_set, 
                                          wheel_speed);

    if (chassis_move_control_loop->chassis_mode == CHASSIS_ZERO_FORCE)
    {        
        for (uint8_t i = 0; i < 4; i++)
        {
            chassis_move_control_loop->motor_chassis[i].give_current = 0;
            chassis_move_control_loop->motor_chassis[i].current_set = 0;          
        }
        chassis_move_control_loop->vx_set = 0.0;
        chassis_move_control_loop->vy_set = 0.0;
        chassis_move_control_loop->wz_set = 0.0;
        return;
    }
    else 
    {
      //计算轮子控制最大速度，并限制其最大速度
      for (uint8_t i = 0; i < 4; i++)
        {
          chassis_move_control_loop->motor_chassis[i].speed_set = wheel_speed[i];
        }
    //计算pid
      for (uint8_t i = 0; i < 4; i++)
        {
          chassis_move_control_loop->motor_chassis[i].current_set = PID_Calculate_Delta(&chassis_move_control_loop->motor_speed_pid[i], 
                                                                                  chassis_move_control_loop->motor_chassis[i].speed, 
                                                                                  chassis_move_control_loop->motor_chassis[i].speed_set);
       
        }   //赋值电流值
      for (uint8_t i = 0; i < 4; i++)
        {
          chassis_move_control_loop->motor_chassis[i].give_current = (int16_t)(chassis_move_control_loop->motor_chassis[i].current_set);      
        }
    }
}
/**
  * @brief          根据遥控器通道值，计算纵向和横移速度
  *                 
  * @param[out]     vx_set: 纵向速度指针
  * @param[out]     vy_set: 横向速度指针
  * @param[out]     chassis_move_rc_to_vector: "chassis_move" 变量指针
  * @retval         none
  */

void chassis_rc_to_control_vector(fp32 *vx_set, fp32 *vy_set, chassis_move_t *chassis_move_rc_to_vector)
{
    if (chassis_move_rc_to_vector == NULL || vx_set == NULL || vy_set == NULL)
    {
        return;
    }
    
    int16_t vx_channel, vy_channel;
    fp32 vx_set_channel, vy_set_channel;
    //deadline, because some remote control need be calibrated,  the value of rocker is not zero in middle place,
    //死区限制，因为遥控器可能存在差异 摇杆在中间，其值不为0
    rc_deadband_limit(chassis_move_rc_to_vector->chassis_RC->rc.ch[CHASSIS_X_CHANNEL], vx_channel, CHASSIS_RC_DEADLINE);
    rc_deadband_limit(chassis_move_rc_to_vector->chassis_RC->rc.ch[CHASSIS_Y_CHANNEL], vy_channel, CHASSIS_RC_DEADLINE);

    

    //keyboard set speed set-point
    //键盘控制

      if (chassis_move_rc_to_vector->chassis_RC->key.s ==1)
    {
        vx_set_channel = gear_xy;
       if(chassis_move_rc_to_vector->wz_set == 0.0f)
        {
          vx_set_channel *= 1.1f;
        }
    }
    else if (chassis_move_rc_to_vector->chassis_RC->key.w==1)
    {
        vx_set_channel = -gear_xy;
      if(chassis_move_rc_to_vector->wz_set == 0.0f)
        {
          vx_set_channel *= 1.1f;
        }
    }
    else if(chassis_move_rc_to_vector->chassis_RC->rc.s[1] != 2)
    {
    vx_set_channel = vx_channel * -CHASSIS_VX_RC_SEN;
    }
    else
    {
      vx_set_channel = 0;
    }
    
    if (chassis_move_rc_to_vector->chassis_RC->key.d==1)
    {
        vy_set_channel = gear_xy;
        if(chassis_move_rc_to_vector->wz_set == 0.0f)
        {
          vy_set_channel *= 1.1f;
        }
    }
    else if (chassis_move_rc_to_vector->chassis_RC->key.a==1)
    {
        vy_set_channel = -gear_xy;
        if(chassis_move_rc_to_vector->wz_set == 0.0f)
        {
          vy_set_channel *= 1.1f;
        }
    }    
    else if(chassis_move_rc_to_vector->chassis_RC->rc.s[1] != 2)
    {
    vy_set_channel = vy_channel * CHASSIS_VY_RC_SEN;
    }
    else
    {
      vy_set_channel = 0;
    }
    *vx_set = vx_set_channel;
    *vy_set = vy_set_channel;
}

static void vector_ground_convert(fp32 *vx_set, fp32 *vy_set, fp32* angle)
{
   fp32 R[2][2];
   fp32 vl[2];
   vl[0] = *vx_set;
   vl[1] = *vy_set;
   R[0][0] = arm_cos_f32(*angle);  R[0][1] = -arm_sin_f32(*angle);
   R[1][0] =arm_sin_f32(*angle);  R[1][1] = arm_cos_f32(*angle);
   *vx_set = R[0][0]*vl[0] + R[0][1]*vl[1];
   *vy_set = R[1][0]*vl[0] + R[1][1]*vl[1];
  osDelay(2);
}

/**
  * @brief          计算ecd与offset_ecd之间的相对角度
  * @param[in]      ecd: 电机当前编码
  * @param[in]      offset_ecd: 电机中值编码
  * @retval         相对角度，单位rad
  */
static fp32 motor_ecd_to_angle_change(uint16_t ecd, uint16_t offset_ecd)
{
    int32_t relative_ecd = ecd - offset_ecd;
    if (relative_ecd > HALF_ECD_RANGE)
    {
        relative_ecd -= ECD_RANGE;
    }
    else if (relative_ecd < -HALF_ECD_RANGE)
    {
        relative_ecd += ECD_RANGE;
    }

    return relative_ecd * 0.000766990394f; //      2*  PI  /8192
}








