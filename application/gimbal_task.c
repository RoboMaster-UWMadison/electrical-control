#include "gimbal_task.h"
#include "main.h"
#include "cmsis_os.h"
#include "arm_math.h"
#include "motor_CAN.h"
#include "user_lib.h"
#include "remote_control.h"
#include "INS_task.h"
#include "pid.h"
#include "pid_dji.h"
#include "gimbal_behaviour.h"
#include "detect_task.h"
#include "bsp_buzzer.h"
#include "miniPC_control.h"
#include "volt.h"
/**
  * @brief          初始化"gimbal_control"变量，包括pid初始化， 遥控器指针初始化，云台电机指针初始化，陀螺仪角度指针初始化
  * @param[out]     init:"gimbal_control"变量指针.
  * @retval         none
  */
static void gimbal_init(gimbal_control_t *init);
/**
  * @brief          设置云台控制模式，主要在'gimbal_behaviour_mode_set'函数中改变
  * @param[out]     gimbal_set_mode:"gimbal_control"变量指针.
  * @retval         none
  */
static void gimbal_set_mode(gimbal_control_t *set_mode);
/**
  * @brief          底盘测量数据更新，包括电机速度，欧拉角度，机器人速度
  * @param[out]     gimbal_feedback_update:"gimbal_control"变量指针.
  * @retval         none
  */
static void gimbal_feedback_update(gimbal_control_t *feedback_update);


/**
  * @brief          云台模式改变，有些参数需要改变，例如控制yaw角度设定值应该变成当前yaw角度
  * @param[out]     mode_change:"gimbal_control"变量指针.
  * @retval         none
  */
static void gimbal_mode_change_control_transit(gimbal_control_t *mode_change);

/**
  * @brief          计算ecd与offset_ecd之间的相对角度
  * @param[in]      ecd: 电机当前编码
  * @param[in]      offset_ecd: 电机中值编码
  * @retval         相对角度，单位rad
  */
static fp32 motor_ecd_to_angle_change(uint16_t ecd, uint16_t offset_ecd);

/**
  * @brief          设置云台控制设定值，控制值是通过gimbal_behaviour_control_set函数设置的
  * @param[out]     gimbal_set_control:"gimbal_control"变量指针.
  * @retval         none
  */
static void gimbal_set_control(gimbal_control_t *set_control);

/**
  * @brief          控制循环，根据控制设定值，计算电机电流值，进行控制
  * @param[out]     gimbal_control_loop:"gimbal_control"变量指针.
  * @retval         none
  */
static void gimbal_control_loop(gimbal_control_t *control_loop);
/**
  * @brief          云台角度PID计算, 因为角度范围在(-pi,pi)，不能用PID.c的PID
  * @param[out]     pid:云台PID指针
  * @param[in]      get: 角度反馈
  * @param[in]      set: 角度设定
  * @retval         pid 输出
  */
static float PID_Calculate_gimbal(PID_TypeDef *pid, float measure, float target);
/**
  * @brief          云台控制模式:GIMBAL_MOTOR_GYRO，使用陀螺仪计算的欧拉角进行控制
  * @param[out]     gimbal_motor:yaw电机或者pitch电机
  * @retval         none
  */
static void gimbal_absolute_angle_limit(gimbal_motor_t *gimbal_motor, fp32 add);
/**
  * @brief          云台控制模式:GIMBAL_MOTOR_ENCONDE，使用编码相对角进行控制
  * @param[out]     gimbal_motor:yaw电机或者pitch电机
  * @retval         none
  */
static void gimbal_relative_angle_limit(gimbal_motor_t *gimbal_motor, fp32 add);
//gimbal control data
//云台控制所有相关数据
gimbal_control_t gimbal_control;
fp32 pitch_g_offset=0;
//motor current 
//发送的电机电流
int16_t yaw_can_set_current = 0, pitch_can_set_current = 0;
extern fp32 INS_angle_fusion[3];
extern fp32  miniPC_pitch,miniPC_yaw,fire;
void gimbal_task(void const *pvParameters)
{
  vTaskDelay(200);
  gimbal_init(&gimbal_control);
  play_max();
  while(1)
  {
     gimbal_set_mode(&gimbal_control);                    //设置云台控制模式
     gimbal_mode_change_control_transit(&gimbal_control); //控制模式切换 控制数据过渡
     gimbal_feedback_update(&gimbal_control);             //云台数据反馈
     gimbal_set_control(&gimbal_control);                 //设置云台控制量
     gimbal_control_loop(&gimbal_control);                //云台控制PID计算
     yaw_can_set_current = gimbal_control.gimbal_yaw_motor.given_current;
     pitch_can_set_current =  gimbal_control.gimbal_pitch_motor.given_current;
     CAN_cmd_GIMBAL(yaw_can_set_current, pitch_can_set_current);          
       
     vTaskDelay(1);
  }
  
}

static void gimbal_init(gimbal_control_t *init)
{
   static const fp32 Pitch_speed_pid[3] = {PITCH_SPEED_PID_KP, PITCH_SPEED_PID_KI, PITCH_SPEED_PID_KD};
   static const fp32 Yaw_speed_pid[3] = {YAW_SPEED_PID_KP, YAW_SPEED_PID_KI, YAW_SPEED_PID_KD};
   //电机数据指针获取
    init->gimbal_yaw_motor.gimbal_motor_measure = get_yaw_gimbal_motor_measure_point();
    init->gimbal_pitch_motor.gimbal_motor_measure = get_pitch_gimbal_motor_measure_point();
    //陀螺仪数据指针获取
    init->gimbal_INT_angle_point = get_INS_angle_point();
    init->gimbal_INT_gyro_point = get_gyro_data_point();
    //遥控器数据指针获取
    init->gimbal_rc_ctrl = get_remote_control_point();
    //初始化电机模式
    init->gimbal_yaw_motor.gimbal_motor_mode = init->gimbal_yaw_motor.last_gimbal_motor_mode = GIMBAL_MOTOR_RAW;
    init->gimbal_pitch_motor.gimbal_motor_mode = init->gimbal_pitch_motor.last_gimbal_motor_mode = GIMBAL_MOTOR_RAW;
    //陀螺仪模式 PID
    PID_Init(&init->gimbal_yaw_motor.gimbal_motor_absolute_angle_pid,YAW_GYRO_ABSOLUTE_PID_MAX_OUT,YAW_GYRO_ABSOLUTE_PID_MAX_IOUT,
             0.0f,YAW_GYRO_ABSOLUTE_PID_KP,YAW_GYRO_ABSOLUTE_PID_KI, YAW_GYRO_ABSOLUTE_PID_KD, 
             0.0f,0.0f,0.0f,0.0f, 0);
    PID_Init(&init->gimbal_yaw_motor.gimbal_motor_CV_angle_pid,YAW_CV_MAX_OUT,YAW_CV_IOUT,
             0.0f,YAW_CV_KP,YAW_CV_KI, YAW_CV_KD, 
             0.0f,0.0f,0.0f,0.0f, 0);
    PID_Init(&init->gimbal_pitch_motor.gimbal_motor_absolute_angle_pid,PITCH_GYRO_ABSOLUTE_PID_MAX_OUT,YAW_GYRO_ABSOLUTE_PID_MAX_IOUT,
             0.0f,PITCH_GYRO_ABSOLUTE_PID_KP,PITCH_GYRO_ABSOLUTE_PID_KI, PITCH_GYRO_ABSOLUTE_PID_KD, 
             0.0f,0.0f,0.0f,0.0f, 0);
   //电机编码器模式 PID
    PID_Init(&init->gimbal_yaw_motor.gimbal_motor_relative_angle_pid,YAW_ENCODE_RELATIVE_PID_MAX_OUT,YAW_ENCODE_RELATIVE_PID_MAX_IOUT,
             0.0f,YAW_ENCODE_RELATIVE_PID_KP,YAW_ENCODE_RELATIVE_PID_KI, YAW_ENCODE_RELATIVE_PID_KD, 
             0.0f,0.0f,0.0f,0.0f, 0);
    PID_Init(&init->gimbal_pitch_motor.gimbal_motor_relative_angle_pid,PITCH_ENCODE_RELATIVE_PID_MAX_OUT,PITCH_ENCODE_RELATIVE_PID_MAX_IOUT,
             0.0f,PITCH_ENCODE_RELATIVE_PID_KP,PITCH_ENCODE_RELATIVE_PID_KI, PITCH_ENCODE_RELATIVE_PID_KD, 
             0.0f,0.0f,0.0f,0.0f, 0);
    PID_init(&init->gimbal_yaw_motor.gimbal_motor_gyro_pid, PID_DELTA, Yaw_speed_pid, YAW_SPEED_PID_MAX_OUT, YAW_SPEED_PID_MAX_IOUT);
    PID_init(&init->gimbal_pitch_motor.gimbal_motor_gyro_pid, PID_POSITION, Pitch_speed_pid, PITCH_SPEED_PID_MAX_OUT, PITCH_SPEED_PID_MAX_IOUT);
    init->gimbal_pitch_motor.max_relative_angle = 2.45f;
    init->gimbal_pitch_motor.min_relative_angle =  1.6f;
    init->gimbal_yaw_motor.max_relative_angle = 0.0f;
    init->gimbal_yaw_motor.min_relative_angle = 0.0f;
    gimbal_feedback_update(init);
    
    init->gimbal_yaw_motor.absolute_angle_set = init->gimbal_yaw_motor.absolute_angle;
    init->gimbal_yaw_motor.relative_angle_set = init->gimbal_yaw_motor.relative_angle;
    init->gimbal_yaw_motor.motor_gyro_set = init->gimbal_yaw_motor.motor_gyro;

    init->gimbal_pitch_motor.absolute_angle_set = init->gimbal_pitch_motor.relative_angle;
    init->gimbal_pitch_motor.relative_angle_set = init->gimbal_pitch_motor.relative_angle;
    init->gimbal_pitch_motor.motor_gyro_set = init->gimbal_pitch_motor.motor_gyro;
    init->pitch_g_offset = 0.0f;
    
}

static void gimbal_feedback_update(gimbal_control_t *feedback_update)
{
    if (feedback_update == NULL)
    {
        return;
    }
    //云台数据更新
    feedback_update->gimbal_pitch_motor.absolute_angle = *(feedback_update->gimbal_INT_angle_point + INS_PITCH_ADDRESS_OFFSET);
    feedback_update->gimbal_pitch_motor.relative_angle = motor_ecd_to_angle_change(feedback_update->gimbal_pitch_motor.gimbal_motor_measure->ecd,
                                                                                          feedback_update->gimbal_pitch_motor.offset_ecd);

    feedback_update->gimbal_pitch_motor.motor_gyro = *(feedback_update->gimbal_INT_gyro_point + INS_GYRO_Y_ADDRESS_OFFSET);
    feedback_update->gimbal_pitch_motor.motor_speed = feedback_update->gimbal_pitch_motor.gimbal_motor_measure->speed_rpm *  0.00290888208665721596153948461415f;
    //计算Pithch重力补偿
    feedback_update->pitch_g_offset = arm_cos_f32(INS_angle_fusion[0]+(70.0f/180.0f*3.1415f))* PITCH_G_OFFSET;
    
    feedback_update->gimbal_yaw_motor.absolute_angle = *(feedback_update->gimbal_INT_angle_point + INS_YAW_ADDRESS_OFFSET);
    feedback_update->gimbal_yaw_motor.relative_angle = motor_ecd_to_angle_change(feedback_update->gimbal_yaw_motor.gimbal_motor_measure->ecd,
                                                                                        feedback_update->gimbal_yaw_motor.offset_ecd);
    feedback_update->gimbal_yaw_motor.motor_gyro = arm_cos_f32(feedback_update->gimbal_pitch_motor.relative_angle) * (*(feedback_update->gimbal_INT_gyro_point + INS_GYRO_Z_ADDRESS_OFFSET))
                                                        - arm_sin_f32(feedback_update->gimbal_pitch_motor.relative_angle) * (*(feedback_update->gimbal_INT_gyro_point + INS_GYRO_X_ADDRESS_OFFSET));
    feedback_update->gimbal_yaw_motor.motor_speed = feedback_update->gimbal_yaw_motor.gimbal_motor_measure->speed_rpm * 0.00290888208665721596153948461415f;
    
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

static void gimbal_set_mode(gimbal_control_t *set_mode)
{
    if (set_mode == NULL)
    {
        return;
    }
    gimbal_behaviour_mode_set(set_mode);
}

static void gimbal_mode_change_control_transit(gimbal_control_t *gimbal_mode_change)
{
    if (gimbal_mode_change == NULL)
    {
        return;
    }
    //yaw电机状态机切换保存数据
    if (gimbal_mode_change->gimbal_yaw_motor.last_gimbal_motor_mode != GIMBAL_MOTOR_RAW && gimbal_mode_change->gimbal_yaw_motor.gimbal_motor_mode == GIMBAL_MOTOR_RAW)
    {
        gimbal_mode_change->gimbal_yaw_motor.raw_cmd_current = gimbal_mode_change->gimbal_yaw_motor.current_set = gimbal_mode_change->gimbal_yaw_motor.given_current;
    }
    else if (gimbal_mode_change->gimbal_yaw_motor.last_gimbal_motor_mode != GIMBAL_MOTOR_GYRO && gimbal_mode_change->gimbal_yaw_motor.gimbal_motor_mode == GIMBAL_MOTOR_GYRO)
    {
        gimbal_mode_change->gimbal_yaw_motor.absolute_angle_set = gimbal_mode_change->gimbal_yaw_motor.absolute_angle;
    }
    else if (gimbal_mode_change->gimbal_yaw_motor.last_gimbal_motor_mode != GIMBAL_MOTOR_ENCONDE && gimbal_mode_change->gimbal_yaw_motor.gimbal_motor_mode == GIMBAL_MOTOR_ENCONDE)
    {
        miniPC_yaw = gimbal_mode_change->gimbal_yaw_motor.absolute_angle;
    }
    gimbal_mode_change->gimbal_yaw_motor.last_gimbal_motor_mode = gimbal_mode_change->gimbal_yaw_motor.gimbal_motor_mode;

    //pitch电机状态机切换保存数据
    if (gimbal_mode_change->gimbal_pitch_motor.last_gimbal_motor_mode != GIMBAL_MOTOR_RAW && gimbal_mode_change->gimbal_pitch_motor.gimbal_motor_mode == GIMBAL_MOTOR_RAW)
    {
        gimbal_mode_change->gimbal_pitch_motor.raw_cmd_current = gimbal_mode_change->gimbal_pitch_motor.current_set = gimbal_mode_change->gimbal_pitch_motor.given_current;
    }
    else if (gimbal_mode_change->gimbal_pitch_motor.last_gimbal_motor_mode != GIMBAL_MOTOR_GYRO && gimbal_mode_change->gimbal_pitch_motor.gimbal_motor_mode == GIMBAL_MOTOR_GYRO)
    {
        gimbal_mode_change->gimbal_pitch_motor.absolute_angle_set = gimbal_mode_change->gimbal_pitch_motor.relative_angle;
    }
    else if (gimbal_mode_change->gimbal_pitch_motor.last_gimbal_motor_mode != GIMBAL_MOTOR_ENCONDE && gimbal_mode_change->gimbal_pitch_motor.gimbal_motor_mode == GIMBAL_MOTOR_ENCONDE)
    {
        miniPC_pitch = gimbal_mode_change->gimbal_pitch_motor.relative_angle;
    }

    gimbal_mode_change->gimbal_pitch_motor.last_gimbal_motor_mode = gimbal_mode_change->gimbal_pitch_motor.gimbal_motor_mode;
}
/**
  * @brief          云台控制模式:GIMBAL_MOTOR_GYRO，使用陀螺仪计算的欧拉角进行控制
  * @param[out]     gimbal_motor:yaw电机或者pitch电机
  * @retval         none
  */
static void gimbal_absolute_angle_limit(gimbal_motor_t *gimbal_motor, fp32 add)
{
    gimbal_motor->absolute_angle_set +=  add;
}
/**
  * @brief          云台控制模式:GIMBAL_MOTOR_ENCONDE，使用编码相对角进行控制
  * @param[out]     gimbal_motor:yaw电机或者pitch电机
  * @retval         none
  */
static void gimbal_relative_angle_limit(gimbal_motor_t *gimbal_motor, fp32 add)
{
    if (gimbal_motor == NULL)
    {
        return;
    }
    gimbal_motor->relative_angle_set += add;
    //是否超过最大 最小值
      if (gimbal_motor->relative_angle_set > gimbal_motor->max_relative_angle)
    {
        gimbal_motor->relative_angle_set = gimbal_motor->max_relative_angle;
    }
    else if (gimbal_motor->relative_angle_set < gimbal_motor->min_relative_angle)
    {
        gimbal_motor->relative_angle_set = gimbal_motor->min_relative_angle;
    }
    
}
/**
  * @brief          设置云台控制设定值，控制值是通过gimbal_behaviour_control_set函数设置的
  * @param[out]     gimbal_set_control:"gimbal_control"变量指针.
  * @retval         none
  */
static void gimbal_set_control(gimbal_control_t *set_control)
{
    if (set_control == NULL)
    {
        return;
    }

    fp32 add_yaw_angle = 0.0f;
    fp32 add_pitch_angle = 0.0f;

    gimbal_behaviour_control_set(&add_yaw_angle, &add_pitch_angle, set_control);
    //yaw电机模式控制
    if (set_control->gimbal_yaw_motor.gimbal_motor_mode == GIMBAL_MOTOR_RAW)
    {
        //raw模式下，直接发送控制值
        set_control->gimbal_yaw_motor.raw_cmd_current = add_yaw_angle;
    }
    else if (set_control->gimbal_yaw_motor.gimbal_motor_mode == GIMBAL_MOTOR_GYRO)
    {
        //gyro模式下，陀螺仪角度控制
        gimbal_absolute_angle_limit(&set_control->gimbal_yaw_motor, add_yaw_angle);
    }
    else if (set_control->gimbal_yaw_motor.gimbal_motor_mode == GIMBAL_MOTOR_ENCONDE)
    {

        set_control->gimbal_yaw_motor.absolute_angle_set = miniPC_yaw;
    }

    //pitch电机模式控制
    if (set_control->gimbal_pitch_motor.gimbal_motor_mode == GIMBAL_MOTOR_RAW)
    {
        //raw模式下，直接发送控制值
        set_control->gimbal_pitch_motor.raw_cmd_current = add_pitch_angle;
    }
    else if (set_control->gimbal_pitch_motor.gimbal_motor_mode == GIMBAL_MOTOR_GYRO)
    {
        //enconde模式下，电机编码角度控制
        gimbal_relative_angle_limit(&set_control->gimbal_pitch_motor, add_pitch_angle);
    }
    else if(set_control->gimbal_pitch_motor.gimbal_motor_mode == GIMBAL_MOTOR_ENCONDE )
    {
        set_control->gimbal_pitch_motor.relative_angle_set = miniPC_pitch;
        gimbal_relative_angle_limit(&set_control->gimbal_pitch_motor, 0);
    }
}
/**
  * @brief          云台控制模式:GIMBAL_MOTOR_RAW，电流值直接发送到CAN总线.
  * @param[out]     gimbal_motor:yaw电机或者pitch电机
  * @retval         none
  */
static void gimbal_motor_raw_angle_control(gimbal_motor_t *gimbal_motor)
{
    if (gimbal_motor == NULL)
    {
        return;
    }
    gimbal_motor->current_set = gimbal_motor->raw_cmd_current;
    gimbal_motor->given_current = (int16_t)(gimbal_motor->current_set);
}
/**
  * @brief          云台控制模式:GIMBAL_MOTOR_GYRO，使用陀螺仪计算的欧拉角进行控制
  * @param[out]     gimbal_motor:yaw电机或者pitch电机
  * @retval         none
  */
static void gimbal_motor_absolute_angle_control(gimbal_motor_t *gimbal_motor)
{
    if (gimbal_motor == NULL)
    {
        return;
    }
    //角度环，速度环串级pid调试
    gimbal_motor->motor_gyro_set = PID_Calculate(&gimbal_motor->gimbal_motor_absolute_angle_pid, gimbal_motor->absolute_angle, gimbal_motor->absolute_angle_set);
        
    gimbal_motor->current_set = PID_calc(&gimbal_motor->gimbal_motor_gyro_pid, gimbal_motor->motor_speed, gimbal_motor->motor_gyro_set);

    //控制值赋值
    gimbal_motor->given_current = (int16_t)(gimbal_motor->current_set);
}
/**
  * @brief          云台控制模式:GIMBAL_MOTOR_ENCONDE，使用编码相对角进行控制
  * @param[out]     gimbal_motor:yaw电机或者pitch电机
  * @retval         none
  */
static void gimbal_motor_relative_angle_control(gimbal_motor_t *gimbal_motor)
{
    if (gimbal_motor == NULL)
    {
        return;
    }

    //角度环，速度环串级pid调试
    gimbal_motor->current_set = PID_Calculate_gimbal(&gimbal_motor->gimbal_motor_relative_angle_pid, gimbal_motor->relative_angle, gimbal_motor->relative_angle_set);
    gimbal_motor->current_set = PID_calc(&gimbal_motor->gimbal_motor_gyro_pid, gimbal_motor->motor_speed, gimbal_motor->motor_gyro_set);
    //控制值赋值
    gimbal_motor->given_current = (int16_t)(gimbal_motor->current_set);

}
/**
  * @brief          控制循环，根据控制设定值，计算电机电流值，进行控制
  * @param[out]     gimbal_control_loop:"gimbal_control"变量指针.
  * @retval         none
  */
static void gimbal_control_loop(gimbal_control_t *control_loop)
{
    if (control_loop == NULL)
    {
        return;
    }
    
    
    if (control_loop->gimbal_yaw_motor.gimbal_motor_mode == GIMBAL_MOTOR_RAW)
    {
        gimbal_motor_raw_angle_control(&control_loop->gimbal_yaw_motor);
    }
    else if (control_loop->gimbal_yaw_motor.gimbal_motor_mode == GIMBAL_MOTOR_GYRO)
    {
        gimbal_motor_absolute_angle_control(&control_loop->gimbal_yaw_motor);
    }
    else if (control_loop->gimbal_yaw_motor.gimbal_motor_mode == GIMBAL_MOTOR_ENCONDE)
    {

     control_loop->gimbal_yaw_motor.motor_gyro_set = PID_Calculate(&control_loop->gimbal_yaw_motor.gimbal_motor_CV_angle_pid, control_loop->gimbal_yaw_motor.absolute_angle, control_loop->gimbal_yaw_motor.absolute_angle_set);
        
    control_loop->gimbal_yaw_motor.current_set = PID_calc(&control_loop->gimbal_yaw_motor.gimbal_motor_gyro_pid, control_loop->gimbal_yaw_motor.motor_speed, control_loop->gimbal_yaw_motor.motor_gyro_set);

    //控制值赋值
    control_loop->gimbal_yaw_motor.given_current = (int16_t)(control_loop->gimbal_yaw_motor.current_set);
    }

    if (control_loop->gimbal_pitch_motor.gimbal_motor_mode == GIMBAL_MOTOR_RAW)
    {
        gimbal_motor_raw_angle_control(&control_loop->gimbal_pitch_motor);
    }
    else if (control_loop->gimbal_pitch_motor.gimbal_motor_mode == GIMBAL_MOTOR_GYRO)
    {
        control_loop->gimbal_pitch_motor.given_current = (int16_t)(PID_Calculate(&control_loop->gimbal_pitch_motor.gimbal_motor_relative_angle_pid, 
                                                          control_loop->gimbal_pitch_motor.relative_angle, 
                                                          control_loop->gimbal_pitch_motor.relative_angle_set));
        
    }
    else if(control_loop->gimbal_pitch_motor.gimbal_motor_mode == GIMBAL_MOTOR_ENCONDE)
    {
      control_loop->gimbal_pitch_motor.given_current = (int16_t)(PID_Calculate(&control_loop->gimbal_pitch_motor.gimbal_motor_relative_angle_pid, 
                                                          control_loop->gimbal_pitch_motor.relative_angle, 
                                                          control_loop->gimbal_pitch_motor.relative_angle_set));          
       
    }
    
}
/*****************PID Improvement Function*********************/
static void f_Trapezoid_Intergral(PID_TypeDef *pid)
{
    pid->ITerm = pid->Ki * ((pid->Err + pid->Last_Err) / 2);
}

static void f_Changing_Integral_Rate(PID_TypeDef *pid)
{
    if (pid->Err * pid->Iout > 0)
    {
        //Integral still increasing
        if (ABS(pid->Err) <= pid->ScalarB)
            return; //Full integral
        if (ABS(pid->Err) <= (pid->ScalarA + pid->ScalarB))
            pid->ITerm *= (pid->ScalarA - ABS(pid->Err) + pid->ScalarB) / pid->ScalarA;
        else
            pid->ITerm = 0;
    }
}

static void f_Integral_Limit(PID_TypeDef *pid)
{
    float temp_Output, temp_Iout;
    temp_Iout = pid->Iout + pid->ITerm;
    temp_Output = pid->Pout + pid->Iout + pid->Dout;
    if (ABS(temp_Output) > pid->MaxOut)
    {
        if (pid->Err * pid->Iout > 0)
        {
            //Integral still increasing
            pid->ITerm = 0;
        }
    }

    if (temp_Iout > pid->IntegralLimit)
    {
        pid->ITerm = 0;
        pid->Iout = pid->IntegralLimit;
    }
    if (temp_Iout < -pid->IntegralLimit)
    {
        pid->ITerm = 0;
        pid->Iout = -pid->IntegralLimit;
    }
}

static void f_Derivative_On_Measurement(PID_TypeDef *pid)
{
    pid->Dout = pid->Kd * (pid->Last_Measure - pid->Measure);
}

static void f_Derivative_Filter(PID_TypeDef *pid)
{
    pid->Dout = pid->Dout * pid->Derivative_Filtering_Coefficient +
                pid->Last_Dout * (1 - pid->Derivative_Filtering_Coefficient);
}

static void f_Output_Filter(PID_TypeDef *pid)
{
    pid->Output = pid->Output * pid->Output_Filtering_Coefficient +
                  pid->Last_Output * (1 - pid->Output_Filtering_Coefficient);
}

static void f_Output_Limit(PID_TypeDef *pid)
{
    if (pid->Output > pid->MaxOut)
    {
        pid->Output = pid->MaxOut;
    }
    if (pid->Output < -(pid->MaxOut))
    {
        pid->Output = -(pid->MaxOut);
    }
}

static void f_Proportion_Limit(PID_TypeDef *pid)
{
    //Proportion limit is insignificant in control process
    //but it makes variable chart look better
    if (pid->Pout > pid->MaxOut)
    {
        pid->Pout = pid->MaxOut;
    }
    if (pid->Pout < -(pid->MaxOut))
    {
        pid->Pout = -(pid->MaxOut);
    }
}





/*****************PID ERRORHandle Function*********************/
static void f_PID_ErrorHandle(PID_TypeDef *pid)
{
    /*Motor Blocked Handle*/
    if (pid->Output < pid->MaxOut * 0.01f)
        return;

    if ((ABS(pid->Target - pid->Measure) / pid->Target) > 0.9f)
    {
        //Motor blocked counting
        pid->ERRORHandler.ERRORCount++;
    }
    else
    {
        pid->ERRORHandler.ERRORCount = 0;
    }

    if (pid->ERRORHandler.ERRORCount > 1000)
    {
        //Motor blocked over 1000times
        pid->ERRORHandler.ERRORType = Motor_Blocked;
    }
}
static float PID_Calculate_gimbal(PID_TypeDef *pid, float measure, float target)
{
    if (pid->Improve & ErrorHandle) //ErrorHandle
    {
        f_PID_ErrorHandle(pid); // last_xxx = xxx
        if (pid->ERRORHandler.ERRORType != PID_ERROR_NONE)
        {
            pid->Output = 0;
            return 0; //Catch ERROR
        }
    }
    pid->Last_Last_Err = pid->Last_Err;
    pid->Last_Err = pid->Err;
    pid->Measure = measure;
    pid->Target = target;
    pid->Err = pid->Target - pid->Measure;

    if (ABS(pid->Err) > pid->DeadBand)
    {
        pid->Pout = pid->Kp * (pid->Err - pid->Last_Err);
        pid->ITerm = pid->Ki * pid->Err;
        pid->Dout = pid->Kd * (pid->Err - 2.0f* pid->Last_Err + pid->Last_Last_Err);

        //Trapezoid Intergral
        if (pid->Improve & Trapezoid_Intergral)
            f_Trapezoid_Intergral(pid);
        //Changing Integral Rate
        if (pid->Improve & ChangingIntegralRate)
            f_Changing_Integral_Rate(pid);
        //Integral limit
        if (pid->Improve & Integral_Limit)
            f_Integral_Limit(pid);
        //Derivative On Measurement
        if (pid->Improve & Derivative_On_Measurement)
            f_Derivative_On_Measurement(pid);
        //Derivative filter
        if (pid->Improve & DerivativeFilter)
            f_Derivative_Filter(pid);

        pid->Iout = pid->ITerm;

        pid->Output += pid->Pout + pid->Iout + pid->Dout;

        //Output Filter
        if (pid->Improve & OutputFilter)
            f_Output_Filter(pid);

        //Output limit
        f_Output_Limit(pid);

        //Proportional limit
        f_Proportion_Limit(pid);
    }
    pid->Last_Measure = pid->Measure;
    pid->Last_Output = pid->Output;
    pid->Last_Dout = pid->Dout;
    

    return pid->Output;
}
