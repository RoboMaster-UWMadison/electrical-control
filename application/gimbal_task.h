#ifndef GIMBAL_TASK_H
#define GIMBAL_TASK_H
#include "struct_typedef.h"
#include "motor_CAN.h"
#include "pid.h"
#include "pid_dji.h"
#include "remote_control.h"


//yaw speed close-loop PID params, max out and max iout
//yaw 速度环 PID参数以及 PID最大输出，积分输出
#define YAW_SPEED_PID_KP        3000.0f //3000f
#define YAW_SPEED_PID_KI        0.05f //0.003f
#define YAW_SPEED_PID_KD        0.0f // 0.2f
#define YAW_SPEED_PID_MAX_OUT   30000.0f
#define YAW_SPEED_PID_MAX_IOUT  5000.0f
//yaw gyro angle close-loop PID params, max out and max iout
//yaw 角度环 角度由陀螺仪解算 PID参数以及 PID最大输出，积分输出
#define YAW_GYRO_ABSOLUTE_PID_KP        25.0f //25.0f
#define YAW_GYRO_ABSOLUTE_PID_KI        0.0f //0.0f
#define YAW_GYRO_ABSOLUTE_PID_KD        1000.0f //800.0f
#define YAW_GYRO_ABSOLUTE_PID_MAX_OUT   15.0f
#define YAW_GYRO_ABSOLUTE_PID_MAX_IOUT  1.0f
//yaw 角度环 角度由miniPC解算 PID参数以及 PID最大输出，积分输出
#define YAW_CV_KP        30.0f //25.0f
#define YAW_CV_KI        0.001f //0.0f
#define YAW_CV_KD        1200.0f //800.0f
#define YAW_CV_MAX_OUT   15.0f
#define YAW_CV_IOUT  1.0f
//yaw encode angle close-loop PID params, max out and max iout
//yaw 角度环 角度由编码器 PID参数以及 PID最大输出，积分输出
#define YAW_ENCODE_RELATIVE_PID_KP        3.0f //8.0f
#define YAW_ENCODE_RELATIVE_PID_KI        0.0f
#define YAW_ENCODE_RELATIVE_PID_KD        0.0f
#define YAW_ENCODE_RELATIVE_PID_MAX_OUT   10.0f
#define YAW_ENCODE_RELATIVE_PID_MAX_IOUT  0.0f

//pitch speed close-loop PID params, max out and max iout
//pitch 速度环 PID参数以及 PID最大输出，积分输出
#define PITCH_SPEED_PID_KP        5000.0f //10000f
#define PITCH_SPEED_PID_KI        0.0f //2.0f
#define PITCH_SPEED_PID_KD        0.1f //0.1f
#define PITCH_SPEED_PID_MAX_OUT   10000.0f
#define PITCH_SPEED_PID_MAX_IOUT  10000.0f
//pitch gyro angle close-loop PID params, max out and max iout
//pitch 角度环 角度由陀螺仪解算 PID参数以及 PID最大输出，积分输出
#define PITCH_GYRO_ABSOLUTE_PID_KP 0.0f //7.0f
#define PITCH_GYRO_ABSOLUTE_PID_KI 0.0f //0.0f
#define PITCH_GYRO_ABSOLUTE_PID_KD 0.0f //2.0f
#define PITCH_GYRO_ABSOLUTE_PID_MAX_OUT 80.0f
#define PITCH_GYRO_ABSOLUTE_PID_MAX_IOUT 1.0f
//pitch encode angle close-loop PID params, max out and max iout
//pitch 角度环 角度由编码器 PID参数以及 PID最大输出，积分输出
#define PITCH_ENCODE_RELATIVE_PID_KP 40000.0f //20.0f
#define PITCH_ENCODE_RELATIVE_PID_KI 30.0f
#define PITCH_ENCODE_RELATIVE_PID_KD 450000.0f //0.02f
#define PITCH_ENCODE_RELATIVE_PID_MAX_OUT 10000.0f
#define PITCH_ENCODE_RELATIVE_PID_MAX_IOUT 2000.0f



//任务初始化 空闲一段时间
#define GIMBAL_TASK_INIT_TIME 201
//yaw,pitch控制通道以及状态开关通道
#define YAW_CHANNEL   0
#define PITCH_CHANNEL 1
#define GIMBAL_MODE_CHANNEL 0

//turn 180°
//掉头180 按键
#define TURN_KEYBOARD KEY_PRESSED_OFFSET_F
//turn speed
//掉头云台速度
#define TURN_SPEED    0.04f
//测试按键尚未使用
#define TEST_KEYBOARD KEY_PRESSED_OFFSET_R
//rocker value deadband
//遥控器输入死区，因为遥控器存在差异，摇杆在中间，其值不一定为零
#define RC_DEADBAND   10


#define YAW_RC_SEN    -0.00002f
#define PITCH_RC_SEN  -0.00001f //0.005

#define YAW_MOUSE_SEN   0.00002f
#define PITCH_MOUSE_SEN 0.000004f

#define YAW_ENCODE_SEN    0.01f
#define PITCH_ENCODE_SEN  0.01f

#define GIMBAL_CONTROL_TIME 1

//test mode, 0 close, 1 open
//云台测试模式 宏定义 0 为不使用测试模式
#define GIMBAL_TEST_MODE 0

#define PITCH_TURN  1
#define YAW_TURN    0

//电机码盘值最大以及中值
#define HALF_ECD_RANGE  4096
#define ECD_RANGE       8191
//云台初始化回中值，允许的误差,并且在误差范围内停止一段时间以及最大时间6s后解除初始化状态，
#define GIMBAL_INIT_ANGLE_ERROR     0.1f
#define GIMBAL_INIT_STOP_TIME       100
#define GIMBAL_INIT_TIME            6000
#define GIMBAL_CALI_REDUNDANT_ANGLE 0.1f
//云台初始化回中值的速度以及控制到的角度
#define GIMBAL_INIT_PITCH_SPEED     0.004f
#define GIMBAL_INIT_YAW_SPEED       0.005f

#define INIT_YAW_SET    0.0f
#define INIT_PITCH_SET  0.0f

//云台校准中值的时候，发送原始电流值，以及堵转时间，通过陀螺仪判断堵转
#define GIMBAL_CALI_MOTOR_SET   8000
#define GIMBAL_CALI_STEP_TIME   2000
#define GIMBAL_CALI_GYRO_LIMIT  0.1f

#define GIMBAL_CALI_PITCH_MAX_STEP  1
#define GIMBAL_CALI_PITCH_MIN_STEP  2
#define GIMBAL_CALI_YAW_MAX_STEP    3
#define GIMBAL_CALI_YAW_MIN_STEP    4

#define GIMBAL_CALI_START_STEP  GIMBAL_CALI_PITCH_MAX_STEP
#define GIMBAL_CALI_END_STEP    5

//判断遥控器无输入的时间以及遥控器无输入判断，设置云台yaw回中值以防陀螺仪漂移
#define GIMBAL_MOTIONLESS_RC_DEADLINE 10
#define GIMBAL_MOTIONLESS_TIME_MAX    3000

#define PITCH_G_OFFSET -5400.0f
typedef enum
{
    GIMBAL_MOTOR_RAW = 0, //电机原始值控制
    GIMBAL_MOTOR_GYRO,    //电机陀螺仪角度控制
    GIMBAL_MOTOR_ENCONDE, //电机编码值角度控制
} gimbal_motor_mode_e;
typedef struct
{
    const motor_measure_t *gimbal_motor_measure;
    PID_TypeDef gimbal_motor_absolute_angle_pid;
    PID_TypeDef gimbal_motor_CV_angle_pid;
    PID_TypeDef gimbal_motor_relative_angle_pid;
    pid_type_def gimbal_motor_gyro_pid;
    gimbal_motor_mode_e gimbal_motor_mode;
    gimbal_motor_mode_e last_gimbal_motor_mode;
    uint16_t offset_ecd;
    fp32 max_relative_angle; //rad
    fp32 min_relative_angle; //rad

    fp32 relative_angle;     //rad
    fp32 relative_angle_set; //rad
    fp32 absolute_angle;     //rad
    fp32 absolute_angle_set; //rad
    fp32 motor_gyro;         //rad/s
    fp32 motor_gyro_set;
    fp32 motor_speed;
    fp32 raw_cmd_current;
    fp32 current_set;
    int16_t given_current;

} gimbal_motor_t;
typedef struct
{
    fp32 max_yaw;
    fp32 min_yaw;
    fp32 max_pitch;
    fp32 min_pitch;
    uint16_t max_yaw_ecd;
    uint16_t min_yaw_ecd;
    uint16_t max_pitch_ecd;
    uint16_t min_pitch_ecd;
    uint8_t step;
} gimbal_step_cali_t;
typedef struct
{
    const RC_ctrl_t *gimbal_rc_ctrl;
    const fp32 *gimbal_INT_angle_point;
    const fp32 *gimbal_INT_gyro_point;
    gimbal_motor_t gimbal_yaw_motor;
    gimbal_motor_t gimbal_pitch_motor;
    gimbal_step_cali_t gimbal_cali;
    fp32 pitch_g_offset;
} gimbal_control_t;


extern void gimbal_task(void const *pvParameters);

#endif


















