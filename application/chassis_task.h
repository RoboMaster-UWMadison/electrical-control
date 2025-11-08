#ifndef CHASSIS_TASK_H
#define CHASSIS_TASK_H
#include "struct_typedef.h"
#include "motor_CAN.h"
#include "pid.h"
#include "pid_dji.h"
#include "remote_control.h"
#include "user_lib.h"
//任务开始空闲一段时间
#define CHASSIS_TASK_INIT_TIME 357
//前后的遥控器通道号码
#define CHASSIS_X_CHANNEL 3
//左右的遥控器通道号码
#define CHASSIS_Y_CHANNEL 2
//选择底盘状态 开关通道号
#define CHASSIS_MODE_CHANNEL  1
//遥控器前进摇杆（max 660）转化成车体前进速度（m/s）的比例
#define CHASSIS_VX_RC_SEN 0.0025f
//遥控器左右摇杆（max 660）转化成车体左右速度（m/s）的比例
#define CHASSIS_VY_RC_SEN 0.0025f
//跟随底盘yaw模式下，遥控器的yaw遥杆（max 660）增加到车体角度的比例
#define CHASSIS_ANGLE_Z_RC_SEN 0.000002f
//不跟随云台的时候 遥控器的yaw遥杆（max 660）转化成车体旋转速度的比例
#define CHASSIS_WZ_RC_SEN 0.01f
#define CHASSIS_ACCEL_X_NUM 0.1666666667f
#define CHASSIS_ACCEL_Y_NUM 0.3333333333f
//摇杆死区
#define CHASSIS_RC_DEADLINE 10
#define MOTOR_SPEED_TO_CHASSIS_SPEED_VX 0.25f
#define MOTOR_SPEED_TO_CHASSIS_SPEED_VY 0.25f
#define MOTOR_SPEED_TO_CHASSIS_SPEED_WZ 0.25f
#define MOTOR_DISTANCE_TO_CENTER 0.2f
//底盘任务控制间隔 2ms
#define CHASSIS_CONTROL_TIME_MS 2
//底盘任务控制间隔 0.002s
#define CHASSIS_CONTROL_TIME 0.002f
//底盘任务控制频率，尚未使用这个宏
#define CHASSIS_CONTROL_FREQUENCE 500.0f
//底盘3508最大can发送电流值
#define MAX_MOTOR_CAN_CURRENT 30000.0f
//底盘摇摆按键
#define SWING_KEY KEY_PRESSED_OFFSET_CTRL
//底盘前后左右控制按键
#define CHASSIS_FRONT_KEY KEY_PRESSED_OFFSET_W
#define CHASSIS_BACK_KEY KEY_PRESSED_OFFSET_S
#define CHASSIS_LEFT_KEY KEY_PRESSED_OFFSET_A
#define CHASSIS_RIGHT_KEY KEY_PRESSED_OFFSET_D
//m3508转化成底盘速度(m/s)的比例，
#define M3508_MOTOR_RPM_TO_VECTOR 0.000415809748903494517209f
#define CHASSIS_MOTOR_RPM_TO_VECTOR_SEN M3508_MOTOR_RPM_TO_VECTOR
//单个底盘电机最大速度
#define MAX_WHEEL_SPEED 6.0f
//底盘运动过程最大前进速度
#define NORMAL_MAX_CHASSIS_SPEED_X 2.0f
//底盘运动过程最大平移速度
#define NORMAL_MAX_CHASSIS_SPEED_Y 2.0f
//底盘电机速度环PID
#define M3505_MOTOR_SPEED_PID_KP 5000.0f
#define M3505_MOTOR_SPEED_PID_KI 20.0f
#define M3505_MOTOR_SPEED_PID_KD 0.0f
#define M3505_MOTOR_SPEED_PID_MAX_OUT 20000.0f
#define M3505_MOTOR_SPEED_PID_MAX_IOUT 50.0f
//底盘电机电流环PID
#define M3505_MOTOR_CURRENT_PID_KP 0.5f
#define M3505_MOTOR_CURRENT_PID_KI 0.2f
#define M3505_MOTOR_CURRENT_PID_KD 0.0f


//电机码盘值最大以及中值
#define HALF_ECD_RANGE  4096
#define ECD_RANGE       8191
typedef enum
{
  CHASSIS_FOLLOW_GIMBAL,     //底盘跟随云台
  GIMBAL_FOLLOW_CHASSIS,  //云台跟随底盘
  CHASSIS_ZERO_FORCE,            //control-current will be sent to CAN bus derectly.
} chassis_mode_e;

typedef struct
{
  const motor_measure_t *chassis_motor_measure;
  fp32 accel;
  fp32 speed;
  fp32 speed_set;
  fp32 current_set;
  int16_t give_current;
} chassis_motor_t;

typedef struct
{
  const RC_ctrl_t *chassis_RC;               //底盘使用的遥控器指针
  const motor_measure_t *chassis_INS_angle;             //获取陀螺仪解算出的欧拉角指针
  chassis_mode_e chassis_mode;               // 底盘控制状态机
  chassis_mode_e last_chassis_mode;          //底盘上次控制状态机
  chassis_motor_t motor_chassis[4];          //底盘电机数据
  
  PID_TypeDef motor_speed_pid[4];             //底盘电机速度pid
  pid_type_def motor_current_pid[4];             //底盘电机速度pid

  first_order_filter_type_t chassis_cmd_slow_set_vx;  //use first order filter to slow set-point.使用一阶低通滤波减缓设定值
  first_order_filter_type_t chassis_cmd_slow_set_vy;  //use first order filter to slow set-point.使用一阶低通滤波减缓设定值

  fp32 vx;                          //底盘速度 前进方向 前为正，单位 m/s
  fp32 vy;                          //底盘速度 左右方向 左为正  单位 m/s
  fp32 wz;                          //底盘旋转角速度，逆时针为正 单位 rad/s
  fp32 vx_set;                      //底盘设定速度 前进方向 前为正，单位 m/s
  fp32 vy_set;                      //底盘设定速度 左右方向 左为正，单位 m/s
  fp32 wz_set;                      //底盘设定旋转角速度，逆时针为正 单位 rad/s
  fp32 chassis_absolute_angle;      //底盘绝对角度，单位 rad

  fp32 vx_max_speed;  //前进方向最大速度 单位m/s
  fp32 vx_min_speed;  //后退方向最大速度 单位m/s
  fp32 vy_max_speed;  //左方向最大速度 单位m/s
  fp32 vy_min_speed;  //右方向最大速度 单位m/s

} chassis_move_t;
/**
  * @brief          底盘任务，间隔 CHASSIS_CONTROL_TIME_MS 2ms
  * @param[in]      pvParameters: 空
  * @retval         none
  */
extern void chassis_task(void const *pvParameters);
/**
  * @brief          根据遥控器通道值，计算纵向和横移速度
  *                 
  * @param[out]     vx_set: 纵向速度指针
  * @param[out]     vy_set: 横向速度指针
  * @param[out]     chassis_move_rc_to_vector: "chassis_move" 变量指针
  * @retval         none
  */
extern void chassis_rc_to_control_vector(fp32 *vx_set, fp32 *vy_set, chassis_move_t *chassis_move_rc_to_vector);

#endif
