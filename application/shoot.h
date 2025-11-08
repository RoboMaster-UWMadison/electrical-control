#ifndef SHOOT_H
#define SHOOT_H
#include "struct_typedef.h"

#include "motor_CAN.h"
#include "remote_control.h"
#include "user_lib.h"
#include "pid.h"
#include "volt.h"
//电机rmp 变化成 旋转速度的比例
#define MOTOR_RPM_TO_SPEED          0.00290888208665721596153948461415f
#define MOTOR_ECD_TO_ANGLE          0.000021305288720633905968306772076277f
//拨弹速度
#define TRIGGER_SPEED_slow               -9.0f
#define TRIGGER_SPEED_fast               -15.0f
#define FRIC_SPEED                  17.8f

#define BLOCK_TRIGGER_SPEED         2.0f
#define BLOCK_TIME                  200
#define REVERSE_TIME                400

#define PI_FOUR                     0.78539816339744830961566084581988f
#define PI_TEN                      0.314f

#define TRIGGER_RPM_KP        400.0f
#define TRIGGER_RPM_KI        0.1f
#define TRIGGER_RPM_KD        0.0f

#define FRIC_RPM_KP        600.0f
#define FRIC_RPM_KI        0.1f
#define FRIC_RPM_KD        0.0f
typedef enum
{
  SHOOT_STOP = 0,
  SHOOT_READY,
  SHOOT_LAUNCH,
} shoot_state_e;

typedef struct
{
    shoot_state_e shoot_state;
    const RC_ctrl_t *shoot_rc;
    const motor_measure_t *turret_motor_measure[3];
    PID_TypeDef trigger_motor_pid;
    PID_TypeDef fric_motor_pid_r;
    PID_TypeDef fric_motor_pid_l;
    fp32 speed[3];
    fp32 speed_set[3];
    int16_t given_current[3];

    uint16_t block_time;
    uint16_t reverse_time;

} shoot_control_t;

extern void shoot_task(void const *pvParameters);

extern void shoot_init(void);

extern int16_t* shoot_control_loop(void);
#endif
