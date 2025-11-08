#ifndef GIMBAL_DEMO_H
#define GIMBAL_DEMO_H

#include "struct_typedef.h"

#define MOTOR_RPM_TO_SPEED          0.00290888208665721596153948461415f
#define MOTOR_ECD_TO_ANGLE          0.000021305288720633905968306772076277f

typedef enum
{
    CAN_TURRET_ID = 0x200,
    CAN_SHOOT_M1 = 0x201,
    CAN_SHOOT_M2 = 0x202,
    CAN_TRIGGER = 0x203,
    
    CAN_GIMBAL_ID = 0x1FF,
    CAN_YAW_MOTOR_ID = 0x205,
    CAN_PIT_MOTOR_ID = 0x206,

} can1_msg_id_e;

typedef enum
{
    CAN_CHASSIS_ALL_ID = 0x200,
    CAN_3508_M1_ID = 0x201,
    CAN_3508_M2_ID = 0x202,
    CAN_3508_M3_ID = 0x203,
    CAN_3508_M4_ID = 0x204,
    
    CAN_SENSOR_ID = 0x1FF,
    CAN_IMU_ID = 0x205,

} can2_msg_id_e;
//rm motor data
typedef struct
{
    uint16_t ecd;
    int16_t speed_rpm;
    int16_t given_current;
    uint8_t temperate;
    int16_t last_ecd;
} motor_measure_t;

extern void CAN_cmd_TURRET(int16_t shoot1,int16_t shoot2,int16_t trigger);

extern void CAN_cmd_GIMBAL(int16_t yaw,int16_t pitch);

extern void CAN_cmd_chassis(int16_t motor1, int16_t motor2, int16_t motor3, int16_t motor4);

extern const motor_measure_t *get_yaw_gimbal_motor_measure_point(void);

extern const motor_measure_t *get_pitch_gimbal_motor_measure_point(void);

extern const motor_measure_t *get_trigger_motor_measure_point(void);

extern const motor_measure_t *get_fric_motor_r_measure_point(void);

extern const motor_measure_t *get_fric_motor_l_measure_point(void);

extern const motor_measure_t *get_chassis_motor_measure_point(uint8_t i);

extern const motor_measure_t *get_chassis_imu_point(void);
#endif






