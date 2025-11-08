#include "motor_CAN.h"
#include "main.h"
#include "detect_task.h"

extern CAN_HandleTypeDef hcan1;
extern CAN_HandleTypeDef hcan2;

#define get_motor_measure(ptr, data)                                    \
    {                                                                   \
        (ptr)->last_ecd = (ptr)->ecd;                                   \
        (ptr)->ecd = (uint16_t)((data)[0] << 8 | (data)[1]);            \
        (ptr)->speed_rpm = (uint16_t)((data)[2] << 8 | (data)[3]);      \
        (ptr)->given_current = (uint16_t)((data)[4] << 8 | (data)[5]);  \
        (ptr)->temperate = (data)[6];                                   \
    }
    
    

static CAN_TxHeaderTypeDef turret_tx_messege;
static CAN_TxHeaderTypeDef gimbal_tx_messege;
static CAN_TxHeaderTypeDef  chassis_tx_message;
    
static uint8_t turret_can_send_data[8];
static uint8_t gimbal_can_send_data[8];
static uint8_t chassis_can_send_data[8];
    
motor_measure_t motor_chassis[5];
motor_measure_t motor_gimbal[5];
/**
  * @brief          发送电机控制电流(0x205,0x206)
  * @param[in]      motor1: (0x201) 2006电机控制电流, 范围 [-10000,10000]
  * @param[in]      motor2: (0x202) 3508电机控制电流, 范围 [-16384,16384]
  * @param[in]      motor3: (0x203) 3508电机控制电流, 范围 [-16384,16384]
  * @retval         none
  */
void CAN_cmd_TURRET(int16_t shoot1,int16_t shoot2,int16_t trigger)
{
  turret_tx_messege.StdId = CAN_TURRET_ID;
  turret_tx_messege.RTR   = CAN_RTR_DATA;
  turret_tx_messege.DLC   = 0x08;
  turret_tx_messege.IDE   = CAN_ID_STD;
  
  turret_can_send_data[0] = shoot1 >> 8;
  turret_can_send_data[1] = shoot1;
  turret_can_send_data[2] = shoot2 >> 8;
  turret_can_send_data[3] = shoot2;
  turret_can_send_data[4] = trigger>> 8;
  turret_can_send_data[5] = trigger;
  turret_can_send_data[6] = 0;
  turret_can_send_data[7] = 0;
  
  HAL_CAN_AddTxMessage(&hcan1,&turret_tx_messege,turret_can_send_data,0);
  
  
}
/**
  * @brief          发送电机控制电流(0x205,0x206)
  * @param[in]      motor1: (0x205) 6020电机控制电压, 范围 [-30000,30000]
  * @param[in]      motor2: (0x206) 6020电机控制电压, 范围 [-30000,30000]
  * @retval         none
  */
void CAN_cmd_GIMBAL(int16_t yaw,int16_t pitch)
{
  gimbal_tx_messege.StdId = CAN_GIMBAL_ID;
  gimbal_tx_messege.RTR   = CAN_RTR_DATA;
  gimbal_tx_messege.DLC   = 0x08;
  gimbal_tx_messege.IDE   = CAN_ID_STD;
  
  gimbal_can_send_data[0] = yaw >> 8;
  gimbal_can_send_data[1] = yaw;
  gimbal_can_send_data[2] = pitch >> 8;
  gimbal_can_send_data[3] = pitch;
  gimbal_can_send_data[4] = 0;
  gimbal_can_send_data[5] = 0;
  gimbal_can_send_data[6] = 0;
  gimbal_can_send_data[7] = 0;
  
  HAL_CAN_AddTxMessage(&hcan1,&gimbal_tx_messege,gimbal_can_send_data,0);
}
/**
  * @brief          发送电机控制电流(0x201,0x202,0x203,0x204)
  * @param[in]      motor1: (0x201) 3508电机控制电流, 范围 [-16384,16384]
  * @param[in]      motor2: (0x202) 3508电机控制电流, 范围 [-16384,16384]
  * @param[in]      motor3: (0x203) 3508电机控制电流, 范围 [-16384,16384]
  * @param[in]      motor4: (0x204) 3508电机控制电流, 范围 [-16384,16384]
  * @retval         none
  */
void CAN_cmd_chassis(int16_t motor1, int16_t motor2, int16_t motor3, int16_t motor4)
{
    uint32_t send_mail_box;
    chassis_tx_message.StdId = CAN_CHASSIS_ALL_ID;
    chassis_tx_message.IDE = CAN_ID_STD;
    chassis_tx_message.RTR = CAN_RTR_DATA;
    chassis_tx_message.DLC = 0x08;
    chassis_can_send_data[0] = motor1 >> 8;
    chassis_can_send_data[1] = motor1;
    chassis_can_send_data[2] = motor2 >> 8;
    chassis_can_send_data[3] = motor2;
    chassis_can_send_data[4] = motor3 >> 8;
    chassis_can_send_data[5] = motor3;
    chassis_can_send_data[6] = motor4 >> 8;
    chassis_can_send_data[7] = motor4;

    HAL_CAN_AddTxMessage(&hcan2, &chassis_tx_message, chassis_can_send_data, &send_mail_box);
}
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
  CAN_RxHeaderTypeDef rx_header;
  uint8_t rx_data[8];
  HAL_CAN_GetRxMessage(hcan,0,&rx_header,rx_data);
  
  if(hcan == &hcan1)
  {
    switch(rx_header.StdId)
   {
    case CAN_SHOOT_M1:
    case CAN_SHOOT_M2:
    case CAN_TRIGGER:
    {
      static uint8_t i = 0;
            //get motor id
      i = rx_header.StdId - CAN_SHOOT_M1;
      get_motor_measure(&motor_gimbal[i], rx_data);
      break;
    }
    case CAN_YAW_MOTOR_ID:
    case CAN_PIT_MOTOR_ID:      
    {
      static uint8_t i = 0;
            //get motor id
      i = rx_header.StdId - 0x202;
      get_motor_measure(&motor_gimbal[i], rx_data);        
      break;
    }
     default:
    {
      break;
    }
   }   
  }
  else if(hcan == &hcan2)
  {
    switch (rx_header.StdId)
    {
        case CAN_3508_M1_ID:
        case CAN_3508_M2_ID:
        case CAN_3508_M3_ID:
        case CAN_3508_M4_ID:
        case CAN_IMU_ID:
        {
            static uint8_t i = 0;
            //get motor id
            i = rx_header.StdId - CAN_3508_M1_ID;
            get_motor_measure(&motor_chassis[i], rx_data);
            detect_hook(CHASSIS_MOTOR1_TOE + i);
            break;
        }
        
        default:
        {
            break;
        }
    }
  }
   
}

/**
  * @brief          返回6020电机数据指针
  * @param[in]      none
  * @retval         电机数据指针
  */
const motor_measure_t *get_pitch_gimbal_motor_measure_point(void)
{
    return &motor_gimbal[4];
}
const motor_measure_t *get_yaw_gimbal_motor_measure_point(void)
{
    return &motor_gimbal[3];
}

/**
  * @brief          返回拨弹电机 2006电机数据指针
  * @param[in]      none
  * @retval         电机数据指针
  */
const motor_measure_t *get_trigger_motor_measure_point(void)
{
    return &motor_gimbal[2];
}
/**
  * @brief          返回右摩擦轮电机 3508电机数据指针
  * @param[in]      none
  * @retval         电机数据指针
  */
const motor_measure_t *get_fric_motor_r_measure_point(void)
{
    return &motor_gimbal[1];
}
/**
  * @brief          返回左摩擦轮电机 3508电机数据指针
  * @param[in]      none
  * @retval         电机数据指针
  */
const motor_measure_t *get_fric_motor_l_measure_point(void)
{
    return &motor_gimbal[0];
}
/**
  * @brief          返回底盘电机 3508电机数据指针
  * @param[in]      i: 电机编号,范围[0,3]
  * @retval         电机数据指针
  */
const motor_measure_t *get_chassis_motor_measure_point(uint8_t i)
{
    return &motor_chassis[(i & 0x03)];
}

const motor_measure_t *get_chassis_imu_point(void)
{
    return &motor_chassis[4];
}



















