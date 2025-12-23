/**
  ****************************(C) COPYRIGHT 2019 DJI****************************
  * @file       remote_control.c/h
  * @brief      遥控器处理，遥控器是通过类似SBUS的协议传输，利用DMA传输方式节约CPU
  *             资源，利用串口空闲中断来拉起处理函数，同时提供一些掉线重启DMA，串口
  *             的方式保证热插拔的稳定性。
  * @note       该任务是通过串口中断启动，不是freeRTOS任务
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Dec-26-2018     RM              1. done
  *  V1.0.0     Nov-11-2019     RM              1. support development board tpye c
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2019 DJI****************************
  */
	
#include "main.h"
#include "remote_control.h"
#include <string.h>


//遥控器出错数据上限
// upper limit of remote control error value
#define RC_CHANNAL_ERROR_VALUE 700

extern UART_HandleTypeDef huart3;
extern DMA_HandleTypeDef hdma_usart3_rx;


/**
  * @brief          get absolute value
	* @param[in]      value: original value
	* @retval         absolute value of value
  */
static int16_t RC_abs(int16_t value);
/**
  * @brief          remote control protocol resolution
  * @param[in]      sbus_buf: raw data point
  * @param[out]     rc_ctrl: remote control data struct point
  * @retval         none
  */
/**
  * @brief          遥控器协议解析
  * @param[in]      sbus_buf: 原生数据指针
  * @param[out]     rc_ctrl: 遥控器数据指
  * @retval         none
  */
static void sbus_to_rc(volatile const uint8_t *sbus_buf, RC_ctrl_t *rc_ctrl);

//remote control data 
//遥控器控制变量
RC_ctrl_t rc_ctrl;
//接收原始数据，为18个字节，给了36个字节长度，防止DMA传输越界
//buffer for raw received data. The data frame is 18 bits, but is given 36 bits to prevent DMA transmission out-of-bounds.
static uint8_t sbus_rx_buf[2][SBUS_RX_BUF_NUM];


/**
  * @brief          遥控器初始化
  * @param[in]      none
  * @retval         none
  */
/**
  * @brief          remote control init
  * @param[in]      none
  * @retval         none
  */
void remote_control_init(void)
{
    RC_Init(sbus_rx_buf[0], sbus_rx_buf[1], SBUS_RX_BUF_NUM);
}

/**
  * @brief          获取遥控器数据指针
  * @param[in]      none
  * @retval         遥控器数据指针
  */
/**
  * @brief          get remote control data point
  * @param[in]      none
  * @retval         remote control data point
  */
const RC_ctrl_t *get_remote_control_point(void)
{
    return &rc_ctrl;
}

//判断遥控器数据是否出错，
// not used
/**
  * @brief          check if rc data is wrong
  * @retval         0 if rc_ctrl values are all within the correct range, 1 is something is wrong
	*/
uint8_t RC_data_is_error(void)
{
    if (RC_abs(rc_ctrl.rc.ch[0]) > RC_CHANNAL_ERROR_VALUE || 
				RC_abs(rc_ctrl.rc.ch[1]) > RC_CHANNAL_ERROR_VALUE ||
				RC_abs(rc_ctrl.rc.ch[2]) > RC_CHANNAL_ERROR_VALUE ||
				RC_abs(rc_ctrl.rc.ch[3]) > RC_CHANNAL_ERROR_VALUE ||
				rc_ctrl.rc.s[0] == 0 ||
				rc_ctrl.rc.s[1] == 0
		)
    {
      memset(&rc_ctrl.rc.ch, 0, sizeof(rc_ctrl.rc.ch));
			memset(&rc_ctrl.rc.s, RC_SW_DOWN, sizeof(rc_ctrl.rc.s));
			memset(&rc_ctrl.mouse, 0, sizeof(rc_ctrl.mouse));
      rc_ctrl.key.v = 0;
      return 1;
    }
    return 0;
}

// not used
// typo here, should be solve... not slove...
void slove_RC_lost(void)
{
    RC_restart(SBUS_RX_BUF_NUM);
}

// not used
void slove_data_error(void)
{
    RC_restart(SBUS_RX_BUF_NUM);
}

//串口中断
void USART3_IRQHandler(void)
{
    if(huart3.Instance->SR & UART_FLAG_RXNE)//接收到数据 receives data
    {
        __HAL_UART_CLEAR_PEFLAG(&huart3);
    }
    else if(USART3->SR & UART_FLAG_IDLE)
    {
        static uint16_t this_time_rx_len = 0;
				uint16_t mem_buf_index = (hdma_usart3_rx.Instance->CR & DMA_SxCR_CT) == RESET ? 0 : 1;

        __HAL_UART_CLEAR_PEFLAG(&huart3);

        //disable DMA
        //失效DMA
        __HAL_DMA_DISABLE(&hdma_usart3_rx);

        //get receive data length, length = set_data_length - remain_length
        //获取接收数据长度,长度 = 设定长度 - 剩余长度
        this_time_rx_len = SBUS_RX_BUF_NUM - hdma_usart3_rx.Instance->NDTR;

        //reset set_data_lenght
        //重新设定数据长度
        hdma_usart3_rx.Instance->NDTR = SBUS_RX_BUF_NUM;

				// set memory buffer 1 if Memory 0 is used; set memory buffer 0 if Mmeory 1 is used
        if (mem_buf_index == 0) {
					//set memory buffer 1
          //设定缓冲区1
					hdma_usart3_rx.Instance->CR |= DMA_SxCR_CT;
				} else {
					//set memory buffer 0
          //设定缓冲区0
					DMA1_Stream1->CR &= ~(DMA_SxCR_CT);
				}
            
        //enable DMA
        //使能DMA
        __HAL_DMA_ENABLE(&hdma_usart3_rx);

        if(this_time_rx_len == RC_FRAME_LENGTH)
        {
            sbus_to_rc(sbus_rx_buf[mem_buf_index], &rc_ctrl);
            //记录数据接收时间
						// record time that data is received [not implemented]

        }
    }
}

/**
  * @brief          get absolute value
	* @param[in]      value: original value
	* @retval         absolute value of value
  */
static int16_t RC_abs(int16_t value)
{
    if (value > 0)
    {
        return value;
    }
    else
    {
        return -value;
    }
}

/**
  * @brief          遥控器协议解析
  * @param[in]      sbus_buf: 原生数据指针
  * @param[out]     rc_ctrl: 遥控器数据指
  * @retval         none
  */
/**
  * @brief          remote control protocol resolution
  * @param[in]      sbus_buf: raw data point
  * @param[out]     rc_ctrl: remote control data struct point
  * @retval         none
  */
static void sbus_to_rc(volatile const uint8_t *sbus_buf, RC_ctrl_t *rc_ctrl)
{
    if (sbus_buf == NULL || rc_ctrl == NULL)
    {
        return;
    }

    rc_ctrl->rc.ch[0] = (sbus_buf[0] | (sbus_buf[1] << 8)) & 0x07ff;        //!< Channel 0
    rc_ctrl->rc.ch[1] = ((sbus_buf[1] >> 3) | (sbus_buf[2] << 5)) & 0x07ff; //!< Channel 1
    rc_ctrl->rc.ch[2] = ((sbus_buf[2] >> 6) | (sbus_buf[3] << 2) |          //!< Channel 2
                         (sbus_buf[4] << 10)) &0x07ff;
    rc_ctrl->rc.ch[3] = ((sbus_buf[4] >> 1) | (sbus_buf[5] << 7)) & 0x07ff; //!< Channel 3
    rc_ctrl->rc.s[0] = ((sbus_buf[5] >> 4) & 0x0003);                  //!< Switch left
    rc_ctrl->rc.s[1] = ((sbus_buf[5] >> 4) & 0x000C) >> 2;                       //!< Switch right
    rc_ctrl->mouse.x = sbus_buf[6] | (sbus_buf[7] << 8);                    //!< Mouse X axis
    rc_ctrl->mouse.y = sbus_buf[8] | (sbus_buf[9] << 8);                    //!< Mouse Y axis
    rc_ctrl->mouse.z = sbus_buf[10] | (sbus_buf[11] << 8);                  //!< Mouse Z axis
    rc_ctrl->mouse.press_l = sbus_buf[12];                                  //!< Mouse Left Is Press ?
    rc_ctrl->mouse.press_r = sbus_buf[13];                                  //!< Mouse Right Is Press ?
    // 假设 rc_ctrl 已经通过 sbus_to_rc 解析完毕
uint16_t key_value = sbus_buf[14] | (sbus_buf[15] << 8);

// 直接按位判断每个键的状态
rc_ctrl->key.w    = (key_value & KEY_PRESSED_OFFSET_W)     ? 1 : 0;
rc_ctrl->key.s     = (key_value & KEY_PRESSED_OFFSET_S)     ? 1 : 0;
rc_ctrl->key.a     = (key_value & KEY_PRESSED_OFFSET_A)     ? 1 : 0;
rc_ctrl->key.d     = (key_value & KEY_PRESSED_OFFSET_D)     ? 1 : 0;
rc_ctrl->key.shift = (key_value & KEY_PRESSED_OFFSET_SHIFT) ? 1 : 0;
rc_ctrl->key.ctrl   = (key_value & KEY_PRESSED_OFFSET_CTRL)  ? 1 : 0;
rc_ctrl->key.q     = (key_value & KEY_PRESSED_OFFSET_Q)     ? 1 : 0;
rc_ctrl->key.e     = (key_value & KEY_PRESSED_OFFSET_E)     ? 1 : 0;
rc_ctrl->key.r     = (key_value & KEY_PRESSED_OFFSET_R)     ? 1 : 0;
rc_ctrl->key.f     = (key_value & KEY_PRESSED_OFFSET_F)     ? 1 : 0;
rc_ctrl->key.g     = (key_value & KEY_PRESSED_OFFSET_G)     ? 1 : 0;
rc_ctrl->key.z     = (key_value & KEY_PRESSED_OFFSET_Z)     ? 1 : 0;
rc_ctrl->key.x     = (key_value & KEY_PRESSED_OFFSET_X)     ? 1 : 0;
rc_ctrl->key.c     = (key_value & KEY_PRESSED_OFFSET_C)     ? 1 : 0;
rc_ctrl->key.v     = (key_value & KEY_PRESSED_OFFSET_V)     ? 1 : 0;
rc_ctrl->key.b     = (key_value & KEY_PRESSED_OFFSET_B)     ? 1 : 0;

    
    rc_ctrl->rc.ch[4] = sbus_buf[16] | (sbus_buf[17] << 8);                 //NULL

    rc_ctrl->rc.ch[0] -= RC_CH_VALUE_OFFSET;
    rc_ctrl->rc.ch[1] -= RC_CH_VALUE_OFFSET -6;
    rc_ctrl->rc.ch[2] -= RC_CH_VALUE_OFFSET;
    rc_ctrl->rc.ch[3] -= RC_CH_VALUE_OFFSET;
    rc_ctrl->rc.ch[4] -= RC_CH_VALUE_OFFSET;
}

