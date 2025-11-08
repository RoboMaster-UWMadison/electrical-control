#include "main.h"
#include "miniPC_control.h"

extern UART_HandleTypeDef huart6;
extern DMA_HandleTypeDef hdma_usart6_rx;

// DMA 双缓冲区
uint8_t usart6_rx_buf[2][24]; 

float miniPC_yaw = 0, miniPC_pitch = 0,fire = 1;
static float yaw = 0, pitch = 0;

void miniPC_control_init(uint8_t *rx1_buf, uint8_t *rx2_buf, uint16_t dma_buf_num)
{
    //使能DMA串口接收
    SET_BIT(huart6.Instance->CR3, USART_CR3_DMAR);
    //使能空闲中断
    __HAL_UART_ENABLE_IT(&huart6, UART_IT_IDLE);
    //失效DMA
    __HAL_DMA_DISABLE(&hdma_usart6_rx);
    while(hdma_usart6_rx.Instance->CR & DMA_SxCR_EN)
    {
        __HAL_DMA_DISABLE(&hdma_usart6_rx);
    }
    hdma_usart6_rx.Instance->PAR = (uint32_t) & (USART6->DR);
    //内存缓冲区1
    hdma_usart6_rx.Instance->M0AR = (uint32_t)(rx1_buf);
    //内存缓冲区2
    hdma_usart6_rx.Instance->M1AR = (uint32_t)(rx2_buf);
    //数据长度
    hdma_usart6_rx.Instance->NDTR = dma_buf_num;
    //使能双缓冲区
    SET_BIT(hdma_usart6_rx.Instance->CR, DMA_SxCR_DBM);
    //使能DMA
    __HAL_DMA_ENABLE(&hdma_usart6_rx);
    HAL_GPIO_WritePin(GPIOH, GPIO_PIN_12, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOH, GPIO_PIN_11, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOH, GPIO_PIN_10, GPIO_PIN_RESET);

}

void USART6_IRQHandler(void)  
{
    volatile uint8_t receive;

    // 接收中断
    if (__HAL_UART_GET_FLAG(&huart6, UART_FLAG_RXNE))
    {
        receive = huart6.Instance->DR;
        // 如果需要，可以在此处处理字节级接收数据
    }

    // 空闲中断
    else if (__HAL_UART_GET_FLAG(&huart6, UART_FLAG_IDLE))
    {
        // 关键：清除 IDLE 中断标志位（原来错误地使用了 __HAL_UART_CLEAR_PEFLAG）
        __HAL_UART_CLEAR_IDLEFLAG(&huart6);

        static uint16_t this_time_rx_len = 0;

        if ((hdma_usart6_rx.Instance->CR & DMA_SxCR_CT) == RESET)
        {
            // 当前使用的是 Memory 0
            __HAL_DMA_DISABLE(&hdma_usart6_rx);
            this_time_rx_len = 24u - hdma_usart6_rx.Instance->NDTR;
            hdma_usart6_rx.Instance->NDTR = 24u;
            hdma_usart6_rx.Instance->CR |= DMA_SxCR_CT;  // 切换到 Memory 1
            __HAL_DMA_ENABLE(&hdma_usart6_rx);

            if (this_time_rx_len == 12u)
            {
                memcpy(&yaw,   &usart6_rx_buf[0][8], 4);
                memcpy(&pitch, &usart6_rx_buf[0][4], 4);
                memcpy(&fire,  &usart6_rx_buf[0][0], 4);
                HAL_GPIO_TogglePin(GPIOH, GPIO_PIN_10);

                if (yaw <= 3.14f)   miniPC_yaw   += yaw;
                if (pitch <= 3.14f) miniPC_pitch += pitch;
            }
        }
        else
        {
            // 当前使用的是 Memory 1
            __HAL_DMA_DISABLE(&hdma_usart6_rx);
            this_time_rx_len = 24u - hdma_usart6_rx.Instance->NDTR;
            hdma_usart6_rx.Instance->NDTR = 24u;
            DMA1_Stream1->CR &= ~(DMA_SxCR_CT);  // 切换回 Memory 0
            __HAL_DMA_ENABLE(&hdma_usart6_rx);

            if (this_time_rx_len == 12u)
            {
                memcpy(&yaw,   &usart6_rx_buf[1][0], 4);
                memcpy(&pitch, &usart6_rx_buf[1][4], 4);
                memcpy(&fire,  &usart6_rx_buf[1][8], 4);
                HAL_GPIO_TogglePin(GPIOH, GPIO_PIN_10);

                if (yaw <= 3.14f)   miniPC_yaw   += yaw;
                if (pitch <= 3.14f) miniPC_pitch += pitch;
            }
        }
    }
}
