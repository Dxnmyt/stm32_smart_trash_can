#include "stm32f10x.h"                  // Device header
#include "Delay.h"


// 引脚定义
#define TRIG_PIN     GPIO_Pin_0     // PA0
#define ECHO_PIN     GPIO_Pin_1     // PA1


uint32_t startTime = 0;
uint32_t endTime = 0;
uint8_t captureState = 0;
float distance_cm = 0;
uint8_t distance_updated = 0;
const uint16_t DISTANCE_THRESHOLD_CM = 30;


void Trig_Pulse(void) {
    GPIO_ResetBits(GPIOA, TRIG_PIN);
    Delay_us(20);                   // 20µs触发脉冲
    GPIO_SetBits(GPIOA, TRIG_PIN);
}

void choashengbo(void)
{
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);

    // 配置Trig引脚（PA0）为推挽输出
    GPIO_InitTypeDef GPIO_InitStruct;
    GPIO_InitStruct.GPIO_Pin = TRIG_PIN;
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_InitStruct);

    // 配置Echo引脚（PA1）为上拉输入
    GPIO_InitStruct.GPIO_Pin = ECHO_PIN;
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_IPU;
    GPIO_Init(GPIOA, &GPIO_InitStruct);

    // 初始化TIM2时基（同时用于输入捕获和PWM）
    TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStruct;
    TIM_TimeBaseInitStruct.TIM_Prescaler = 72 - 1;      // 1MHz计数频率
    TIM_TimeBaseInitStruct.TIM_Period = 20000 - 1;      // 20ms周期 (50Hz)
    TIM_TimeBaseInitStruct.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInitStruct.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseInit(TIM2, &TIM_TimeBaseInitStruct);

    // 输入捕获通道配置（PA1对应TIM2_CH2）
    TIM_ICInitTypeDef TIM_ICInitStruct;
    TIM_ICInitStruct.TIM_Channel = TIM_Channel_2;
    TIM_ICInitStruct.TIM_ICPolarity = TIM_ICPolarity_Rising;
    TIM_ICInitStruct.TIM_ICSelection = TIM_ICSelection_DirectTI;
    TIM_ICInitStruct.TIM_ICPrescaler = TIM_ICPSC_DIV1;
    TIM_ICInitStruct.TIM_ICFilter = 0x0;
    TIM_ICInit(TIM2, &TIM_ICInitStruct);
	
    TIM_ITConfig(TIM2, TIM_IT_CC2, ENABLE);
    NVIC_InitTypeDef NVIC_InitStruct;
    NVIC_InitStruct.NVIC_IRQChannel = TIM2_IRQn;
    NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = 0;
    NVIC_InitStruct.NVIC_IRQChannelSubPriority = 1;
    NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStruct);

    TIM_Cmd(TIM2, ENABLE);
}

void TIM2_IRQHandler(void) {
    if (TIM_GetITStatus(TIM2, TIM_IT_CC2)) {
        if (captureState == 0) {  // 上升沿捕获
            startTime = TIM_GetCapture2(TIM2);
            // 切换为下降沿捕获
            TIM_OC2PolarityConfig(TIM2, TIM_ICPolarity_Falling);
            captureState = 1;
        } else {  // 下降沿捕获
            endTime = TIM_GetCapture2(TIM2);
            // 计算时间差（处理计数器溢出）
            uint32_t duration = (endTime >= startTime) ? 
                (endTime - startTime) : 
                (TIM2->ARR - startTime + endTime);
            distance_cm = (duration * 0.034f) / 2.0f;  // 单位：cm
            distance_updated = 1;
            // 恢复为上升沿捕获
            TIM_OC2PolarityConfig(TIM2, TIM_ICPolarity_Rising);
            captureState = 0;
        }
        TIM_ClearITPendingBit(TIM2, TIM_IT_CC2);
    }
}
