#include "stm32f10x.h"                  // Device header


#define SERVO_PWM_PIN GPIO_Pin_2 

void servoinit(void)
{
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
	
	GPIO_InitTypeDef GPIO_InitStruct;
    GPIO_InitStruct.GPIO_Pin = SERVO_PWM_PIN;
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF_PP;        // 复用推挽输出
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_InitStruct);
	
    TIM_OCInitTypeDef TIM_OCInitStruct;
    TIM_OCStructInit(&TIM_OCInitStruct);
    TIM_OCInitStruct.TIM_OCMode = TIM_OCMode_PWM1;
    TIM_OCInitStruct.TIM_OutputState = TIM_OutputState_Enable;
    TIM_OCInitStruct.TIM_Pulse = 1500;                  // 初始脉冲1.5ms
    TIM_OCInitStruct.TIM_OCPolarity = TIM_OCPolarity_High;
    TIM_OC3Init(TIM2, &TIM_OCInitStruct);               // 通道3初始化
	
	TIM_ITConfig(TIM2, TIM_IT_CC2, ENABLE);
    TIM_Cmd(TIM2, ENABLE);
    TIM_CtrlPWMOutputs(TIM2, ENABLE);
}

void Servo_SetAngle(float angle) {
    // SF90脉冲范围：500µs (0°) ~ 2500µs (180°)
    uint16_t pulse = 500 + (uint16_t)(angle * (2000.0f / 180.0f));
    TIM_SetCompare3(TIM2, pulse);  // 更新TIM2_CH3的占空比
}
