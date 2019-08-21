/*
 * timer.c
 *
 *  Created on: 14. 8. 2019
 *      Author: Petr Dvorak
 */

#include "timer.h"
#include "application.h"
#include "camera.h"

void timebase_pwm(void);
void timebase_toggle(void);

void timer_capture_init(void)
{
#ifdef TIM_CAPTURE
	TIM_ICInitTypeDef  TIM_ICInitStructure;
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	GPIO_InitTypeDef  GPIO_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;

	////////////////////////////////////////////////////////////////////////
	// Mereni kmitoctu pomoci citacu
	RCC_APB1PeriphClockCmd(TACH_0_TIMER_CLOCKS, ENABLE);
	RCC_APB2PeriphClockCmd(TACH_1_TIMER_CLOCKS, ENABLE);
	RCC_APB2PeriphClockCmd(PWM_AS_TIMER_CLOCKS, ENABLE);
	GPIO_InitStructure.GPIO_Pin   = TACH_0_PIN | TACH_1_PIN | PWM_AS_PIN;
	GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP ;
	GPIO_Init(IN_PORT_A, &GPIO_InitStructure);

	// nastaveni AF funkce
	GPIO_PinAFConfig(TACH_0_PORT, TACH_0_SOURCE, TACH_0_AF);
	GPIO_PinAFConfig(TACH_1_PORT, TACH_1_SOURCE, TACH_1_AF);
	GPIO_PinAFConfig(PWM_AS_PORT, PWM_AS_SOURCE, PWM_AS_AF);

	// povoleni preruseni pro TACH_0_TIMER
	NVIC_InitStructure.NVIC_IRQChannel = TACH_0_IRQ;
	NVIC_InitStructure.NVIC_IRQChannelPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

	// povoleni preruseni pro TACH_1_TIMER
	NVIC_InitStructure.NVIC_IRQChannel = TACH_1_IRQ;
	NVIC_InitStructure.NVIC_IRQChannelPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

	// povoleni preruseni pro PWM_AS_TIMER
	NVIC_InitStructure.NVIC_IRQChannel = PWM_AS_IRQ;
	NVIC_InitStructure.NVIC_IRQChannelPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

	/* ---------------------------------------------------------------------------
	 TIM configuration: PWM Input mode
	 The external signal is connected to x_PIN	on x_PORT
	 TIM CCR1 is used to compute the frequency value
	 TIM input clock is set to APB1 clock (PCLK1)/60000, since
	 APB1 prescaler is set to 60000, TIMCLK = PCLK1/48000 = 1000Hz

	 External Signal Frequency = TIMCLK / TIM_CCR2 in Hz.
	  --------------------------------------------------------------------------- */

	/* Time base configuration */
	TIM_TimeBaseStructure.TIM_Period = 65535;
	TIM_TimeBaseStructure.TIM_ClockDivision = 0;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseStructure.TIM_Prescaler = (uint16_t) (TACH_0_PRESCALER) - 1;
	TIM_TimeBaseInit(TACH_0_TIMER, &TIM_TimeBaseStructure);				// casova zakladna pro TACH_0

	TIM_TimeBaseStructure.TIM_Period = 65535;
	TIM_TimeBaseStructure.TIM_ClockDivision = 0;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseStructure.TIM_Prescaler = (uint16_t) (TACH_1_PRESCALER) - 1;
	TIM_TimeBaseInit(TACH_1_TIMER, &TIM_TimeBaseStructure);				// casova zakladna pro TACH_1


	TIM_TimeBaseStructure.TIM_Prescaler = (uint16_t) (PWM_AS_PRESCALER) - 1;
	TIM_TimeBaseInit(PWM_AS_TIMER, &TIM_TimeBaseStructure);				// casova zakladna pro PWM_AS

	// Input capture nastaveni TACH_0
	/* (1) Select the active input TI1 (CC1S = 01),
	       select the rising edge on CC1 (CC1P = 0, reset value)
	       and prescaler at each valid transition (IC1PS = 00, reset value) */
	/* (2) Enable capture by setting CC1E */
	/* (3) Enable interrupt on Capture/Compare */ /* (4) Enable counter */
	TACH_0_TIMER->CCMR1 |= TIM_CCMR1_CC1S_0 | TIM_CCMR1_IC1F_0 | TIM_CCMR1_IC1F_1; /* (1)*/
	TACH_0_TIMER->CCER |= TIM_CCER_CC1E; /* (2) */
	TACH_0_TIMER->DIER |= TIM_DIER_CC1IE; /* (3) */
	TACH_0_TIMER->CR1 |= TIM_CR1_CEN; /* (4) */

	// Input capture nastaveni TACH_1
	TACH_1_TIMER->CCMR1 |= TIM_CCMR1_CC1S_0 | TIM_CCMR1_IC1F_0 | TIM_CCMR1_IC1F_1; /* (1)*/
	TACH_1_TIMER->CCER |= TIM_CCER_CC1E; /* (2) */
	TACH_1_TIMER->DIER |= TIM_DIER_CC1IE; /* (3) */
	TACH_1_TIMER->CR1 |= TIM_CR1_CEN; /* (4) */

	// Input capture nastaveni PWM_AS
	PWM_AS_TIMER->CCMR1 |= TIM_CCMR1_CC1S_0 | TIM_CCMR1_IC1F_0 | TIM_CCMR1_IC1F_1; /* (1)*/
	PWM_AS_TIMER->CCER |= TIM_CCER_CC1E; /* (2) */
	PWM_AS_TIMER->DIER |= TIM_DIER_CC1IE; /* (3) */
	PWM_AS_TIMER->CR1 |= TIM_CR1_CEN; /* (4) */

	/* TIM enable counter */
	TIM_Cmd(TACH_0_TIMER, ENABLE);
	TIM_Cmd(TACH_1_TIMER, ENABLE);
	TIM_Cmd(PWM_AS_TIMER, ENABLE);

	/* Enable the CC2 Interrupt Request */
	//TIM_ITConfig(TACH_0_TIMER, TIM_IT_CC2, ENABLE);
	//TIM_ITConfig(TACH_1_TIMER, TIM_IT_CC2, ENABLE);
	//TIM_ITConfig(PWM_AS_TIMER, TIM_IT_CC2, ENABLE);
#endif
}

void timer_timebase_init(void)
{
	timebase_output_init();

	//timebase_pwm();
	timebase_toggle();
}

void timebase_pwm(void)
{
	/////////////////////////////////////
	// nastaveni hodinoveho generatoru
	/////////////////////////////////////
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
	TIM_OCInitTypeDef TIM_OCInitStructure;

	/* XCLK_TIM clock enable */
	RCC_APB2PeriphClockCmd(XCLK_TIM_CLOCKS, ENABLE);		// nastavit spravnou domenu

	/* Time Base configuration */
	uint16_t period = ((2*APB2CLK / XCLKFREQ ) - 1);
	TIM_TimeBaseStructure.TIM_Prescaler = 0;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseStructure.TIM_Period = period/2;
	TIM_TimeBaseStructure.TIM_ClockDivision = 0;
	TIM_TimeBaseStructure.TIM_RepetitionCounter = 0;
	TIM_TimeBaseInit(XCLK_TIM, (TIM_TimeBaseInitTypeDef *)&TIM_TimeBaseStructure);
	TIM_PrescalerConfig(XCLK_TIM, 0, TIM_PSCReloadMode_Immediate);

	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;		// povolit spravny vystup
	TIM_OCInitStructure.TIM_OutputNState = TIM_OutputNState_Disable;		// povolit spravny vystup
	TIM_OCInitStructure.TIM_Pulse = period/4-1;
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_Low;			// nastavit spravnou polaritu
	TIM_OCInitStructure.TIM_OCNPolarity = TIM_OCPolarity_Low;			// nastavit spravnou polaritu
	TIM_OCInitStructure.TIM_OCIdleState = TIM_OCIdleState_Set;
	TIM_OCInitStructure.TIM_OCNIdleState = TIM_OCIdleState_Set;
	TIM_OC4Init(XCLK_TIM, (TIM_OCInitTypeDef *)&TIM_OCInitStructure);	// nastavit spravny OCx vystup
	TIM_OC4PreloadConfig(XCLK_TIM, TIM_OCPreload_Enable);

	TIM_Cmd(XCLK_TIM, ENABLE);
	TIM_CtrlPWMOutputs(XCLK_TIM, ENABLE);
}

void timebase_toggle(void)
{
	/////////////////////////////////////
	// nastaveni hodinoveho generatoru
	/////////////////////////////////////
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
	TIM_OCInitTypeDef TIM_OCInitStructure;

	/* XCLK_TIM clock enable */
	RCC_APB2PeriphClockCmd(XCLK_TIM_CLOCKS, ENABLE);

	/* Time Base configuration */
	TIM_TimeBaseStructure.TIM_Prescaler = XCLK_TIM_PRESC;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseStructure.TIM_Period = ((APB2CLK / XCLKFREQ )/2 - 1);
	TIM_TimeBaseStructure.TIM_ClockDivision = 0;
	TIM_TimeBaseStructure.TIM_RepetitionCounter = 0;
	TIM_TimeBaseInit(XCLK_TIM, (TIM_TimeBaseInitTypeDef *)&TIM_TimeBaseStructure);
	TIM_PrescalerConfig(XCLK_TIM, 0, TIM_PSCReloadMode_Immediate);

	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_Toggle;
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;		// povolit spravny vystup
	TIM_OCInitStructure.TIM_OutputNState = TIM_OutputState_Disable;		// povolit spravny vystup
	TIM_OCInitStructure.TIM_Pulse = ((APB2CLK / XCLKFREQ )/2 - 1);
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_Low;			// nastavit polaritu
	TIM_OCInitStructure.TIM_OCNPolarity = TIM_OCPolarity_Low;			// nastavit polaritu
	TIM_OCInitStructure.TIM_OCIdleState = TIM_OCIdleState_Set;
	TIM_OCInitStructure.TIM_OCNIdleState = TIM_OCIdleState_Set;
	TIM_OC4Init(XCLK_TIM, (TIM_OCInitTypeDef *)&TIM_OCInitStructure);	// nastavit spravny OCx vystup
	TIM_OC4PreloadConfig(XCLK_TIM, TIM_OCPreload_Enable);

	TIM_Cmd(XCLK_TIM, ENABLE);
	TIM_CtrlPWMOutputs(XCLK_TIM, ENABLE);
}

void timebase_output_init(void)
{
	GPIO_InitTypeDef  GPIO_InitStructure;

	// nastaveni XCLK portu hodinoveho generatoru
	RCC_APB2PeriphClockCmd(XCLK_CLOCKS, ENABLE);
	GPIO_InitStructure.GPIO_Pin = XCLK_PIN;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(XCLK_PORT, &GPIO_InitStructure);
}
