#include "stm32f407xx.h"
#include "RCCconfig.h"

void TIM2_CH4_Init(void)
{
    // Enable TIM2 and GPIOA clock
    RCC->APB1ENR |= (1<<0); // Enable TIM2 Clock
    RCC->AHB1ENR |= (1<<0); // Enable GPIOA clock

    // Set PA3 as alternate function
    GPIOA->MODER |= (0x2 << (2*3));

    // SET PA3 as AF1
    GPIOA->AFR[0] |= (1 << (4*3));

    TIM2->PSC = (84-1);
    TIM2->ARR = (100-1);

    // Set Channel 4 as PWM mode 1
    TIM2->CCMR2 |= (0x6 << 12);

    // Enable the output for Channel 4
    TIM2->CCER |= (1 << 12);

    TIM2->CR1 = 1;
}

void TIM2_CH4_PWM_Duty_Cycle(int dutyCycle)
{
	TIM2->CCR4 = (uint32_t)dutyCycle;
}

int main (void) 
{
	SysClockConfig();
	
	TIM2_CH4_Init();
	
	while(1)
	{
		for(int i=0; i<100; i++){
					TIM2_CH4_PWM_Duty_Cycle(i);
					for(int j=0; j<100000; j++);
		 }
	}
	
}

