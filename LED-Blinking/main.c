#include "stm32f407xx.h"

void SysClockConfig (void);
void GPIO_config (void);
void delay(uint32_t time);

void SysClockConfig (void)
{
    #define PLL_M 4
    #define PLL_N 168
    #define PLL_P 0

    // 1. ENABLE HSE and wait for the HSE to become Ready
    RCC->CR |= RCC_CR_HSEON; // HSEON  (1<<16)
    while (!(RCC->CR & RCC_CR_HSERDY)); // HSERDY (1<<17)

    // 2. Set the POWER ENABLE CLOCK and VOLTAGE REGULATOR
    RCC->APB1ENR |= RCC_APB1ENR_PWREN; // PWREN (1<<28)
    PWR->CR |=  PWR_CR_VOS; // VOS (1<<14)

    // 3. Configure the FLASH PREFETCH and the LATENCY Related Settings
    FLASH->ACR |= FLASH_ACR_PRFTEN; // PRFTEN (1<<8)
    FLASH->ACR |= FLASH_ACR_ICEN; // ICEN (1<<10)
    FLASH->ACR |= FLASH_ACR_DCEN;// DCEN (1<<9) 
    FLASH->ACR |= FLASH_ACR_LATENCY_5WS; // LATENCY (5 << 0)

    // 4. Configure the PRESCALARS HCLK, PCLK1, PCLK2
    RCC->CFGR &= RCC_CFGR_HPRE_DIV1; // HPRE AHB Prescaler  i.e 1
    RCC->CFGR |= RCC_CFGR_PPRE1_DIV4; // PPRE1 APB1 Prescaler i.e 4
    RCC->CFGR |= RCC_CFGR_PPRE2_DIV2; // PPRE2 APB2 Prescaler i.e 2

    // 5. Configure the MAIN PLL
    RCC->PLLCFGR = (PLL_M <<0) | (PLL_N << 6) | (PLL_P <<16) | (1<<22);

    // 6. Enable the PLL and wait for it to become ready
    RCC->CR |= RCC_CR_PLLON; // PLLON (1<<24)
    while (!(RCC->CR & RCC_CR_PLLRDY)); // PLLRDY (1<<25)

    // 7. Select the Clock Source and wait for it to be set
    RCC->CFGR |= RCC_CFGR_SW_PLL; // SW (2<<0)
//    while (!(RCC->CFGR & (2<<2))); // SWS
		while ((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_PLL); // SWS
    
}

void GPIO_config (void) 
{
	/*
	1. Enable the GPIO clock 
	2. Setup the Pin as output
	3. Configure the output mode
	
	*/
	RCC->AHB1ENR |= (1<<3);
	
	GPIOD->MODER |= (1<< 24); //PD12
	GPIOD->OTYPER = 0;
	GPIOD->OSPEEDR = 0;
}

void delay(uint32_t time)
{
	while (time--);
}

int main(void)
{
	SysClockConfig();
	GPIO_config();
	while(1)
	{
		GPIOD->BSRR |= (1<<12);
		delay (30000000);
		GPIOD->BSRR |= (1<<28);
		delay (10000000);
	}
}
