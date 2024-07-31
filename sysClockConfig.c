#include "sysClockConfig.h"

/* Function used to configure System Clock, in this case we're using the external crystal oscillator in
 ST-Link debugger connected to Nucleo Board STM32F401RE using Phase Locked Loop (PLL) */
void SysClockConfig (void){
	
	/* Frequency of crystal oscillator is 8MHz, to calculate frequency obtained with PLL, we use the formula:
	 ((inputfreq/PLL_M)*PLL_N)/PLL_P  in this case ((80/4)*80)/2 */
	
	/* Values to write to reset and clock control pll configuration register (RCC->PLLCFGR) to get desired clock values
	 (these values have been obtained using STM32CubeIDE) */
	#define PLL_M 	4
	#define PLL_N 	80
	#define PLL_P 	0  // PLLP = 2

	// 1. Enable HSE (high speed external clock source) and wait for the HSE to become ready
	RCC->CR |= RCC_CR_HSEON;  // RCC->CR |= 1<<16;
	while (!(RCC->CR & RCC_CR_HSERDY));  // while (!(RCC->CR & (1<<17)));
	
	// 2. Enable power interface clock
	RCC->APB1ENR |= RCC_APB1ENR_PWREN;  // RCC->APB1ENR |= 1<<28;
	// Set VOS (voltage output scaling) in PWR register as reserved (scale 2 mode selected) to allow a maximum freq of 84 MHz
	PWR->CR |= PWR_CR_VOS;  // PWR->CR |= 3<<14; 
	
	/* 3. Configure the flash prefetch and the latency related settings
	 (these values have been obtained using STM32CubeIDE) */
	FLASH->ACR = FLASH_ACR_ICEN | FLASH_ACR_DCEN | FLASH_ACR_PRFTEN | FLASH_ACR_LATENCY_2WS;  // FLASH->ACR = (1<<8) | (1<<9)| (1<<10)| (5<<0);
	
	/* 4. Configure the prescalers to get final values for HCLK, PCLK1, PCLK2
	 (these values have been obtained using STM32CubeIDE) */
	
	// AHB prescaler = 1
	RCC->CFGR |= RCC_CFGR_HPRE_DIV1;  // RCC->CFGR |= (0<<4);
	
	// APB1 prescaler = 2
	RCC->CFGR |= RCC_CFGR_PPRE1_DIV2;  // RCC->CFGR |= (4<<10);
	
	// APB2 prescaler = 1
	RCC->CFGR |= RCC_CFGR_PPRE2_DIV1;  // RCC->CFGR |= (0<<13);
	
	// 5. Configure the main PLL
	RCC->PLLCFGR = (PLL_M << 0) | (PLL_N << 6) | (PLL_P <<16) | (RCC_PLLCFGR_PLLSRC_HSE);  // (1<<22);

	// 6. Enable the PLL and wait for it to become ready
	RCC->CR |= RCC_CR_PLLON;  // RCC->CR |= (1<<24);
	while (!(RCC->CR & RCC_CR_PLLRDY));  // while (!(RCC->CR & (1<<25)));
	
	// 7. Select the Clock Source as PLL and wait for it to be set
	RCC->CFGR |= RCC_CFGR_SW_PLL;  // RCC->CFGR |= (2<<0);
	while ((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_PLL);  // while (!(RCC->CFGR & (2<<2)));
	
}