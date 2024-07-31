#include "gpioConfig.h"
#include "sysClockConfig.h"

//Function to configure GPIO port and pins used, in this case GPIO port A
void GPIO_Config(void){
	
	// 1. Enable the GPIOB CLOCK
	RCC->AHB1ENR |= (1<<1);
	
	// 2. Set the respective GPIOB pins as Alternate Mode
	GPIOB->MODER |= (2<<16);  // alternate mode for PB 8
	GPIOB->MODER |= (2<<18);  // alternate mode for PB 9
	
	// 3. Set the respective GPIOB pins' Alternate Function
	GPIOB->AFR[1] |= (4<<0);  //set alternate function of GPIOB 8 as af4 (I2C1 SCL)
	GPIOB->AFR[1] |= (4<<4);   //set alternate function of GPIOB 9 as af4 (I2C1 SDA)
	
	// 4. Set the respective GPIOB pins' mode as open drain output
	GPIOB->OTYPER |= (1<<8);
	GPIOB->OTYPER |= (1<<9);
	
	// 5. Set the respective GPIOB pins' speed as very high
	GPIOB->OSPEEDR |= (3<<16);
	GPIOB->OSPEEDR |= (3<<18);
	
	// 6.- Set the respective GPIOB pins' speed as pull up
	GPIOB->PUPDR |= (1<<16);
	GPIOB->PUPDR |= (1<<18);
	
}