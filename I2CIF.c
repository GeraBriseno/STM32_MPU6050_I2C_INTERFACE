#include "I2CIF.h"
#include "sysClockConfig.h"

void I2C_Config(void){

	// 1. Enable the I2C 1 clock
	RCC->AHB2ENR |= (1<<21);
	
	// 2. Put I2C in reset mode
	I2C1->CR1 |= (1<<15);
	
	// 3.- Take I2C out of reset mode
	I2C1->CR1 &= (0<<15);
	
	// 4.- Enable I2C1 peripheral clock
	RCC->APB1ENR |= (1 << 21);
	
	// 4.- Set frequency I2C1 peripheral clock to 40 MHz
	I2C1->CR2 |= (40<<0);
	
	// 5.- Set I2C1 master mode as standard mode
	I2C1->CCR |= (0<<15);
	
	// 6.- Set clock control register value using formula: Th = (tr + tw) / pclk 
	// (4000+1000)/(1/40)
	I2C1->CCR = 200<<0;
	
	// 7.- Set value of rise time using formula: TRISE = (1000/pclk) + 1
	// (1000/(1/40)) + 1
	I2C1->TRISE = 41<<0;
	
	// 8.- Enable I2C1
	I2C1->CR1 |= (1<<0);
	
}

void I2C_Start(void){
	
	// 1.- Enable Acknowledge
	I2C1->CR1 |= (1<<10);
	
	// 2.- Start I2C1 
	I2C1->CR1 |= (1<<8);
	
	// 3.- Wait for status bit to set
	while (!(I2C1->SR1 & (1<<0)));
	
}

void I2C_Address(uint8_t Address){

	// 1.- Send the slave address by loading it into data register
	I2C1->DR = Address;
	
	// 1.- Wait for ADDR bit to set, indicating end of address transmission
	while (!(I2C1->SR1 & (1<<1)));
	
	// 3.- Read status bits for status registers 1 and 2 to clear ADDR bit
	uint8_t temp = I2C1->SR1 | I2C1->SR2;
	
}

void I2C_Write(uint8_t data){
	
	// 1.- Wait for TxE bit to set, indicating DR is empty
	while (!(I2C1->SR1 & (1<<7)));
	
	// 2.- Send data by loading it into data register
	I2C1->DR = data;
	
	// 3.- Wait for BTF bit to set, indicating end of last data transmission
	while (!(I2C1->SR1 & (1<<2)));
	
}

void I2C_Write_Multi(uint8_t *data, uint8_t size){
	
	// 1.- Wait for TxE bit to set, indicating DR is empty
	while (!(I2C1->SR1 & (1<<7)));
	
	// 2.- Send data by loading it into data register
	while(size)
	{	
		//Send data
		I2C1->DR = (volatile uint32_t)*data++;
		
		//Wait for TxE bit to set
		while (!(I2C1->SR1 & (1<<7)));
		
		size--;
	}
	// 3.- Wait for BTF bit to set, indicating end of last data transmission
	while (!(I2C1->SR1 & (1<<2)));
	
}

void I2C_Stop(void){
	
	// 1.- Stop I2C1 
	I2C1->CR1 |= (1<<9);
	
}

void I2C_Read(uint8_t Address, uint8_t *buffer, uint8_t size){
	
	int remaining = size;
	
	if(size ==1){
		//Send the slave address
		I2C1->DR = Address;
		
		//Wait for ADDR bit to set, indicating end of address transmission
		while (!(I2C1->SR1 & (1<<1)));
		
		//Clear acknowledge bit
		I2C1->CR1 &= ~(1<<10);
		
		//Read status bits for status registers 1 and 2 to clear ADDR bit
		uint8_t temp = I2C1->SR1 | I2C1->SR2;
		
		//Stop I2C1 
		I2C1->CR1 |= (1<<9);
	
		//Wait for RxNE bit to set
		while (!(I2C1->SR1 & (1<<6)));
		
		//Copy received data to buffer
		buffer[size-remaining] = I2C1->DR;
	}
	
	else{
		//Send the slave address
		I2C1->DR = Address;
		
		//Wait for ADDR bit to set, indicating end of address transmission
		while (!(I2C1->SR1 & (1<<1)));
		
		//Read status bits for status registers 1 and 2 to clear ADDR bit
		uint8_t temp = I2C1->SR1 | I2C1->SR2;
		
		while(remaining > 2){
			//Wait for RxNE bit to set
			while (!(I2C1->SR1 & (1<<6)));
			
			//Copy received data to buffer
			buffer[size-remaining] = I2C1->DR;
			
			//Set acknowledge bit
			I2C1->CR1 |= (1<<10);
			
			remaining--;
		}
		
		//READ SECOND LAST BYTE
		
		//Wait for RxNE bit to set
		while (!(I2C1->SR1 & (1<<6)));
		
		//Copy received data to buffer
		buffer[size-remaining] = I2C1->DR;
		
		//Clear acknowledge bit
		I2C1->CR1 &= ~(1<<10);
		
		//Stop I2C1 
		I2C1->CR1 |= (1<<9);
		
		remaining--;

		//READ THE LAST BYTE
		
		//Wait for RxNE bit to set
		while (!(I2C1->SR1 & (1<<6)));
		
		//Copy received data to buffer
		buffer[size-remaining] = I2C1->DR;
	}
	
}