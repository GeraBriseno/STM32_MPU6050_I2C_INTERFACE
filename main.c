#include "stm32f401xe.h"
#include <math.h>

uint8_t delay_flag = 0;

/* Function used to configure System Clock, in this case we're using the external crystal oscillator in
 ST-Link debugger connected to Nucleo Board STM32F401RE using Phase Locked Loop (PLL) */
static void SysClockConfig (void){
	
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

static void TIM2_config(void){
	
	RCC->APB1ENR |= (1 << 0);   //ENABLE TIM2 PERIPHERAL CLOCK
	
	TIM2->PSC = 79;     //SET TIM2 FREQUENCY TO 1 MHZ (80000000/(79+1)) USING PRESCALER
	
	TIM2->ARR = (uint32_t)500000;   //SET COUNTER RESET TIME
	
	//TIM2->CNT = 0;    //SET TIM2 COUNTER TO 0
	
	TIM2->DIER |= TIM_DIER_UIE;   //(1 << 0) ENABLE TIM2 INTERRUPT
	
	TIM2->SR &= ~TIM_SR_UIF;  //CLEAR TIM2 INTERRUPT STATUS
	
	NVIC_EnableIRQ(TIM2_IRQn);  //ENABLE GLOBAL NVIC INTERRUPTS FOR TIM2
	
	TIM2->CR1 = TIM_CR1_CEN;  // |= (1 << 0);  //ENABLE COUNTER FOR TIM2
	}


static void I2C_Config(void){

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

static void I2C_Start(void){
	
	// 1.- Enable Acknowledge
	I2C1->CR1 |= (1<<10);
	
	// 2.- Start I2C1 
	I2C1->CR1 |= (1<<8);
	
	// 3.- Wait for status bit to set
	while (!(I2C1->SR1 & (1<<0)));
	
}

static void I2C_Address(uint8_t Address){

	// 1.- Send the slave address by loading it into data register
	I2C1->DR = Address;
	
	// 1.- Wait for ADDR bit to set, indicating end of address transmission
	while (!(I2C1->SR1 & (1<<1)));
	
	// 3.- Read status bits for status registers 1 and 2 to clear ADDR bit
	uint8_t temp = I2C1->SR1 | I2C1->SR2;
	
}

static void I2C_Write(uint8_t data){
	
	// 1.- Wait for TxE bit to set, indicating DR is empty
	while (!(I2C1->SR1 & (1<<7)));
	
	// 2.- Send data by loading it into data register
	I2C1->DR = data;
	
	// 3.- Wait for BTF bit to set, indicating end of last data transmission
	while (!(I2C1->SR1 & (1<<2)));
	
}

static void I2C_Write_Multi(uint8_t *data, uint8_t size){
	
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

static void I2C_Stop(void){
	
	// 1.- Stop I2C1 
	I2C1->CR1 |= (1<<9);
	
}

static void I2C_Read(uint8_t Address, uint8_t *buffer, uint8_t size){
	
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

//Function to configure GPIO port and pins used, in this case GPIO port A
static void GPIO_Config(void){
	
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

static void MPU_Write(uint8_t Address, uint8_t Register, uint8_t Data){
	I2C_Start();
	I2C_Address(Address);
	I2C_Write(Register);
	I2C_Write(Data);
	I2C_Stop();
}

static void MPU_Read(uint8_t Address, uint8_t Register, uint8_t *buffer, uint8_t size){
	I2C_Start();
	I2C_Address(Address);
	I2C_Write(Register);
	I2C_Start();
	I2C_Read(Address+0x01, buffer, size);
	I2C_Stop();
}

#define MPU6050_ADDRESS 0xD0

#define SMPLRT_DIV_REG 0x19
#define GYRO_CONFIG_REG 0x1B
#define ACCEL_CONFIG_REG 0x1C
#define ACCEL_XOUT_H_REG 0x3B
#define TEMP_OUT_H_REG 0x41
#define GYRO_XOUT_H_REG 0x43
#define PWR_MGMT_1_REG 0x6B
#define WHO_AM_I_REG 0x75

uint16_t Accel_X_Raw = 0;
uint16_t Accel_Y_Raw = 0;
uint16_t Accel_Z_Raw = 0;

uint16_t Gyro_X_Raw = 0;
uint16_t Gyro_Y_Raw = 0;
uint16_t Gyro_Z_Raw = 0;

float Ax, Ay, Az, Gx, Gy, Gz;

uint8_t check;

static void MPU6050_Init(){
	
	uint8_t check;
	uint8_t data;
	
	//Get device ID
	MPU_Read(MPU6050_ADDRESS, WHO_AM_I_REG, &check, 1);
	
	//if we get ID 104 (expected)
	if(check == 104){
		
		//Write 0 to power management register to wake sensor up
		data = 0;
		MPU_Write(MPU6050_ADDRESS, PWR_MGMT_1_REG, data);
		
		//Set data rate to 1KHz
		data = 0x07;
		MPU_Write(MPU6050_ADDRESS, SMPLRT_DIV_REG, data);
		
		//Set accelerometer config
		//XA_ST=0, YA_ST=0, ZA_ST=0, AFS_SEL=0 +-2g
		data = 0x00;
		MPU_Write(MPU6050_ADDRESS, ACCEL_CONFIG_REG, data);
		
		//Set gyroscope config
		//XG_ST=0, YG_ST=0, ZG_ST=0, FS_SEL=0
		data = 0x00;
		MPU_Write(MPU6050_ADDRESS, GYRO_CONFIG_REG, data);
		
	}
	
}

static void MPU6050_Read_Accel(void){
	
	uint8_t Rx_Data[6];
	
	MPU_Read(MPU6050_ADDRESS, ACCEL_XOUT_H_REG, Rx_Data, 6);
	
	Accel_X_Raw = (int16_t)(Rx_Data[0] << 8 | Rx_Data[1]);
	Accel_Y_Raw = (int16_t)(Rx_Data[2] << 8 | Rx_Data[3]);
	Accel_Z_Raw = (int16_t)(Rx_Data[4] << 8 | Rx_Data[5]);
	
	MPU_Read(MPU6050_ADDRESS, ACCEL_YOUT_H_REG, Rx_Data, 6);
	
	MPU_Read(MPU6050_ADDRESS, ACCEL_YOUT_H_REG, Rx_Data, 6);
	
	Ax = Accel_X_Raw/16384.0;
	Ay = Accel_Y_Raw/16384.0;
	Az = Accel_Z_Raw/16384.0;
	
}

static void MPU6050_Read_Gyro(void){
	
	uint8_t Rx_Data[6];
	
	MPU_Read(MPU6050_ADDRESS, GYRO_XOUT_H_REG, Rx_Data, 6);
	
	Gyro_X_Raw = (int16_t)(Rx_Data[0] << 8 | Rx_Data[1]);
	Gyro_Y_Raw = (int16_t)(Rx_Data[2] << 8 | Rx_Data[3]);
	Gyro_Z_Raw = (int16_t)(Rx_Data[4] << 8 | Rx_Data[5]);
	
	Gx = Gyro_X_Raw/16384.0;
	Gy = Gyro_Y_Raw/16384.0;
	Gz = Gyro_Z_Raw/16384.0;
	
}

void TIM2_IRQHandler(void)
{
  if(TIM2->SR & TIM_SR_UIF)   /* if UIF flag is set */
  {
    delay_flag++;
		MPU6050_Read_Accel();
    TIM2->SR &= ~TIM_SR_UIF;  /* Clear the Interrupt Status */
  }
}

// Our main function
int main (void){
	
	// Run our configuration functions
	SysClockConfig ();
	TIM2_config();
	GPIO_Config();
	I2C_Config();
	
	MPU6050_Init();
	
	// Our while loop
	while (1)
	{		
		
	}
		
}
