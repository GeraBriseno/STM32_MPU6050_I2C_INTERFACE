#include "sysClockConfig.h"
#include "TIM2setUp.h"
#include "gpioConfig.h"
#include "I2CIF.h"

static uint8_t counter = 0;

#define MPU6050_ADDRESS 0xD0

#define SMPLRT_DIV_REG 0x19
#define GYRO_CONFIG_REG 0x1B
#define ACCEL_CONFIG_REG 0x1C
#define ACCEL_XOUT_H_REG 0x3B
#define TEMP_OUT_H_REG 0x41
#define GYRO_XOUT_H_REG 0x43
#define PWR_MGMT_1_REG 0x6B
#define WHO_AM_I_REG 0x75

int16_t Accel_X_Raw = 0;
int16_t Accel_Y_Raw = 0;
int16_t Accel_Z_Raw = 0;

/*
int16_t Gyro_X_Raw = 0;
int16_t Gyro_Y_Raw = 0;
int16_t Gyro_Z_Raw = 0;
*/

float Ax, Ay, Az;
//float Gx, Gy, Gz;

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
	
	Ax = Accel_X_Raw/16384.0;
	Ay = Accel_Y_Raw/16384.0;
	Az = Accel_Z_Raw/16384.0;
	
}

/*
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
*/

void TIM2_IRQHandler(void)
{
  if(TIM2->SR & TIM_SR_UIF)   /* if UIF flag is set */
  {
    counter++;
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
