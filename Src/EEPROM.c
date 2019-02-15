#include "EEPROM.h"
#include "main.h"
#include "mpu6050.h" 


ch2int con_;

void EEPROM_Write_Byte(uint16_t RegAdd,uint8_t data)
{
	HAL_I2C_Mem_Write(&EEPROM_I2C,EEPROM_ADD,RegAdd,2,(uint8_t *)&data,3,1);
	HAL_Delay(10);
}

uint8_t EEPROM_Read_Byte(uint16_t RegAdd)
{
	char data;
	HAL_I2C_Mem_Read(&EEPROM_I2C,EEPROM_ADD,RegAdd,2,(uint8_t *)&data,1,1);
	return data;
}

void EEPROM_Write_int16_t(int16_t RegAdd,int16_t data)
{
	con_.real=data;
	HAL_I2C_Mem_Write(&EEPROM_I2C,EEPROM_ADD,RegAdd,2,(uint8_t *)con_.byte,2,1);
	HAL_Delay(10);
	
}

int16_t EEPROM_Read_int16_t(int16_t RegAdd)
{
	HAL_I2C_Mem_Read(&EEPROM_I2C,EEPROM_ADD,RegAdd,2,(uint8_t *)con_.byte,2,1);
	return con_.real;
}



