#ifndef H_EEPROM_H
#define H_EEPROM_H
#include "stm32f4xx_hal.h"
#include "main.h"
#include "mpu6050.h" 

#define  EEPROM_ADD		0xAF
#define  EEPROM_I2C   hi2c1



//****************  define EEPROM ADD *****************************
//                  EnD Add    0x0050

//              IMU
#define  EEPROM_GYRO_Offset_X  	   0x0000
#define  EEPROM_GYRO_Offset_Y    	 0x0002
#define  EEPROM_GYRO_Offset_Z  	   0x0004

#define  EEPROM_ACC_Offset_X       0x0006
#define  EEPROM_ACC_Offset_Y       0x0008
#define  EEPROM_ACC_Offset_Z       0x000a

#define  EEPROM_GRAVITY            0x000c

//              RC
#define  EEPROM_channel_offset_0	 0x000e
#define  EEPROM_channel_offset_1	 0x0010
#define  EEPROM_channel_offset_2	 0x0012
#define  EEPROM_channel_offset_3	 0x0014
#define  EEPROM_channel_offset_4	 0x0016
#define  EEPROM_channel_offset_5	 0x0018
#define  EEPROM_channel_offset_6	 0x001a
#define  EEPROM_channel_offset_7	 0x001c


#define  EEPROM_channel_scale_0	   0x001e
#define  EEPROM_channel_scale_1	   0x0020
#define  EEPROM_channel_scale_2	   0x0022
#define  EEPROM_channel_scale_3	   0x0024
#define  EEPROM_channel_scale_4	   0x0026
#define  EEPROM_channel_scale_5	   0x0028
#define  EEPROM_channel_scale_6	   0x002a
#define  EEPROM_channel_scale_7	   0x002c

//             control

#define  EEPROM_Yaw_Kp             0x002e
#define  EEPROM_Yaw_Ki             0x0030
#define  EEPROM_Yaw_Kd             0x0032

#define  EEPROM_Roll_Kp            0x0034
#define  EEPROM_Roll_Ki            0x0036
#define  EEPROM_Roll_Kd            0x0038

#define  EEPROM_Pitch_Kp           0x003a
#define  EEPROM_Pitch_Ki           0x003c
#define  EEPROM_Pitch_Kd           0x003e

#define  EEPROM_Altitude_Kp				 0x0040
#define  EEPROM_Altitude_Ki				 0x0042
#define  EEPROM_Altitude_Kd				 0x0044

#define  EEPROM_Altitude_Velocity_Kp				 0x0046		//**//
#define  EEPROM_Altitude_Velocity_Ki				 0x0048		//**//
#define  EEPROM_Altitude_Velocity_Kd				 0x004a		//**//

#define	 EEPROM_Position_Kp								 	 0x004c		//**//
#define	 EEPROM_Position_Ki								 	 0x004e		//**//
#define	 EEPROM_Position_Kd								 	 0x0050		//**//

#define	 EEPROM_opti_x_Kp								  	 0x0052		//**//
#define	 EEPROM_opti_x_Ki								  	 0x0054		//**//
#define	 EEPROM_opti_x_Kd								  	 0x0056   //**//

#define	 EEPROM_opti_y_Kp								  	 0x0058		//**//
#define	 EEPROM_opti_y_Ki								  	 0x006a		//**//
#define	 EEPROM_opti_y_Kd								  	 0x006c   //**//
 
#define  EEPROM_Altitude_take_off_Kp				 0x006e   //moj
#define  EEPROM_Altitude_take_off_Ki				 0x0070   //moj
#define  EEPROM_Altitude_take_off_Kd				 0x0072   //moj




void EEPROM_Write_Byte(uint16_t RegAdd,uint8_t data);
uint8_t EEPROM_Read_Byte(uint16_t RegAdd);

void EEPROM_Write_int16_t(int16_t RegAdd,int16_t data);
int16_t EEPROM_Read_int16_t(int16_t RegAdd);

//void EEPROM_Write_uint32_t(uint16_t RegAdd,uint32_t data);
//uint32_t EEPROM_Read_uint32_t(uint32_t RegAdd);


#endif 

