#ifndef H_NRF_H
#define H_NRF_H

#include "stm32f4xx_hal.h"
#include "MS5611.h"
#include "stdarg.h"
#include "mpu6050.h"
#include "IMU.h"
#include "RC.h"
#include "Altitude.h"
#include "Control.h"
#include "Odometery.h"
#include "Optical_Flow.h"


#define		ACCEL							1
#define		GYRO							2
#define   GRAVITY         	3
#define   Euler_angles      4

#define		RC_ATITUDE				5
#define		RC_CHANNEL				6

#define   ms_coefficient    7
#define   ms_detail			    8

#define   Altitude_Detail   9

#define   Vel_Z    			    10

#define   Roll_Gain         			11
#define   Pitch_Gain        			12
#define   Yaw_Gain          			13
#define   Altitude_Gain     			14
#define   Altitude_Velocity_Gain  15
#define		Position_Gain						16


#define   Roll_Gain_set     					17
#define   Pitch_Gain_set    					18
#define   Yaw_Gain_set      					19
#define   Altitude_Gain_set 					20
#define   Altitude_Velocity_Gain_set 	21
#define		Position_Gain_set						22


#define   Roll_Control_signal	      					23
#define   Pitch_Control_signal	     					24
#define   Yaw_Control_signal	       					25
#define   Altitude_Control_signal  						26
#define   Altitude_Velocity_Control_signal  	27
#define	Position_Control_signal								28


#define   BAT               									29
#define   Time                        				30
#define   PTAM_DATA														31
#define   Optical_Flow												32
#define   opti_x_set													33
#define   opti_x_read													34
#define   opti_y_set													35
#define   opti_y_read													36
#define   opti_x_signal												37
#define   opti_y_signal												38
#define   Altitude_take_off_Gain              39
#define   Altitude_take_off_Gain_set          40


 typedef struct {
	 
	I2C_HandleTypeDef I2C;
	uint8_t Add;	 
	uint8_t data[60];// age bekhad beshkane do bodish kon :p
	uint8_t State;
	uint8_t Num;
	uint8_t Check_Sum;
	uint8_t Len;
	uint8_t g_data;
	uint8_t fail;
	
	 
}NRF_BOARD;
 
extern NRF_BOARD NRF;

void Nrf_Empty_Data(NRF_BOARD* Nrf);
void NrF_Fill_Data(NRF_BOARD* Nrf , uint8_t num , ... );
void Nrf_Send_Data(NRF_BOARD* Nrf);
void Nrf_Init(NRF_BOARD* Nrf,I2C_HandleTypeDef hi2c,uint8_t Add);
void Nrf_(NRF_BOARD* Nrf,int State);
void Nrf_Tele(NRF_BOARD* Nrf);

#endif 

