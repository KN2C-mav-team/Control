#ifndef H_Main_H
#define H_Main_H

#include "stm32f4xx_hal.h"
#include "Define.h"
#include "stdio.h"
#include "mpu6050.h"
//#include <mavlink/common/mavlink.h>
#include "MPC.h"
#include "Altitude.h"


#define BATT_Coef   350.0f 

extern uint8_t	 SBUS_Packet_Data[25]; 
extern unsigned int SBUS_Channel_Data[18];
extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart2;
extern UART_HandleTypeDef huart3;
extern I2C_HandleTypeDef hi2c1;
extern I2C_HandleTypeDef hi2c3;
extern TIM_HandleTypeDef htim1;
extern uint32_t Bat_;
extern UART_HandleTypeDef huart4;
extern int time;
void Check_battery(int System_counter,int System_state);



void decode_mavlink_packet(uint8_t c,_MPC* Mpc);
void decode_mavlink_packet_2(uint8_t c,_MPC* Mpc);
void encode_mavlink_packet(void);



#endif 
