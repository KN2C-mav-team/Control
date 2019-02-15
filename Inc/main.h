#ifndef H_Main_H
#define H_Main_H

#include "stm32f4xx_hal.h"
#include "Define.h"
#include "stdio.h"
#include "mpu6050.h"
#include "Telemetri.h"
#include <mavlink/common/mavlink.h>
#include "MPC.h"


#define BATT_Coef   350.0f 
#define ADDRESS (0xE0) 
#define REGISTER_ADDRESS (0x00) 
#define H_BYTE (0x02) 
#define L_BYTE (0x03) 

#define SET_CM (0x51) //cm

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
void Led_Beat(char state,int count);
void Check_battery(int System_counter,int System_state);
void SBUS_Packet_ground_mode() ; 
void SBUS_Packet_fly_mode() ;
void 	read_srf02();
void decode_mavlink_packet(uint8_t c,_MPC* Mpc);
void decode_mavlink_packet_2(uint8_t c,_MPC* Mpc);
void encode_mavlink_packet(void);



#endif 
