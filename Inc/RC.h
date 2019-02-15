#ifndef H_RC_H
#define H_RC_H
#include "main.h"
#include "stm32f4xx_hal.h"
#include "mpu6050.h"
#include "EEPROM.h"


#define sbus_packet_size   25

#define RC_I2C_Err         1
#define RC_Invalid_Data    2

#define Throttle_range     400 //400
#define angle_range        (20.0f)


#define Roll_offset     0//5//fajina   2     old robot -11
#define Pitch_offset    0//-3 //fajina  -1                -5

#define YawThreshold       2
#define RC_noise           2000  //**//

#define	F_CUT_RC			      15//25
#define FILTER_RC           1/(2*PI*F_CUT_RC)


//channel define
#define Throttle_channel  6         //0      6
#define Roll_channel      5         //2        5
#define Pitch_channel     7         //1        7
#define Yaw_channel       4         //3        4
#define RC_SW_channel     2          //4       2 
#define HOV_THR_channel   0          //5        1
#define RC_TRIM_channel   1           //7      0
#define HOV_PIT_channel   3          //6       3

typedef struct 
{
 uint16_t channel[9];
 uint16_t	data[22]  ;
 int     	state;
 int      m;
 int      flag;
 int      ready_data;
 uint8_t  buffer[sbus_packet_size];
}_SBus;
extern _SBus SBUS;
typedef struct 
{
	uint16_t RC_channel[8];
  uint16_t last_RC_channel[8];
	uint16_t channel_offset[8];
	uint16_t channel_scale[8];
	float Throttle,last_Throttle,Throttle_save;
	float Roll,last_Roll;
	float Pitch,last_Pitch;
	float Yaw,last_Yaw;
	float RC_SW;
	float THR_CUT;
	float HOV_THR;
	float RC_TRIM;
	float HOV_PIT;
	uint16_t State;
	uint8_t  init;
	uint8_t  fail;
	uint8_t  Invalid_Data_counter;
	I2C_HandleTypeDef I2C;
	
	
}_RC;


extern _RC RC;

void RC_Read(_RC* Rc,int State);
void RC_Init(_RC* Rc ,char calib);
void RC_Calib(_RC* Rc,char calib);
void RC_2_SetPoint(_RC* Rc);
void RC_Read_EEPROM(_RC* Rc);
void recieve_sbus_radio(_SBus* sbus , _RC* Rc);
void SBUS_Packet_ground_mode() ; 
void SBUS_Packet_fly_mode() ;
float fsign(float x);

#endif 

