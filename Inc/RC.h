#ifndef H_RC_H
#define H_RC_H
#include "stm32f4xx_hal.h"
#include "mpu6050.h"
#include "Telemetri.h"
#include "EEPROM.h"


#define Throttle_range     400
#define angle_range        (20.0f)


#define Roll_offset        0
#define Pitch_offset       -10.5

#define YawThreshold       2
#define RC_noise           2000  //**//

#define	F_CUT_RC			      25
#define FILTER_RC           1/(2*PI*F_CUT_RC)


//channel define
#define Throttle_channel    6
#define Roll_channel        5
#define Pitch_channel       7
#define Yaw_channel         4
#define RC_SW_channel       2
#define HOV_THR_channel     1
#define RC_TRIM_channel     0
#define HOV_PIT_channel     3 


typedef struct 
{
	uint16_t RC_channel[8];
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
	uint8_t fail;
	I2C_HandleTypeDef I2C;
	
	
}_RC;

extern _RC RC;

void RC_Read(_RC* Rc,int State);
void RC_Init(_RC* Rc,I2C_HandleTypeDef hi2cx ,char calib);
void RC_Calib(_RC* Rc,char calib);
void RC_2_SetPoint(_RC* Rc);
void RC_Read_EEPROM(_RC* Rc);
float fsign(float x);

#endif 

