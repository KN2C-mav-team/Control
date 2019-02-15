#ifndef H_Height_H
#define H_Height_H

#include "stm32f4xx_hal.h"
#include "Telemetri.h"
#include "math.h"
#include "Kalman.h"


//************************ultra*******************************
#define Ultra_Trig_ON  HAL_GPIO_WritePin(GPIOD,GPIO_PIN_0,1)
#define Ultra_Trig_OFF HAL_GPIO_WritePin(GPIOD,GPIO_PIN_0,0)
#define Max_Ultra_Thr  450 

#define	F_CUT_Ultra			   		6 
#define FILTER_Ultra          1/(2*PI*F_CUT_Ultra		      	)



typedef struct 
{	
	uint16_t Begine;
	uint16_t End;
	float    Diff_Time;
	uint16_t last_Time;
	float    real;
	float    last_real;
	float		 point;
	float    last_point;
	float    diff;
	float    last_diff;
	float    vel;
	float    last_vel;
	float    K_point;
	float    K_last_point;
	float 	 lpf_Gain;
	uint16_t State;
	uint8_t  ready;
	uint8_t  fail;
}_Ultra;


extern _Ultra Ultra;
extern float vel_z;

void  Read_Srf(TIM_HandleTypeDef _htim,_Ultra* ultra);
float Vel_z(_Kalman1x1 *Kalman_state,_Ultra *ultra,float acc);
void  ultra_filter_lpf(_Ultra* ultra);
void Ultra_Kalman_init(void);
#endif 

