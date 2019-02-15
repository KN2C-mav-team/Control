#ifndef  H_Optical_Flow_H
#define  H_Optical_Flow_H

#include "stm32f4xx_hal.h"
#include "IMU.h"
#include "MPC.h"
#include "mpu6050.h"
#include "math.h"
#include "Kalman.h"

#define  pixel_size  0.264583333 //mm
#define  Tales       12.5
#define  fps         30 

#define acc_noise    1
#define opti_noise   5

#define	F_CUT_Gyro			   	 4.0f 
#define FILTER_Gyro          1/(2*PI*F_CUT_Gyro		      	)

#define	F_CUT_Opti			   	 6.0f 
#define FILTER_Opti          1/(2*PI*F_CUT_Opti		      	)

#define acc_threshold        4

#define focale_len           360//291.61


typedef struct
{
	
	float Vel_X,Vel_Y;
	float Vel_X_Last,Vel_Y_Last;
	float Gyro_X,last_Gyro_X;
	float Gyro_Y,last_Gyro_Y;
	float Gyro_X_sum;
	float Gyro_Y_sum;
	float Vel_correction_X;
	float Vel_correction_Y;
	float last_Vel_correction_X;
	float last_Vel_correction_Y;
	float real_Vel_X;
	float real_Vel_Y;
	float acc_X;
	float acc_Y;
	int count;
	int data_reay;
	float Pos_X;
	float Pos_Y;
	float Pos_Z;
	
}optical_flow_HandleTypeDef;

extern optical_flow_HandleTypeDef optical;
extern _Kalman1x1 x_vel,y_vel;

void optical_get_data(_MPC* Mpc,optical_flow_HandleTypeDef *opti,MPU_SENSOR *sen);
void do_optical(_MPC* Mpc,optical_flow_HandleTypeDef *opti,MPU_SENSOR *sen);

void optical_kalman_init();
void opti_vel_update(optical_flow_HandleTypeDef *opti,MPU_SENSOR *sen);






#endif