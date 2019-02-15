#ifndef  H_par_optical_H
#define H_par_optical_H

#include "stm32f4xx_hal.h"
#include "IMU.h"
#include "MPC.h"
#include "mpu6050.h"
#include "math.h"
#include "Kalman.h"

#define focal_pix   3// 290

#define acc_noise    3
#define opti_noise   25

#define acc_threshold        8

#define	F_CUT_Gyro1			   	4.0f 
#define FILTER_Gyro          1/(2*PI*F_CUT_Gyro1)

#define	F_CUT_Opti			   	 4.0f 
#define FILTER_Opti          1/(2*PI*F_CUT_Opti)

#define fps       30.0f
#define optical_sample_time 0.0155f
#define ALTITUDE_HOVER     130

typedef struct
{
	
	float delta_X,delta_Y;
	float delta_X_Last,delta_Y_Last;
	float Gyro_X,last_Gyro_X,diff_Gyro_X,buff_Gyro_X;
	float Gyro_Y,last_Gyro_Y,diff_Gyro_Y,buff_Gyro_Y;
	float Gyro_X_sum;
	float Gyro_Y_sum;
	float delta_X_correct;
	float delta_Y_correct;
	
	float integral_x;
	float integral_y;
	
	float x_correction;
	float y_correction;
	
	float last_delta_X_correct;
	float last_delta_Y_correct;
	

	float real_vel_X , last_real_vel_X;
	float real_vel_Y , last_real_vel_Y;
	int count;
	int data_ready;
	float Pos_X;
	float Pos_Y;
	float Pos_Z;
	
	float acc_X;
	float acc_Y;
	
	int for_kooft;
	int gyro_buff_number;
	
	int data_check;
	
	int flag_for_diff;
	
	int x_vel_orb,y_vel_orb,last_x_vel_orb,last_y_vel_orb,x_acc,y_acc,last_x_orb,last_y_orb;
  float imu_x_vel,imu_y_vel;
	
}optical_par_struct;

extern optical_par_struct optical_par;

//void do_optical_par(_MPC *MPC,_MPC* Mpc_2,MPU_SENSOR *MPU_sen,optical_par_struct *opti);
void  do_optical_par(_MPC *MPC,_MPC *MPC_2,MPU_SENSOR *MPU_sen,optical_par_struct *opti ,_Ultra *ultra);
void optical_kalman_init();

extern _Kalman1x1 x_vel,y_vel;
#endif