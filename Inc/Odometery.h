#ifndef H_ODOMETERY_H
#define H_ODOMETERY_H

#include "stm32f4xx_hal.h"
#include "IMU.h"
#include "MPC.h"



#define Velocity_Scale 			1000
#define History_Amount      40
#define F_CUT_CAM_VEL				3
#define Cam_Velo_Filter          1/(2*PI*F_CUT_CAM_VEL)
typedef struct
{
	float Vel_X,Vel_Y;
	float Vel_X_Last,Vel_Y_Last;
	
	float POS_X,POS_Y,POS_Z;
	float POS_X_Last,POS_Y_Last,POS_Z_Last;
	float POS_X_Diff,POS_Y_Diff,POS_Z_Diff;
	float POS_X_Diff_Last,POS_Y_Diff_Last,POS_Z_Diff_Last;
	
	float Acc_x,Acc_y,Acc_z;
	int Scale_state;
	float H1,H2,Z1,Z2,Axis_Scale;
	
	float Modified_POS_X, Modified_POS_Y, Modified_POS_Z;
}Odometery_Pro;

typedef struct
{
	float Gyro_X[History_Amount],Gyro_Y[History_Amount];
	float G_X,G_Y;
	int counter;
}MPU_History_Data;

extern MPU_History_Data MPU_Hist;
extern Odometery_Pro Cam_Position;
extern _Kalman2x2 Cam_X_Kalman,Cam_Y_Kalman,Cam_Z_Kalman;
extern _Kalman2x2 Ultra_Z_Kalman;

void Optical_Flow_get_data(_MPC* Mpc,Odometery_Pro *Cam_Vel,MPU_SENSOR *sen,MPU_History_Data* MPU_hist);
void PTAM_get_data(_MPC* Mpc,Odometery_Pro *Cam_POS,MPU_SENSOR *sen);
void Velocity_Correction(Odometery_Pro *Cam_Vel,MPU_History_Data* MPU_hist,int Delay);
void kalman_predict_2X2(_Kalman2x2* K_F_DATA,float acc,float mesurement,float MeasureNoise,float ProNoise);
void Cam_Kalman_init(void);
void Correct_Cam_POS(Odometery_Pro *Cam_POS);
void Scaling_Cam_POS(Odometery_Pro *Cam_POS,float Height);



#endif 
