#ifndef H_ODOMETERY_H
#define H_ODOMETERY_H

#include "stm32f4xx_hal.h"
#include "IMU.h"
#include "MPC.h"
#include "Kalman.h"
#include "par_optical.h"


#define Velocity_Scale 			1000
#define History_Amount      40
#define F_CUT_CAM_VEL				3
#define Cam_Velo_Filter          1/(2*PI*F_CUT_CAM_VEL)
#define F_CUT_VEL_ORB_HPF   3
#define F_VEL_ORB_HPF            1/(2*PI*F_CUT_VEL_ORB_HPF)
typedef struct
{
	float Vel_X,Vel_Y,Vel_X_hpf,Vel_Y_hpf,Vel_X_hpf_last,Vel_Y_hpf_last,Vel_X_NEW,Vel_Y_NEW,Vel_X_NEW_hpf,Vel_Y_NEW_hpf;
	float Vel_X_Last,Vel_Y_Last,Vel_X_NEW_last,Vel_Y_NEW_last,Vel_X_NEW_hpf_last,Vel_Y_NEW_hpf_last,Vel_Y_NEW_NEW,Vel_X_NEW_NEW;
	
	float POS_X,POS_Y,POS_Z;
	float POS_X_Last,POS_Y_Last,POS_Z_Last;
	float POS_X_Diff,POS_Y_Diff,POS_Z_Diff;
	float POS_X_Diff_Last,POS_Y_Diff_Last,POS_Z_Diff_Last;
	
	float Acc_x,Acc_y,Acc_z;
	int Scale_state,ORB_Scale_state;
	float H1,H2,Z1,Z2,Axis_Scale,ORB_Axis_Scale;
	
	
	int data_check;
	int filter_flag;
	int RUN_FLAG;
	
	float Modified_POS_X, Modified_POS_Y, Modified_POS_Z;
}Odometery_Pro;

extern Odometery_Pro Cam_Position;
extern Odometery_Pro ORB_Position;
extern _Kalman2x2 Cam_X_Kalman,Cam_Y_Kalman,Cam_Z_Kalman;
extern _Kalman2x2 Ultra_Z_Kalman;

void kalman_predict_2X2(_Kalman2x2* K_F_DATA,float acc,float mesurement,float MeasureNoise,float ProNoise);
void Cam_Kalman_init(void);
void Correct_ORB_POS(Odometery_Pro *ORB_POS);
void Scaling_ORB_POS(Odometery_Pro *ORB_POS,_MPC *MPC,float Height);
void ORB_get_data(_MPC* Mpc,_MPC* Mpc_2,Odometery_Pro *ORB_POS,MPU_SENSOR *sen,optical_par_struct *opti);
void ORB_Kalman_init(void);
void window_get_data(_MPC* Mpc);


#endif 
