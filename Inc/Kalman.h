#ifndef H_KALMAN_H
#define H_KALMAN_H

#include "stm32f4xx_hal.h"
#include "Define.h"


typedef struct {
	
	float A[2][2],AT[2][2];
	float B[2][1];
	float C[1][2];
	float CT[2][1];
	
	float STATE[2][1];
	
	float X[2][1];
	
	float Ex[2][2];
	float Ez;
	
	float P[2][2];
	float K[2][1];
	
	float T[2][2];
	float T2[2][2];
	float T3[2][2];
	float T4[2][2];
	float T5[1][2];
	float BU[2][1];
	
	
	
}_Kalman2x2;




typedef struct {
	
	float A;
	float B;
	float C;
	
	float state;
	float last_state;
	float acc_kalman_output;
	float Ex;
	float Ez;
	
	float P;
	float K;
	int need_2_res;
	
}_Kalman1x1;


extern _Kalman1x1 z_vel;
extern _Kalman1x1 orb_pos;
void Any_Matrix_Multiply(int n,int m,int k,float *A, float *B,float *C);
void Matrix_Tranpose(int n,int m,float* src,float* dst);
void Update_Kalman2x2(_Kalman2x2 *Kalman_state,float Ez,float Ex);
void Update_Kalman1x1(_Kalman1x1 *Kalman_state,float measurement,float process,float Ez,float Ex);

#endif 


