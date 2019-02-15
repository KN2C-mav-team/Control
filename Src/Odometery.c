#include "main.h"
#include "Kalman.h"
#include "Control.h"
#include "par_optical.h"
#include "Odometery.h"

Odometery_Pro Cam_Position;
Odometery_Pro ORB_Position;

_Kalman2x2 Cam_X_Kalman,Cam_Y_Kalman,Cam_Z_Kalman;
_Kalman2x2 Ultra_Z_Kalman;
_Kalman2x2 ORB_X_Kalman,ORB_Y_Kalman,ORB_Z_Kalman;



void Cam_Kalman_init(void)
{
	
	Cam_X_Kalman.STATE[0][0] = 0;
	Cam_X_Kalman.STATE[1][0] = 0;
	
	Cam_X_Kalman.P[0][0] = 0;
	Cam_X_Kalman.P[0][1] = 0;
	Cam_X_Kalman.P[1][0] = 0;
	Cam_X_Kalman.P[1][1] = 0;

	
	Cam_X_Kalman.A[0][0] = 1;
	Cam_X_Kalman.A[0][1] = DT;
	Cam_X_Kalman.A[1][0] = 0;
	Cam_X_Kalman.A[1][1] = 1;
	
	Cam_X_Kalman.B[0][0] = 0.5f*DT*DT;
	Cam_X_Kalman.B[1][0] = DT;
	
	Cam_X_Kalman.C[0][0] = 1;
	Cam_X_Kalman.C[0][1] = 0;
	
	
	Cam_Y_Kalman.STATE[0][0] = 0;
	Cam_Y_Kalman.STATE[1][0] = 0;
	
	Cam_Y_Kalman.P[0][0] = 0;
	Cam_Y_Kalman.P[0][1] = 0;
	Cam_Y_Kalman.P[1][0] = 0;
	Cam_Y_Kalman.P[1][1] = 0;

	
	Cam_Y_Kalman.A[0][0] = 1;
	Cam_Y_Kalman.A[0][1] = DT;
	Cam_Y_Kalman.A[1][0] = 0;
	Cam_Y_Kalman.A[1][1] = 1;
	
	Cam_Y_Kalman.B[0][0] = 0.5f*DT*DT;
	Cam_Y_Kalman.B[1][0] = DT;
	
	Cam_Y_Kalman.C[0][0] = 1;
	Cam_Y_Kalman.C[0][1] = 0;
	
	Cam_Z_Kalman.STATE[0][0] = 0;
	Cam_Z_Kalman.STATE[1][0] = 0;
	
	Cam_Z_Kalman.P[0][0] = 0;
	Cam_Z_Kalman.P[0][1] = 0;
	Cam_Z_Kalman.P[1][0] = 0;
	Cam_Z_Kalman.P[1][1] = 0;

	
	Cam_Z_Kalman.A[0][0] = 1;
	Cam_Z_Kalman.A[0][1] = DT;
	Cam_Z_Kalman.A[1][0] = 0;
	Cam_Z_Kalman.A[1][1] = 1;
	
	Cam_Z_Kalman.B[0][0] = 0.5f*DT*DT;
	Cam_Z_Kalman.B[1][0] = DT;
	
	Cam_Z_Kalman.C[0][0] = 1;
	Cam_Z_Kalman.C[0][1] = 0;
	
}




void kalman_predict_2X2(_Kalman2x2* K_F_DATA,float acc,float mesurement,float MeasureNoise,float ProNoise)
{
	float inn=0,S=0;
	
	
	K_F_DATA->Ex[0][0]=ProNoise;
	K_F_DATA->Ex[0][1]=0;
	K_F_DATA->Ex[1][0]=0;
	K_F_DATA->Ex[1][1]=ProNoise;

	K_F_DATA->Ez = MeasureNoise;//measurement  nois

	Matrix_Tranpose(2,2,&K_F_DATA->A[0][0],&K_F_DATA->AT[0][0]);
	Matrix_Tranpose(1,2,&K_F_DATA->C[0][0],&K_F_DATA->CT[0][0]);
//x_state......

//Matrix_Multiply_3x3_3x1(A,STATE,X);//A X

	Any_Matrix_Multiply(2,2,1,&K_F_DATA->A[0][0],&K_F_DATA->STATE[0][0],&K_F_DATA->X[0][0]);
	Any_Matrix_Multiply(2,1,1,&K_F_DATA->B[0][0],&acc,&K_F_DATA->BU[0][0]);


	K_F_DATA->X[0][0]+=K_F_DATA->BU[0][0];
	K_F_DATA->X[1][0]+=K_F_DATA->BU[1][0];
	
	
	inn=mesurement-K_F_DATA->X[0][0];
	
	//S=p[1][1]+Sz;//C P C'
	Any_Matrix_Multiply(1,2,2,&K_F_DATA->C[0][0],&K_F_DATA->P[0][0],&K_F_DATA->T5[0][0]);//T5=C P
	Any_Matrix_Multiply(1,2,1,&K_F_DATA->T5[0][0],&K_F_DATA->CT[0][0],&S);//C P C'
	S+=K_F_DATA->Ez;
	//K:
	//Matrix_Multiply_3x3(A, p,T);// T=A P
	Any_Matrix_Multiply(2,2,2,&K_F_DATA->A[0][0],&K_F_DATA->P[0][0],&K_F_DATA->T[0][0]);
	//Matrix_Multiply_3x3_3x1(T,CT,K);//K= A P C'
	Any_Matrix_Multiply(2,2,1,&K_F_DATA->T[0][0],&K_F_DATA->CT[0][0],&K_F_DATA->K[0][0]);
	K_F_DATA->K[0][0]/=S;
	K_F_DATA->K[1][0]/=S;

	//x:
	K_F_DATA->STATE[0][0]=K_F_DATA->X[0][0]+inn*K_F_DATA->K[0][0];
	K_F_DATA->STATE[1][0]=K_F_DATA->X[1][0]+inn*K_F_DATA->K[1][0];
	//P:
	//Matrix_Multiply_3x3(A, p,T);//T=A P
	Any_Matrix_Multiply(2,2,2,&K_F_DATA->A[0][0],&K_F_DATA->P[0][0],&K_F_DATA->T[0][0]);
	//Matrix_Multiply_3x3(T, AT,T2);//T2=A P A'
	Any_Matrix_Multiply(2,2,2,&K_F_DATA->T[0][0],&K_F_DATA->AT[0][0],&K_F_DATA->T2[0][0]);

	//Matrix_Multiply_3x3(p, AT,T);//T=P A'
	Any_Matrix_Multiply(2,2,2,&K_F_DATA->P[0][0],&K_F_DATA->AT[0][0],&K_F_DATA->T[0][0]);

	//Matrix_Multiply_3x1_1x3(K,C,T3);//T3=K C
	Any_Matrix_Multiply(2,1,2,&K_F_DATA->K[0][0],&K_F_DATA->C[0][0],&K_F_DATA->T3[0][0]);

	//Matrix_Multiply_3x3(T3,T,T4);//T4=K C P A'
	Any_Matrix_Multiply(2,2,2,&K_F_DATA->T3[0][0],&K_F_DATA->T[0][0],&K_F_DATA->T4[0][0]);

	K_F_DATA->P[0][0]=K_F_DATA->T2[0][0]-K_F_DATA->T4[0][0]+K_F_DATA->Ex[0][0];
	K_F_DATA->P[0][1]=K_F_DATA->T2[0][1]-K_F_DATA->T4[0][1]+K_F_DATA->Ex[0][1];
	K_F_DATA->P[1][0]=K_F_DATA->T2[1][0]-K_F_DATA->T4[1][0]+K_F_DATA->Ex[1][0];
	K_F_DATA->P[1][1]=K_F_DATA->T2[1][1]-K_F_DATA->T4[1][1]+K_F_DATA->Ex[1][1];

}
void ORB_Kalman_init(void)
{
	
	ORB_X_Kalman.STATE[0][0] = 0;
	ORB_X_Kalman.STATE[1][0] = 0;
	ORB_X_Kalman.P[0][0] = 0;
	ORB_X_Kalman.P[0][1] = 0;
	ORB_X_Kalman.P[1][0] = 0;
	ORB_X_Kalman.P[1][1] = 0;

	
	ORB_X_Kalman.A[0][0] = 1;
	ORB_X_Kalman.A[0][1] = DT;
	ORB_X_Kalman.A[1][0] = 0;
	ORB_X_Kalman.A[1][1] = 1;
	
	ORB_X_Kalman.B[0][0] = 0.5f*DT*DT;
	ORB_X_Kalman.B[1][0] = DT;
	
	ORB_X_Kalman.C[0][0] = 1;
	ORB_X_Kalman.C[0][1] = 0;
	
	///////////////////////////////////
	
	ORB_Y_Kalman.STATE[0][0] = 0;
	ORB_Y_Kalman.STATE[1][0] = 0;
	ORB_Y_Kalman.P[0][0] = 0;
	ORB_Y_Kalman.P[0][1] = 0;
	ORB_Y_Kalman.P[1][0] = 0;
	ORB_Y_Kalman.P[1][1] = 0;

	
	ORB_Y_Kalman.A[0][0] = 1;
	ORB_Y_Kalman.A[0][1] = DT;
	ORB_Y_Kalman.A[1][0] = 0;
	ORB_Y_Kalman.A[1][1] = 1;
	
	ORB_Y_Kalman.B[0][0] = 0.5f*DT*DT;
	ORB_Y_Kalman.B[1][0] = DT;
	
	ORB_Y_Kalman.C[0][0] = 1;
	ORB_Y_Kalman.C[0][1] = 0;
}