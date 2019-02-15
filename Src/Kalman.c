#include "Kalman.h"

_Kalman1x1 z_vel;
_Kalman1x1 orb_pos;

//Multiply two n*m * m*k matrixs. This function developed by Omid
void Any_Matrix_Multiply(int n,int m,int k,float *A, float *B,float *C)
{
	int i=0,j=0,q=0;
	
	for(i=0;i<n;i++)
	{
		for(j=0;j<k;j++)
		{
			*(C+j+i*k)=0;
			for(q=0;q<m;q++)
			{
				*(C+j+i*k)=*(C+j+i*k)+(*(A+q+i*m)) * (*(B+q*k+j));
			}
		}
	}
}

void Matrix_Tranpose(int n,int m,float* src,float* dst)
{
	for(int i=0;i<n;i++)
	{
		for(int j=0;j<m;j++)
			*(dst+j*n+i)= *(src+j+i*m);
	}
}




void Update_Kalman1x1(_Kalman1x1 *Kalman_state,float measurement,float process,float Ez,float Ex)
{
	
	Kalman_state->Ex = Ex;// * Ex * DT * DT;
	Kalman_state->Ez = Ez;// * Ez;
	
	Kalman_state->state =Kalman_state->A * Kalman_state->state + process * Kalman_state->B;
	Kalman_state->P = Kalman_state->P + Ex;
	Kalman_state->K = Kalman_state->P *(1/(Kalman_state->P + Ez));
	Kalman_state->state = Kalman_state->state + Kalman_state->K * (measurement -Kalman_state->state);
	Kalman_state->P = (1 - Kalman_state->K) * Kalman_state->P;
	
}



void Update_Kalman2x2(_Kalman2x2 *Kalman_state,float Ez,float Ex)
{
	
	float Ex2 = Ex*Ex;
	//                 Noise
	Kalman_state->Ez = Ez * Ez;
	
	Kalman_state->Ex[0][0]=Ex2 *(DT * DT * DT * DT)/4;  
	Kalman_state->Ex[0][1]=Ex2 *(DT * DT * DT)/2;
	Kalman_state->Ex[1][0]=Kalman_state->Ex[0][1];
	Kalman_state->Ex[1][1]=Ex2 *(DT * DT );
	
	//                 Q_estimate = A * Q_estimate + B * u;
	
	
}
































