#include "Odometery.h"
#include "Kalman.h"

Odometery_Pro Cam_Position;
//.....ch.....
MPU_History_Data MPU_Hist;
_Kalman2x2 Cam_X_Kalman,Cam_Y_Kalman,Cam_Z_Kalman;
_Kalman2x2 Ultra_Z_Kalman;
//.........

void Optical_Flow_get_data(_MPC* Mpc,Odometery_Pro *Cam_Vel,MPU_SENSOR *sen,MPU_History_Data* MPU_hist)
{
	MPU_hist->Gyro_X[MPU_hist->counter] = -15.882*1.13*sen->gyro_x;
	MPU_hist->Gyro_Y[MPU_hist->counter++] = 15.882*1.13*sen->gyro_y;
	
	
	MPU_hist->counter %= History_Amount;
	
	if(Mpc->ready)
	{
		Cam_Vel->Vel_X = ((float)Mpc->data[0]) / Velocity_Scale;
		Cam_Vel->Vel_Y = ((float)Mpc->data[1]) / Velocity_Scale;
		
		//Velocity_Correction(Cam_Vel,MPU_hist,30);
		
		Mpc->ready = 0;
	}
	
	
	
}

void PTAM_get_data(_MPC* Mpc,Odometery_Pro *Cam_POS,MPU_SENSOR *sen)
{

	Cam_POS->Acc_x = Mpu.acc_x - Mahony.pure_acc_x;
	Cam_POS->Acc_x = 100*((Cam_Position.Acc_x*9.81f)/Mpu.Gravity);
	
	Cam_POS->Acc_y = Mpu.acc_y - Mahony.pure_acc_y;
	Cam_POS->Acc_y = -100*((Cam_Position.Acc_y*9.81f)/Mpu.Gravity);

	Cam_POS->Acc_z = Mpu.acc_z - Mahony.pure_acc_z;
	Cam_POS->Acc_z = 100*((Cam_Position.Acc_z*9.81f)/Mpu.Gravity);
	
	if(Mpc->ready)
	{
		Cam_POS->POS_X = ((float)Mpc->data[0]) / Velocity_Scale;
		Cam_POS->POS_Y = ((float)Mpc->data[1]) / Velocity_Scale;
		Cam_POS->POS_Z = ((float)Mpc->data[2]) / Velocity_Scale;
		
		Mpc->ready = 0;
	}	
}

void Velocity_Correction(Odometery_Pro *Cam_Vel,MPU_History_Data* MPU_hist,int Delay)
{
	int pointer = MPU_hist->counter;
	if(pointer < Delay)
		pointer = History_Amount - (Delay - pointer);
	else
		pointer -= Delay;
	
	
	MPU_hist->G_X = MPU_hist->Gyro_X[pointer];
	MPU_hist->G_Y = MPU_hist->Gyro_Y[pointer];
	
	Cam_Vel->Vel_X = Cam_Vel->Vel_X - MPU_hist->G_X  ;
	Cam_Vel->Vel_Y = Cam_Vel->Vel_Y - MPU_hist->G_Y  ;
	
	Cam_Vel->Vel_X=Cam_Vel->Vel_X_Last+((DT*15) / ( Cam_Velo_Filter + (DT*15))) * (Cam_Vel->Vel_X-Cam_Vel->Vel_X_Last);
	Cam_Vel->Vel_Y=Cam_Vel->Vel_Y_Last+((DT*15) / ( Cam_Velo_Filter + (DT*15))) * (Cam_Vel->Vel_Y-Cam_Vel->Vel_Y_Last);

	Cam_Vel->Vel_X_Last = Cam_Vel->Vel_X;
	Cam_Vel->Vel_Y_Last = Cam_Vel->Vel_Y;	
}

void Scaling_Cam_POS(Odometery_Pro *Cam_POS,float Height)
{
	if( Height< 35 && Height> 30 && Cam_POS->Scale_state == 1)
	{
		Cam_POS->H1 = Height;
		Cam_POS->Z1 = Cam_POS->POS_Z;
		Cam_POS->Scale_state = 0;
	}
	else if(Height< 105 && Height> 100 && Cam_POS->Scale_state == 0)
	{
		Cam_POS->H2 = Height;
		Cam_POS->Z2 = Cam_POS->POS_Z;
		Cam_POS->Axis_Scale = (Cam_POS->H2 - Cam_POS->H1)/(Cam_POS->Z2-Cam_POS->Z1);
		Cam_POS->Scale_state = 1;
	}
}

void Correct_Cam_POS(Odometery_Pro *Cam_POS)
{
	if( Cam_POS->Axis_Scale !=0 )
	{
		Cam_POS->Modified_POS_X = Cam_POS->POS_X*Cam_POS->Axis_Scale;
		Cam_POS->Modified_POS_Y = Cam_POS->POS_Y*Cam_POS->Axis_Scale;
		Cam_POS->Modified_POS_Z = Cam_POS->H1 + (Cam_POS->POS_Z-Cam_POS->Z1)*Cam_POS->Axis_Scale ;
	
	
	
	kalman_predict_2X2(&Cam_X_Kalman,Cam_Position.Acc_x,Cam_Position.Modified_POS_X,2,8);
	kalman_predict_2X2(&Cam_Y_Kalman,Cam_Position.Acc_y,Cam_Position.Modified_POS_Y,2,8);
	
	if(Cam_Position.Modified_POS_Z !=0)					
	kalman_predict_2X2(&Cam_Z_Kalman,Cam_Position.Acc_z,Cam_Position.Modified_POS_Z,2,8);
	
	Cam_Position.Modified_POS_X = Cam_X_Kalman.STATE[0][0];
	Cam_Position.Modified_POS_Y = Cam_Y_Kalman.STATE[0][0];
	//Cam_Position.Modified_POS_Z = Cam_Z_Kalman.STATE[0][0];					
	
	
	Cam_Position.POS_X_Diff =(float)( Cam_Position.Modified_POS_X - Cam_Position.POS_X_Last)/DT; 
	Cam_Position.POS_X_Last= Cam_Position.Modified_POS_X;
	
	Cam_Position.POS_X_Diff=Cam_Position.POS_X_Diff_Last+(DT / ( FILTER_CAM_DIFF + DT)) * (Cam_Position.POS_X_Diff-Cam_Position.POS_X_Diff_Last);
	Cam_Position.POS_X_Diff_Last = Cam_Position.POS_X_Diff;
	
	
	
	Cam_Position.POS_Y_Diff =(float)( Cam_Position.Modified_POS_Y - Cam_Position.POS_Y_Last)/DT; 
	Cam_Position.POS_Y_Last= Cam_Position.Modified_POS_Y;
	
	Cam_Position.POS_Y_Diff=Cam_Position.POS_Y_Diff_Last+(DT / ( FILTER_CAM_DIFF + DT)) * (Cam_Position.POS_Y_Diff-Cam_Position.POS_Y_Diff_Last);
	Cam_Position.POS_Y_Diff_Last = Cam_Position.POS_Y_Diff;
	
	
		
	Cam_Position.POS_Z_Diff =(float)( Cam_Position.Modified_POS_Z - Cam_Position.POS_Z_Last)/DT; 
	Cam_Position.POS_Z_Last= Cam_Position.Modified_POS_Z;
	
	Cam_Position.POS_Z_Diff=Cam_Position.POS_Z_Diff_Last+(DT / ( FILTER_CAM_DIFF + DT)) * (Cam_Position.POS_Z_Diff-Cam_Position.POS_Z_Diff_Last);
	Cam_Position.POS_Z_Diff_Last = Cam_Position.POS_Z_Diff;
	}
}

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
