#include "main.h"
#include "Odometery.h"
#include "Kalman.h"
#include "Control.h"
#include "par_optical.h"

int start_flag_orb=0;
Odometery_Pro Cam_Position;
Odometery_Pro ORB_Position;
//optical_par_struct optical_par;
//.....ch.....
MPU_History_Data MPU_Hist;
_Kalman2x2 Cam_X_Kalman,Cam_Y_Kalman,Cam_Z_Kalman;
_Kalman2x2 Ultra_Z_Kalman;
_Kalman2x2 ORB_X_Kalman,ORB_Y_Kalman,ORB_Z_Kalman;
//.........
//************************OPTICAL FLOW********************************************************
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

//****************************ORB**********************************************
void ORB_get_data(_MPC* Mpc,_MPC* Mpc_2,Odometery_Pro *ORB_POS,MPU_SENSOR *sen,optical_par_struct *opti)
//void ORB_get_data(_MPC* Mpc,Odometery_Pro *ORB_POS,MPU_SENSOR *sen)
{
  start_flag_orb++;
	//if(start_flag_orb>=500){
	
	ORB_POS->Acc_y = Mpu.acc_x - Mahony.pure_acc_x;
	ORB_POS->Acc_y = -100*((ORB_Position.Acc_y*9.81f)/Mpu.Gravity);
	
	ORB_POS->Acc_x = Mpu.acc_y - Mahony.pure_acc_y;
	ORB_POS->Acc_x = 100*((ORB_Position.Acc_x*9.81f)/Mpu.Gravity);

	ORB_POS->Acc_z = Mpu.acc_z - Mahony.pure_acc_z;
	ORB_POS->Acc_z = 100*((ORB_Position.Acc_z*9.81f)/Mpu.Gravity);
	
	ORB_POS->Vel_X=ORB_POS->Acc_x*DT + ORB_POS->Vel_X;
	ORB_POS->Vel_Y=ORB_POS->Acc_y*DT + ORB_POS->Vel_Y;
	
	ORB_POS->Vel_X_hpf=(F_VEL_ORB_HPF/(F_VEL_ORB_HPF + DT) )*(ORB_POS->Vel_X - ORB_POS->Vel_X_Last+ 	ORB_POS->Vel_X_hpf_last) ;
	ORB_POS->Vel_Y_hpf=(F_VEL_ORB_HPF/(F_VEL_ORB_HPF + DT) )*(ORB_POS->Vel_Y - ORB_POS->Vel_Y_Last+ 	ORB_POS->Vel_Y_hpf_last) ;
  	
	ORB_POS->Vel_X_NEW = ORB_POS->Vel_X_hpf*DT +  ORB_POS->Vel_X_NEW ;
	ORB_POS->Vel_Y_NEW = ORB_POS->Vel_Y_hpf*DT + ORB_POS->Vel_Y_NEW;
		
	ORB_POS->Vel_X_NEW_hpf=(F_VEL_ORB_HPF/(F_VEL_ORB_HPF + DT) )*(ORB_POS->Vel_X_NEW - ORB_POS->Vel_X_NEW_last+ 	ORB_POS->Vel_X_NEW_hpf_last) ;
	ORB_POS->Vel_Y_NEW_hpf=(F_VEL_ORB_HPF/(F_VEL_ORB_HPF + DT) )*(ORB_POS->Vel_Y_NEW - ORB_POS->Vel_Y_NEW_last+ 	ORB_POS->Vel_Y_NEW_hpf_last) ;
  
//	ORB_POS->Vel_X_NEW_hpf=1*ORB_POS->Vel_X_NEW_hpf;
//	ORB_POS->Vel_Y_NEW_hpf=1*ORB_POS->Vel_Y_NEW_hpf;
//	
//	ORB_POS->Vel_X_NEW_NEW= (ORB_POS->Vel_X_NEW_hpf*DT +  ORB_POS->Vel_X_NEW_NEW );
//	ORB_POS->Vel_Y_NEW_NEW = (ORB_POS->Vel_Y_NEW_hpf*DT + ORB_POS->Vel_Y_NEW_NEW);
	


	if(fabs(ORB_POS->Vel_X_NEW_hpf) < 0.004){
	ORB_POS->Vel_X_NEW=ORB_POS->Vel_X_NEW/3;
	}	

	if(fabs(ORB_POS->Vel_Y_NEW_hpf) < 0.004){
	ORB_POS->Vel_Y_NEW=ORB_POS->Vel_Y_NEW/3;
	}
	
	 ORB_POS->Vel_X_NEW_last = ORB_POS->Vel_X_NEW ;
	 ORB_POS->Vel_Y_NEW_last = ORB_POS->Vel_Y_NEW ;


	
//	if(fabs(ORB_POS->Vel_X_NEW_hpf) > 0.095){
//	ORB_POS->Vel_X_hpf_last = 	ORB_POS->Vel_X_hpf;
//	}
//		if(fabs(ORB_POS->Vel_X_NEW_hpf) < 0.095){
//	ORB_POS->Vel_X_hpf_last = 	0;
//	}
//		
//		if(fabs(ORB_POS->Vel_Y_NEW_hpf) > 0.095){
//	ORB_POS->Vel_Y_hpf_last = 	ORB_POS->Vel_Y_hpf;
//	}
//		if(fabs(ORB_POS->Vel_Y_NEW_hpf) < 0.095){
//	ORB_POS->Vel_Y_hpf_last = 	0;
//	}
	
	ORB_POS->Vel_X_Last=ORB_POS->Vel_X;
	ORB_POS->Vel_Y_Last=ORB_POS->Vel_Y;
	
	ORB_POS->Vel_X_hpf_last=ORB_POS->Vel_X_hpf;
	ORB_POS->Vel_Y_hpf_last=ORB_POS->Vel_Y_hpf;
	
	ORB_POS->data_check++;
	ORB_Position.filter_flag=0;
//	if(Mpc_2->ready)
//	{
		ORB_POS->Vel_Y=0;
		
		ORB_Position.filter_flag=1;
		ORB_POS->data_check=0;
		
		ORB_Position.POS_X=((float)MPC_2.data[0] / 1);
		ORB_Position.POS_Y=((float)MPC_2.data[1] / 1);
		ORB_position.X.point =  (float)MPC_2.data[0]; 
		ORB_position.Y.point 	=(float)MPC_2.data[1];
//		ORB_Position.POS_Z=((float)MPC.data[2] / 1);
		
//		ORB_position.X.setpoint = (float)MPC.data[3];
//		ORB_position.Y.setpoint = (float)MPC.data[4];
//		ORB_position.Z.setpoint = (float)MPC.data[5];
		ORB_position.X.setpoint = 0;
		ORB_position.Y.setpoint = 0;
		ORB_position.Z.setpoint = 0;

		
  	Mpc_2->ready = 0;
	//}	
}
//}

void Scaling_ORB_POS(Odometery_Pro *ORB_POS,_MPC *MPC,float Height)
{
	  start_flag_orb++;
	//if(start_flag_orb>=500){
	if( Height< 35 && Height> 30 && ORB_POS->ORB_Scale_state == 1)
	{
		ORB_POS->H1 = Height;
		ORB_POS->Z1 = MPC->data[2];
		ORB_POS->ORB_Scale_state = 0;
	}
	else if(Height< 105 && Height> 100 && ORB_POS->ORB_Scale_state == 0)
	{
		ORB_POS->H2 = Height;
		ORB_POS->Z2 = MPC->data[2];
		ORB_POS->ORB_Axis_Scale = (ORB_POS->H2 - ORB_POS->H1)/(ORB_POS->Z2-ORB_POS->Z1);
		ORB_POS->ORB_Scale_state = 1;
	}
//}
}
void Correct_ORB_POS(Odometery_Pro *ORB_POS)
{
	  start_flag_orb++;
	//if(start_flag_orb>=500){
	//if( ORB_POS->ORB_Axis_Scale !=0 )
	//{
//		ORB_POS->Modified_POS_X = ORB_POS->POS_X*ORB_POS->ORB_Axis_Scale;
//		ORB_POS->Modified_POS_Y = ORB_POS->POS_Y*ORB_POS->ORB_Axis_Scale;
//		ORB_POS->Modified_POS_Z = ORB_POS->POS_Z*ORB_POS->ORB_Axis_Scale;
		ORB_POS->Modified_POS_X = ( ORB_POS->POS_X*Ultra.point*1.5346/640) - ( Ultra.point*tan(PI*Mahony.Pitch/180));
		ORB_POS->Modified_POS_Y = ( ORB_POS->POS_Y*Ultra.point*1.5346/480) + ( Ultra.point*tan(PI*Mahony.Roll/180));
		ORB_POS->Modified_POS_Z = ORB_POS->POS_Z;
		//ORB_POS->Modified_POS_Z = ORB_POS->H1 + (ORB_POS->POS_Z-Cam_POS->Z1)*ORB_POS->ORB_Axis_Scale ;   // cheraaa ?
	 
  	orb_pos.need_2_res = 1;
    Update_Kalman1x1(&orb_pos,ORB_Position.Modified_POS_X,ORB_Position.Vel_X_NEW,7777,5);	
		orb_pos.last_state=orb_pos.state;
		
    kalman_predict_2X2(&Cam_X_Kalman,ORB_Position.Acc_x,ORB_Position.Modified_POS_X,3,7);  //4,03
    kalman_predict_2X2(&Cam_Y_Kalman,ORB_Position.Acc_y,ORB_Position.Modified_POS_Y,3,7);
//	
	if(ORB_Position.Modified_POS_Z !=0)					
	kalman_predict_2X2(&Cam_Z_Kalman,ORB_Position.Acc_z,ORB_Position.Modified_POS_Z,2,8);
//	
	ORB_Position.Modified_POS_X = Cam_X_Kalman.STATE[0][0];
   	ORB_Position.Modified_POS_Y = Cam_Y_Kalman.STATE[0][0];
//    ORB_Position.Modified_POS_Z = Cam_Z_Kalman.STATE[0][0];			

	
	if(ORB_Position.filter_flag==1){
	
	ORB_Position.POS_X_Diff = (float)( ORB_Position.Modified_POS_X - ORB_Position.POS_X_Last)/(7.5*DT); 
	ORB_Position.POS_X_Last = ORB_Position.Modified_POS_X;
		
	ORB_Position.POS_Y_Diff = (float)( ORB_Position.Modified_POS_Y - ORB_Position.POS_Y_Last)/(7.5*DT); 
  ORB_Position.POS_Y_Last = ORB_Position.Modified_POS_Y;
		

	}
	
			ORB_position.X.point = ORB_Position.Modified_POS_X;
		  ORB_position.Y.point = ORB_Position.Modified_POS_Y;
					
			ORB_position.X.diff_point = ORB_Position.POS_X_Diff;
			ORB_position.Y.diff_point = ORB_Position.POS_Y_Diff;	
	
	ORB_position.X.point = ORB_position.X.last_point+(DT / ( FILTER_CAM_DIFF + DT)) * (ORB_position.X.point - ORB_position.X.last_point);
	ORB_position.Y.point = ORB_position.Y.last_point+(DT / ( FILTER_CAM_DIFF + DT)) * (ORB_position.Y.point - ORB_position.Y.last_point);
	
	ORB_position.Y.last_point = ORB_position.Y.point;
	ORB_position.X.last_point = ORB_position.X.point;
	
	
	
////	
  ORB_Position.POS_X_Diff = ORB_Position.POS_X_Diff_Last+(DT / ( FILTER_CAM_DIFF + DT)) * (ORB_Position.POS_X_Diff-ORB_Position.POS_X_Diff_Last);
	ORB_Position.POS_X_Diff_Last = ORB_Position.POS_X_Diff;
//	
	ORB_Position.POS_Y_Diff = ORB_Position.POS_Y_Diff_Last+(DT / ( FILTER_CAM_DIFF + DT)) * (ORB_Position.POS_Y_Diff-ORB_Position.POS_Y_Diff_Last);
	ORB_Position.POS_Y_Diff_Last = ORB_Position.POS_Y_Diff;
//	
//	
//		
	ORB_Position.POS_Z_Diff =(float)( ORB_Position.Modified_POS_Z - ORB_Position.POS_Z_Last)/DT; 
	ORB_Position.POS_Z_Last = ORB_Position.Modified_POS_Z;
//	
//	ORB_Position.POS_Z_Diff = ORB_Position.POS_Z_Diff_Last+(DT / ( FILTER_CAM_DIFF + DT)) * (ORB_Position.POS_Z_Diff-ORB_Position.POS_Z_Diff_Last);
	ORB_Position.POS_Z_Diff_Last = ORB_Position.POS_Z_Diff;

	//}
//}
}
//**********************************PTAM****************************
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
//	if( Cam_POS->Axis_Scale !=0 )
//	{
//		Cam_POS->Modified_POS_X = Cam_POS->POS_X*Cam_POS->Axis_Scale;
//		Cam_POS->Modified_POS_Y = Cam_POS->POS_Y*Cam_POS->Axis_Scale;
//		Cam_POS->Modified_POS_Z = Cam_POS->H1 + (Cam_POS->POS_Z-Cam_POS->Z1)*Cam_POS->Axis_Scale ;
//	
//	
//	
//	kalman_predict_2X2(&Cam_X_Kalman,Cam_Position.Acc_x,Cam_Position.Modified_POS_X,2,8);
//	kalman_predict_2X2(&Cam_Y_Kalman,Cam_Position.Acc_y,Cam_Position.Modified_POS_Y,2,8);
//	
//	if(Cam_Position.Modified_POS_Z !=0)					
//	kalman_predict_2X2(&Cam_Z_Kalman,Cam_Position.Acc_z,Cam_Position.Modified_POS_Z,2,8);
//	
//	Cam_Position.Modified_POS_X = Cam_X_Kalman.STATE[0][0];
//	Cam_Position.Modified_POS_Y = Cam_Y_Kalman.STATE[0][0];
//	//Cam_Position.Modified_POS_Z = Cam_Z_Kalman.STATE[0][0];					
//	
//	
//	Cam_Position.POS_X_Diff =(float)( Cam_Position.Modified_POS_X - Cam_Position.POS_X_Last)/DT; 
//	Cam_Position.POS_X_Last= Cam_Position.Modified_POS_X;
//	
//	Cam_Position.POS_X_Diff=Cam_Position.POS_X_Diff_Last+(DT / ( FILTER_CAM_DIFF + DT)) * (Cam_Position.POS_X_Diff-Cam_Position.POS_X_Diff_Last);
//	Cam_Position.POS_X_Diff_Last = Cam_Position.POS_X_Diff;
//	
//	
//	
//	Cam_Position.POS_Y_Diff =(float)( Cam_Position.Modified_POS_Y - Cam_Position.POS_Y_Last)/DT; 
//	Cam_Position.POS_Y_Last= Cam_Position.Modified_POS_Y;
//	
//	Cam_Position.POS_Y_Diff=Cam_Position.POS_Y_Diff_Last+(DT / ( FILTER_CAM_DIFF + DT)) * (Cam_Position.POS_Y_Diff-Cam_Position.POS_Y_Diff_Last);
//	Cam_Position.POS_Y_Diff_Last = Cam_Position.POS_Y_Diff;
//	
//	
//		
//	Cam_Position.POS_Z_Diff =(float)( Cam_Position.Modified_POS_Z - Cam_Position.POS_Z_Last)/DT; 
//	Cam_Position.POS_Z_Last= Cam_Position.Modified_POS_Z;
//	
//	Cam_Position.POS_Z_Diff=Cam_Position.POS_Z_Diff_Last+(DT / ( FILTER_CAM_DIFF + DT)) * (Cam_Position.POS_Z_Diff-Cam_Position.POS_Z_Diff_Last);
//	Cam_Position.POS_Z_Diff_Last = Cam_Position.POS_Z_Diff;
//	}
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