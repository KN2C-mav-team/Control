#include "Optical_Flow.h"

optical_flow_HandleTypeDef optical;
_Kalman1x1 x_vel,y_vel;



void do_optical(_MPC* Mpc,optical_flow_HandleTypeDef *opti,MPU_SENSOR *sen)
{
	if(counter > 300)
	{
		optical_get_data(Mpc,opti,sen);
		opti_vel_update(opti,sen);
	}
}

void optical_get_data(_MPC* Mpc,optical_flow_HandleTypeDef *opti,MPU_SENSOR *sen)
{
	
	
	opti->acc_X = sen->acc_y - Mahony.pure_acc_y;
	opti->acc_X = -100*((opti->acc_X*9.81f)/sen->Gravity);
	if(fabs(opti->acc_X) < acc_threshold)
		opti->acc_X = 0;
	
	
	opti->acc_Y = sen->acc_x - Mahony.pure_acc_x;
	opti->acc_Y = -100*((opti->acc_Y*9.81f)/sen->Gravity);
	
	if(fabs(opti->acc_Y) < acc_threshold)
		opti->acc_Y = 0;
	
	opti->Gyro_X_sum += ToDeg(sen->gyro_x);
	opti->Gyro_Y_sum += ToDeg(sen->gyro_y);
	opti->count++;
	
	if(Mpc->ready)
	{
		opti->Vel_X = ((float)Mpc->data[0])/(1);
		opti->Vel_Y = ((float)Mpc->data[1])/(1);
		
		
		opti->Gyro_X = opti->Gyro_X_sum;
		opti->Gyro_Y = opti->Gyro_Y_sum;
		
	
		opti->Gyro_X = opti->last_Gyro_X +((0.033)/(FILTER_Gyro + (0.033)))*(opti->Gyro_X - opti->last_Gyro_X);  
		opti->Gyro_Y = opti->last_Gyro_Y +((0.033)/(FILTER_Gyro + (0.033)))*(opti->Gyro_Y - opti->last_Gyro_Y);
		
		opti->last_Gyro_X = opti->Gyro_X;
		opti->last_Gyro_Y = opti->Gyro_Y;
		
		opti->Vel_correction_X = opti->Vel_X + tan(ToRad(opti->Gyro_X)/fps) * focale_len ;
		opti->Vel_correction_Y = opti->Vel_Y - tan(ToRad(opti->Gyro_Y)/fps) * focale_len;
		
		
		opti->Vel_correction_X = opti->last_Vel_correction_X+((0.033)/(FILTER_Opti + (0.033)))*(opti->Vel_correction_X - opti->last_Vel_correction_X);  
		opti->Vel_correction_Y = opti->last_Vel_correction_Y +((0.033)/(FILTER_Opti + (0.033)))*(opti->Vel_correction_Y - opti->last_Vel_correction_Y);
		
		opti->last_Vel_correction_X = opti->Vel_correction_X;
		opti->last_Vel_correction_Y = opti->Vel_correction_Y;
		
		opti->real_Vel_X = (opti->Vel_correction_X * Ultra.point * fps) / (focale_len * 10);
		opti->real_Vel_Y = (opti->Vel_correction_Y * Ultra.point * fps) / (focale_len * 10);
		
		opti->data_reay =1;
		
		
		opti->count =0;
		opti->Gyro_X_sum = 0;
		opti->Gyro_Y_sum = 0;
		
		Mpc->ready = 0;
	}
	
}


void opti_vel_update(optical_flow_HandleTypeDef *opti,MPU_SENSOR *sen)
{
	
	
	//opti->data_reay =0;
	if(opti->data_reay == 1)
	{
		opti->data_reay =0;
		
		Update_Kalman1x1(&x_vel,opti->real_Vel_X,opti->acc_X,opti_noise,acc_noise);	
		Update_Kalman1x1(&y_vel,opti->real_Vel_Y,opti->acc_Y,opti_noise,acc_noise);	
		
	}
	else
	{
		x_vel.state = x_vel.state + opti->acc_X * 0.004f;
		y_vel.state = y_vel.state + opti->acc_Y * 0.004f;
	}
	
	opti->Pos_X = opti->Pos_X + x_vel.state * 0.004;
	opti->Pos_Y = opti->Pos_Y + y_vel.state * 0.004;
}



void optical_kalman_init()
{
	x_vel.state=0;
	x_vel.C=1;
	x_vel.A=1;
	x_vel.B=DT;
	x_vel.P=1;
	
	y_vel.state=0;
	y_vel.C=1;
	y_vel.A=1;
	y_vel.B=DT;
	y_vel.P=1;
	
	optical.Pos_X = 0;
	optical.Pos_Y = 0;
}








