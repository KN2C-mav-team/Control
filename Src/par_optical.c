#include "main.h"
#include "Altitude.h"
#include "MPC.h"
#include "par_optical.h"
#include "Odometery.h"
float gyro_buff_x[14], gyro_buff_y[14];
optical_par_struct optical_par;
_Kalman1x1 x_vel,y_vel;
int for_kooft=0,for_kooft_num=0;
float  optical_c_x=1.0,optical_c_y=1.0;
double counter_mpc_for_check=0;


void do_optical_par(_MPC *MPC,_MPC *MPC_2,MPU_SENSOR *MPU_sen,optical_par_struct *opti ,_Ultra  *ultra)
{
	
	// baraye teste khoruji  

	
	opti->acc_X = MPU_sen->acc_y - Mahony.pure_acc_y;  // dynamic_acc=acc - static_acc
	opti->acc_X = 100*((opti->acc_X*9.81f)/MPU_sen->Gravity); // scaling
	//	opti->acc_X = -100*((opti->acc_X*9.81f)/2048); // scaling

	
	if(fabs(opti->acc_X) < acc_threshold)
		opti->acc_X = 0;	
	
	opti->acc_Y = MPU_sen->acc_x - Mahony.pure_acc_x;
	opti->acc_Y = 100*((opti->acc_Y*9.81f)/MPU_sen->Gravity);

	if(fabs(opti->acc_Y) < acc_threshold)
		opti->acc_Y = 0;
	
	
	opti->Gyro_X_sum += (MPU_sen->gyro_x);
	opti->Gyro_Y_sum += (MPU_sen->gyro_y);
		


  	if (MPC->ready)
	  {
			
		opti->data_check = 0;
	
		counter_mpc_for_check++;
		
			
		opti->Gyro_X= (opti->Gyro_X_sum); //*DT ro bordam jaye dg
		opti->Gyro_Y= (opti->Gyro_Y_sum);	
			
//			if(fabs(opti->Gyro_X) <0.01)opti->Gyro_X=0;
//			if(fabs(opti->Gyro_Y )<0.01)opti->Gyro_Y=0;
//			
//		opti->Gyro_X= opti->last_buff_Gyro_X; //*DT ro bordam jaye dg
//		opti->Gyro_Y= 	opti->last_buff_Gyro_Y;	
			
	  opti->Gyro_X_sum = 0;
		opti->Gyro_Y_sum = 0;
			
		opti->delta_X = ((float)MPC->data[0])/10;
		opti->delta_Y = ((float)MPC->data[1])/10;
		opti->data_check = (int)MPC->data[2];
		
//		if(opti->data_check !=255){
//					opti->delta_X = 0;
//		opti->delta_Y = 0;} 
			
    opti->Gyro_X = opti->last_Gyro_X +(optical_sample_time/(FILTER_Gyro + optical_sample_time))*(opti->Gyro_X - opti->last_Gyro_X);  
  	opti->Gyro_Y = opti->last_Gyro_Y +(optical_sample_time/(FILTER_Gyro + optical_sample_time))*(opti->Gyro_Y - opti->last_Gyro_Y);
		
		opti->diff_Gyro_X = opti->Gyro_X - (opti->last_Gyro_X) ;
		opti->diff_Gyro_Y = opti->Gyro_Y - opti->last_Gyro_Y ;
		
		opti->last_Gyro_X = opti->Gyro_X;
		opti->last_Gyro_Y = opti->Gyro_Y;
			
    opti->Gyro_X = (opti->Gyro_X*0.5);	
	  opti->Gyro_Y = (opti->Gyro_Y*0.68);
		opti->delta_X_correct = opti->delta_X + (opti->Gyro_X);
		opti->delta_Y_correct = opti->delta_Y - (opti->Gyro_Y);
		
	  opti->real_vel_X = (opti->delta_X_correct * ultra->point ) / focal_pix;  // *fps to change delta x to velcocity
		opti->real_vel_Y = (opti->delta_Y_correct * ultra->point ) / focal_pix;
		
//	 opti->real_vel_X= (opti->delta_X_correct * ALTITUDE_HOVER ) / focal_pix;  // *fps to change delta x to velcocity
//		opti->real_vel_Y= (opti->delta_Y_correct * ALTITUDE_HOVER ) / focal_pix;
//		
		optical_c_x = 1;
		optical_c_y = 1;

		if(fabs(opti->diff_Gyro_X) > 0.5 )     optical_c_x = 1.3  ;
		if(fabs(opti->diff_Gyro_X) > 0.8)      optical_c_x = 1.35 ;		
		if(fabs(opti->diff_Gyro_X) > 1.2 )     optical_c_x = 1.4  ;			
		if(fabs(opti->diff_Gyro_X) > 2.0)		   optical_c_x = 1.55 ;		
		if(fabs(opti->diff_Gyro_X) > 2.8 )     optical_c_x = 1.65  ;
		if(fabs(opti->diff_Gyro_X) > 4.0 )     optical_c_x = 1.8 ;
				
		if(fabs(opti->diff_Gyro_Y) > 0.5 )     optical_c_y= 1.3  ;	
		if(fabs(opti->diff_Gyro_Y) > 0.8 )     optical_c_y= 1.35 ;
		if(fabs(opti->diff_Gyro_Y) > 1.2 )     optical_c_y= 1.4  ;			
		if(fabs(opti->diff_Gyro_Y) > 2.0)      optical_c_y= 1.55 ;	
	  if(fabs(opti->diff_Gyro_Y) > 2.8 )     optical_c_y= 1.65  ;
	  if(fabs(opti->diff_Gyro_Y) > 4.0 )     optical_c_y= 1.8 ;
		
		if(fabs(opti->diff_Gyro_X) > 1 )     opti->real_vel_X = opti->real_vel_X/1.4;
		if(fabs(opti->diff_Gyro_X) > 1.2 )     opti->real_vel_X = opti->real_vel_X/1.6;		
		if(fabs(opti->diff_Gyro_X) > 1.5 )     opti->real_vel_X = opti->real_vel_X/4;			
		if(fabs(opti->diff_Gyro_X) > 2 )		 opti->real_vel_X = opti->real_vel_X/10;		
		if(fabs(opti->diff_Gyro_X) > 3.0 )     opti->real_vel_X = opti->real_vel_X/15;
		if(fabs(opti->diff_Gyro_X) > 4.0 )     opti->real_vel_X = opti->real_vel_X/20;
				
		if(fabs(opti->diff_Gyro_Y) > 1 )     opti->real_vel_Y = opti->real_vel_Y/1.4;	
		if(fabs(opti->diff_Gyro_Y) > 1.2 )     opti->real_vel_Y = opti->real_vel_Y/1.6;	
		if(fabs(opti->diff_Gyro_Y) > 1.5 )     opti->real_vel_Y = opti->real_vel_Y/4;			
		if(fabs(opti->diff_Gyro_Y) > 2 )     opti->real_vel_Y = opti->real_vel_Y/10;	
	  if(fabs(opti->diff_Gyro_Y) > 3.0 )     opti->real_vel_Y = opti->real_vel_Y/15;
	  if(fabs(opti->diff_Gyro_Y) > 4.0 )     opti->real_vel_Y = opti->real_vel_Y/20;

		opti->real_vel_X = opti->last_real_vel_X + (optical_sample_time/(optical_c_x*FILTER_Opti + optical_sample_time))*(opti->real_vel_X - opti->last_real_vel_X);  
		opti->real_vel_Y = opti->last_real_vel_Y + (optical_sample_time/(optical_c_y*FILTER_Opti + optical_sample_time))*(opti->real_vel_Y - opti->last_real_vel_Y);
		opti->last_real_vel_X = opti->real_vel_X;
		opti->last_real_vel_Y = opti->real_vel_Y;

		opti->data_ready = 1;
	
		MPC->ready = 0;
		opti->flag_for_diff = 1;
	}
	
	if(opti->data_ready == 1)
	{
		opti->data_ready=0;
		
		Update_Kalman1x1(&x_vel,opti->real_vel_X,opti->acc_X,opti_noise,acc_noise);	
		Update_Kalman1x1(&y_vel,opti->real_vel_Y,opti->acc_Y,opti_noise,acc_noise);	

	}
	
	else  
	{
		x_vel.state = x_vel.state + opti->acc_X * DT;
		y_vel.state = y_vel.state + opti->acc_Y * DT;
	}

	
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
}