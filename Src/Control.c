#include "main.h"
#include "Control.h"
#include "define.h"
#include "Kalman.h"
#include "math.h"

System_Status Altitude;
System_Status Altitude_take_off;
_3D_Vector Velocity;
_3D_Vector window_detection;

int counter_flag_for_accept_window;
int counter_for_start_window_follow;

int Quad_On(_RC Rc)
{

			if((float)RC.Throttle < (0.2f*(float)Throttle_range)  &&(float)RC.Yaw > (0.7f*(float)(angle_range/2.0f)))
				return 1;
			else
				return 0;

}



int Quad_Off(_RC Rc)
{
			if((float)RC.Throttle < (0.15f*(float)Throttle_range)  &&  (float)RC.Yaw < (-0.7f*(float)(angle_range/2.0f)  ))
				return 1;
			else
				return 0;
}




int Control(System_Status *In)
{
		System_Status S;
		float	 P,I,D;
		S=*In;

		S.err=S.setpoint-S.point;
		S.integral_err= S.integral_err + S.err*S.Ki*DT ;
		S.integral_err=(fabs(S.integral_err)>(S.Ilimit) )?(S.Ilimit*fsign(S.integral_err)):(S.integral_err);
		
		P=S.Kp*S.err;  
		I=S.integral_err;
		D=S.Kd*(-S.diff_point);					


		if(fabs(P)>S.Plimit)	P=S.Plimit*fsign(P);
		if(fabs(I)>S.Ilimit)	I=S.Ilimit*fsign(I);
		if(fabs(D)>S.Dlimit)	D=S.Dlimit*fsign(D);
		
		S.P_save=P;
		S.I_save=I;
		S.D_save=D;

		if(P+I+D<S.Max  &&   P+I+D>S.Min)
		{
			S.Out=P+I+D;
			S.Out_float=P+I+D;
		}
		else if(P+I+D>=S.Max)
		{
			S.Out=S.Max;
			S.Out_float=S.Max;
		}
		else if(P+I+D<=S.Min)
		{
			S.Out=S.Min;
			S.Out_float=S.Min;
		}

		*In=S;	
		return S.Out;
}

void control_init_(void)
{		
			Altitude.Kp  = 5	; //map 10 ghermez 7
			Altitude.Ki  = 0  ; //map 5 ghermez 0
			Altitude.Kd  = 0  ; //map 0 ghermez 0
						
			Altitude.Max = 300 ; //map 250 ghermez 500
			Altitude.Min = -300 ; //map -450 ghermez -500
			Altitude.Plimit = 300	; //map 250 ghermez 500
			Altitude.Dlimit = 0.0 ; //map 0 ghermez 0.0
			Altitude.Ilimit = 0 ; //map 200 ghermez 0	
			
}



void Set_zero_system_state(void)
{
		Altitude.setpoint = 110;
		Altitude.flag = 1;
		Altitude.Out =0 ;
		Altitude.Out_float =0;
		Altitude.integral_err =0;
		Altitude.I_save =0;
		Altitude.err =0;
}

void Altitude_control(int Alt_Control_THR){

	
	Altitude.Out_bias = 1040; //map 950 ghermez 1040
	
		if(Alt_Control_THR ==1 ){
			
		ultra_filter_lpf(&Ultra);
	  
		Altitude.point  = Ultra.point ; 
			
		Altitude.diff_point = Altitude.point - Altitude.last_point ;
		Altitude.last_point = Altitude.point ;  
		
			
	//FLY_MODE//

				if(Altitude.flag == 1){
					Control(&Altitude);
					RC.RC_channel[6] = Altitude.Out_bias + Altitude.Out;}
				
//		if (RC.RC_channel[0]>1000){Altitude.flag = 2;}
//		else {Altitude.flag = 1;} //jelo giri az noise
		
// //LAND_MODE//
//			if(Altitude.flag == 2){
//			RC.RC_channel[6] = 306;
//			if (Altitude.point < 20){
//				
//				RC.RC_channel[2]= 306;
//			}}
//		
//		
//		}			
//		else {
//			
//			Altitude.Out = 0 ;
//			
//			RC.RC_channel[6]=306;
//			RC.RC_channel[2]=1690;
//			
//			if( Altitude.point < 25 )
//				{
//					RC.RC_channel[2]=306;
//					RC.RC_channel[6]=306;
		}

}

void Velocity_Control(_3D_Vector *Velocity)
{		

	
	if( RC.RC_SW == 1  )
	{
		optical_par.integral_x += x_vel.state * DT ;
		optical_par.integral_y += y_vel.state * DT ;		
		
		Velocity->X.point = -x_vel.state ; // jahat e harekat
	  Velocity->Y.point = -y_vel.state;

  	Velocity->X.setpoint = -2;
	  Velocity->Y.setpoint = -2;		
		
		Velocity->X.diff_point = Velocity->X.point - Velocity->X.last_point;
		Velocity->X.last_point = Velocity->X.point;
		Velocity->Y.diff_point = Velocity->Y.point - Velocity->Y.last_point;
		Velocity->Y.last_point = Velocity->Y.point;
		
		
		
//		Velocity->Y.diff_point = -optical_par.acc_Y;
//  	Velocity->X.diff_point = -optical_par.acc_X;
		
		Velocity->Y.diff_point=Velocity->Y.last_diffpoint+( DT/(DT + FILTER_diffpoint_y_optical))*(Velocity->Y.diff_point - Velocity->Y.last_diffpoint);
	  Velocity->X.diff_point=Velocity->X.last_diffpoint+( DT/(DT + FILTER_diffpoint_x_optical))*(Velocity->X.diff_point - Velocity->X.last_diffpoint);
	  Velocity->Y.last_diffpoint = Velocity->Y.diff_point;
		Velocity->X.last_diffpoint = Velocity->X.diff_point;
		
		//Fuzzy_Gain(OPTICAL_GAIN_SET);
		
		
		Velocity->X.Kp = 4 ;//1.4 ; //zamanike fcut khruji controler 40 bud khub bud hala kardamesh 8 fek konam bas ghavi tar konam//(float)EEPROM_Read_int16_t(EEPROM_opti_x_Kp) / 1000.0f ;
		Velocity->X.Ki = 1.2 ;//1.5;//(float)EEPROM_Read_int16_t(EEPROM_opti_x_Ki) / 1000.0f ;
  	Velocity->X.Kd = 0 ;//(float)EEPROM_Read_int16_t(EEPROM_opti_x_Kd) / 1000.0f ;
    
		
		Velocity->Y.Kp = 4 ;//1.4
   	Velocity->Y.Ki = 1.2 ;//1.6
		Velocity->Y.Kd = 0 ;
		
	
		Velocity->X.Max = 700;//700;
		Velocity->X.Min = -700;//-700;
		Velocity->X.Plimit = 200;
		Velocity->X.Ilimit = 500;//600;
		Velocity->X.Dlimit = 200 ;
		
		Velocity->Y.Max = Velocity->X.Max;
		Velocity->Y.Min = Velocity->X.Min;
		Velocity->Y.Plimit = Velocity->X.Plimit;
		Velocity->Y.Ilimit = Velocity->X.Ilimit;
		Velocity->Y.Dlimit = Velocity->X.Dlimit;	

		
			Control(&Velocity->X);	
			Control(&Velocity->Y);
		
	  	Velocity->Y.Out_float = Velocity->Y.last_Out_float + ( DT / (DT + FILTER_velocity_y_optical )) * ( Velocity->Y.Out_float - Velocity->Y.last_Out_float);
      Velocity->X.Out_float = Velocity->X.last_Out_float + ( DT / (DT + FILTER_velocity_x_optical )) * ( Velocity->X.Out_float - Velocity->X.last_Out_float);	
	  	Velocity->X.last_Out_float = Velocity->X.Out_float;
	  	Velocity->Y.last_Out_float = Velocity->Y.Out_float;
			
	}
	else
	{		
			Velocity->X.Out = 0;
			Velocity->X.Out_float = 0;
			Velocity->X.integral_err = 0;
			
			Velocity->Y.Out = 0;
			Velocity->Y.Out_float = 0;
			Velocity->Y.integral_err = 0;	
		
			optical_par.integral_x = 0;
			optical_par.integral_y = 0;

	}
	
}


void Window_detection(_3D_Vector *window_detection ,_MPC *MPC ) {
	
if( RC.RC_SW ==1 )  
{
	switch(counter_for_start_window_follow){
		case 0:
				window_detection->X.point = MPC->data[2];
			
				window_detection->X.setpoint = 100;
		
				if(fabs(window_detection->X.point - window_detection->X.setpoint)<80)
				{
					window_detection->X.Kp = 1;
					window_detection->X.Ki = 0;
					window_detection->X.Kd = 0;
				
					window_detection->X.Max=200;
					window_detection->X.Min=-200;
					window_detection->X.Plimit=150;
					window_detection->X.Dlimit=150;
					window_detection->X.Ilimit=0;
				}
				else
				{
					window_detection->X.Kp = 1;
					window_detection->X.Ki = 0;
					window_detection->X.Kd = 0;
					
					window_detection->X.Max=200;
					window_detection->X.Min=-200;
					window_detection->X.Plimit=150;
					window_detection->X.Dlimit=150;
					window_detection->X.Ilimit=0;
				}

				window_detection->X.diff_point = (window_detection->X.point - window_detection->X.last_point);
				window_detection->X.last_point = window_detection->X.point  ;
				
				window_detection->X.diff_point = window_detection->X.last_diffpoint + ((DT/(DT+F_CUT_DIFF_window)) * ( window_detection->X.diff_point  -window_detection->X.last_diffpoint));
				window_detection->X.last_diffpoint =    window_detection->X.diff_point;
				
				
				Control(&window_detection->X);
				
				
				if(fabs(window_detection->X.point-window_detection->X.setpoint)<50) {Velocity.X.setpoint=15;}
				else{Velocity.X.setpoint=5;}
				
		//		if(MPC->data[4]>350)	counter_for_start_window_follow=1;
				
				Velocity.Y.Out=0;
				Velocity.Y.integral_err = -200;
				
				
		break;
		//---------------------------------------------------------------------------------------------------------------------
	
		case 1:
			Velocity.Y.setpoint=15;
			Velocity.X.Out=150;
		
			window_detection->X.Out=0;
			window_detection->Y.Out=0;
		
		  counter_flag_for_accept_window++;

			if(counter_flag_for_accept_window>320)		counter_for_start_window_follow=2;
		break;
		//--------------------------------------------------------------------------------------------------------------------
		
		case 2:
			Velocity.Y.setpoint=15;
			Velocity.X.Out=-170;
		
			window_detection->X.Out=0;
			window_detection->Y.Out=0;
		
		  counter_flag_for_accept_window++;

			if(counter_flag_for_accept_window>1000)		counter_for_start_window_follow=3;
		break;
		//--------------------------------------------------------------------------------------------------------------------
		
		case 3:
				window_detection->X.point = MPC->data[2];
			
				window_detection->X.setpoint = 100;
		
				if(fabs(window_detection->X.point - window_detection->X.setpoint)<80)
				{
					window_detection->X.Kp = 1.2;
					window_detection->X.Ki = 0;
					window_detection->X.Kd = 80;
				
					window_detection->X.Max=300;
					window_detection->X.Min=-300;
					window_detection->X.Plimit=150;
					window_detection->X.Dlimit=150;
					window_detection->X.Ilimit=0;
				}
				else
				{
					window_detection->X.Kp = 0.8;
					window_detection->X.Ki = 0;
					window_detection->X.Kd = 80;
					
					window_detection->X.Max=300;
					window_detection->X.Min=-300;
					window_detection->X.Plimit=150;
					window_detection->X.Dlimit=150;
					window_detection->X.Ilimit=0;
				}

				window_detection->X.diff_point = (window_detection->X.point - window_detection->X.last_point);
				window_detection->X.last_point = window_detection->X.point  ;
				
				window_detection->X.diff_point = window_detection->X.last_diffpoint + ((DT/(DT+F_CUT_DIFF_window)) * ( window_detection->X.diff_point  -window_detection->X.last_diffpoint));
				window_detection->X.last_diffpoint =    window_detection->X.diff_point;
				
				
				Control(&window_detection->X);
				
				
		
				window_detection->Y.point = MPC->data[4];
		
				
				
				if(window_detection->Y.point > 200)		window_detection->Y.setpoint = 170;
				else if(window_detection->Y.point > 150)	window_detection->Y.setpoint = 130;
				else	window_detection->Y.setpoint = 120;
				
				
				window_detection->Y.diff_point = (window_detection->Y.point - window_detection->Y.last_point);
				window_detection->Y.last_point = window_detection->Y.point  ;
				
				window_detection->Y.diff_point = window_detection->Y.last_diffpoint + ((DT/(DT+F_CUT_DIFF_window)) * ( window_detection->Y.diff_point  -window_detection->Y.last_diffpoint));
				window_detection->Y.last_diffpoint =    window_detection->Y.diff_point;
				
				window_detection->Y.Kp = 3;
				window_detection->Y.Ki = 3;
				window_detection->Y.Kd = 200;
				
				window_detection->Y.Max=150;
				window_detection->Y.Min=-150;
				window_detection->Y.Plimit=150;
				window_detection->Y.Dlimit=200;
				window_detection->Y.Ilimit=70;
				
					
				Control(&window_detection->Y);
				
				Velocity.Y.Out=0;
				Velocity.X.Out=0;
				
				Altitude.setpoint=70;
				
				if (window_detection->Y.point <100 && counter_flag_for_accept_window<7000)		counter_for_start_window_follow= 4;
				
				//if( Altitude.point < 30 && fabs(window_detection->Y.point - window_detection->Y.setpoint)< 20 && fabs(window_detection->X.point - window_detection->X.setpoint) < 20 )				counter_for_start_window_follow= 4;

break;
		//--------------------------------------------------------------------------------------------------------------------
		
		case 4: 
			Altitude_take_off.flag = 2;
		
			counter_flag_for_accept_window++;

			if(counter_flag_for_accept_window>15000){
				Altitude_take_off.flag = 1;
				counter_for_start_window_follow= 3;
			}
		break;
		//--------------------------------------------------------------------------------------------------------------------

			
}
	}
	
	 else{
		 window_detection->X.Out=0;
		 window_detection->Y.Out=0;
	 }
	
	
}