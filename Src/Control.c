#include "main.h"
#include "Control.h"
#include "define.h"
#include "Kalman.h"
#include "math.h"

System_Status Altitude;
System_Status Altitude_Velocity;
System_Status Altitude_take_off;

_3D_Vector Velocity;
_3D_Vector ORB_position;
_3D_Vector Marker_Position;
_3D_Vector window_detection;

_Kalman1x1 vel_marker_x,vel_marker_y,window_x,window_y; 
int flag_for_start_pixracer_altitude;
int counter_for_disable_pixracer_altitude;
int flag_for_pixracer_altitude;
int flag_for_change_altitude_setpoint;
int flag_for_start_optical=0;
int counter_for_landing=0;
int landing_flag;
int hover_flag=0;
int counter_for_sure_go_forward;

int counter_disable_enable=0;

int optical_on_off_flag=0;
int counter_flag_for_accept_window=0;  ///mikham har bar k tik zad in ++ beshe bad age 4 shod migam k az panjere umadam birun 
int land_marker=0;
int go_forward=0;
int move_on_line_flag=0;
int counter_for_enter=0;
int counter_stop_velocity=0;
float shalgham ;
int conter_stop_controller_altitude=0;
int flag_stop_controller_altitude=0;
 
 int change_coe_flag_for_window=0;
 int flag_for_back=0;
 int start_take_off_optical;
float eslahe_data_marker;
int Alt_Control_run=FALSE;
int Alt_Setpoint_state=0;
int Throttle_bias=0,Motor_force=0;
float On_Ground_Altitude=0;
float RotationX,RotationY;
float pitch[50],roll[50];
float pitch_correct,roll_correct;
int count;
int marker_count_buff=0;
int window_count_buff=0;
int position_error=0;

void Rc2Controller(_RC Rc)
{	
}



int Quad_On(_RC Rc)
{
//	#ifndef Quad
			if((float)RC.Throttle < (0.2f*(float)Throttle_range)  &&(float)RC.Yaw > (0.7f*(float)(angle_range/2.0f)))
				return 1;
			else
				return 0;
//	#else
//			
//			if((float)RC.Throttle < (0.1f*(float)Throttle_range)  &&  (float)RC.Pitch<(0.7f*(float)angle_range + Pitch_offset) && (float)RC.Roll<(0.7f*(float)angle_range + Roll_offset) &&(float)RC.Yaw <  (0.7f*(float)(angle_range/2.0f)))
//				return 1;
//			else
//				return 0;
//			
//	#endif
//			return 0;
}



int Quad_Off(_RC Rc)
{
//	#ifndef Quad
			if((float)RC.Throttle < (0.15f*(float)Throttle_range)  &&  (float)RC.Yaw < (-0.7f*(float)(angle_range/2.0f)  ))
				return 1;
			else
				return 0;
//	#else
//			
//			if((float)RC.Throttle < (0.1f*(float)Throttle_range)  &&  (float)RC.Pitch<(0.7f*(float)angle_range + Pitch_offset) && (float)RC.Roll<(-0.7f*(float)angle_range + Roll_offset) &&(float)RC.Yaw < (-0.7f*(float)(angle_range/2.0f)))
//				return 1;
//			else
//				return 0;
//			
//	#endif
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
	
			  Altitude.setpoint = ALTITUDE_TAKE_OFF_MAX ;
		
		start_take_off_optical=0;
	
	  Altitude_Velocity.Kp=1.5;
		Altitude_Velocity.Ki=7;
		Altitude_Velocity.Kd=0.4;
		
    Altitude.Kp  = 0.5 ;
	  Altitude.Ki  = 0.1 ;
		Altitude.Kd  = 0 ;
				
		Velocity.X.Kp_save=3.5;//14 ; //zamanike fcut khruji controler 40 bud khub bud hala kardamesh 8 fek konam bas ghavi tar konam//(float)EEPROM_Read_int16_t(EEPROM_opti_x_Kp) / 1000.0f ;
		Velocity.X.Ki_save=6;//6;//(float)EEPROM_Read_int16_t(EEPROM_opti_x_Ki) / 1000.0f ;
  	Velocity.X.Kd_save=0;//(float)EEPROM_Read_int16_t(EEPROM_opti_x_Kd) / 1000.0f ;
    Velocity.X.Ki=6;//5;
		
		Velocity.Y.Kp_save=5;//14;//(float)EEPROM_Read_int16_t(EEPROM_opti_y_Kp) / 1000.0f ;
   	Velocity.Y.Ki_save =0;//6;//(float)EEPROM_Read_int16_t(EEPROM_opti_y_Ki) / 1000.0f ;
		Velocity.Y.Kd_save=0;//(float)EEPROM_Read_int16_t(EEPROM_opti_y_Kd) / 1000.0f ;
		Velocity.Y.Ki=0;//6;
			
		ORB_position.X.Kp_save=0.08;// baraye halatike khoruji zavie bashad1.15;//(float)EEPROM_Read_int16_t(EEPROM_Position_Kp) / 1000.0f ;
   	ORB_position.X.Ki_save=0.0002;//0.35;//(float)EEPROM_Read_int16_t(EEPROM_Position_Ki) / 1000.0f ;
    ORB_position.X.Kd_save=0;//(float)EEPROM_Read_int16_t(EEPROM_Position_Kd) / 1000.0f ;
		
  	ORB_position.Y.Kp_save=ORB_position.X.Kp_save;
		ORB_position.Y.Ki_save=ORB_position.X.Ki_save;
		ORB_position.Y.Kd_save=ORB_position.X.Kd_save;
		
		ORB_position.X.Max=40;// baraye halatike khoruji zavie bashad100;
		ORB_position.X.Min=-40; //-100;
		ORB_position.X.Plimit=20;//60;
		ORB_position.X.Dlimit=0;
		ORB_position.X.Ilimit=0;//60;	
		
	  ORB_position.Y.Max   = ORB_position.X.Max;
		ORB_position.Y.Min   = ORB_position.X.Min;
		ORB_position.Y.Plimit= ORB_position.X.Plimit;
		ORB_position.Y.Dlimit= ORB_position.X.Dlimit;
		ORB_position.Y.Ilimit= ORB_position.X.Ilimit;
		
		
		Velocity.X.Max = 750;//700;
		Velocity.X.Min = -750;//-700;
		Velocity.X.Plimit = 450;
		Velocity.X.Ilimit = 600;
		Velocity.X.Dlimit = 50;
		
		Velocity.Y.Max =750;
		Velocity.Y.Min = -750;;
		Velocity.Y.Plimit = Velocity.X.Plimit;
		Velocity.Y.Ilimit =80;;
		Velocity.Y.Dlimit = Velocity.X.Dlimit;	
		
		
		Altitude.Max=150;
		Altitude.Min=-150;
		Altitude.Plimit=150;
		Altitude.Dlimit=0;
		Altitude.Ilimit=0;
		
		
		Marker_Position.X.Max=85;//2.5;
		Marker_Position.X.Min=-85;//-2.5;
		Marker_Position.X.Plimit=60;//0.5;
		Marker_Position.X.Dlimit=60;//0.5;
		Marker_Position.X.Ilimit=25;//0.8;	
		
	  Marker_Position.Y.Max   = Marker_Position.X.Max;
		Marker_Position.Y.Min   = Marker_Position.X.Min;
		Marker_Position.Y.Plimit= Marker_Position.X.Plimit;
		Marker_Position.Y.Dlimit= Marker_Position.X.Dlimit;
		Marker_Position.Y.Ilimit= Marker_Position.X.Ilimit;
		
		window_detection.X.Kp =0.07;
		window_detection.X.Ki=0;
		window_detection.X.Kd=0;
		
	  window_detection.Y.Kp =0.07;
		window_detection.Y.Ki=0;
		window_detection.Y.Kd=0;
		
		
		window_detection.X.Max=100;//25;
		window_detection.X.Min=-window_detection.X.Max;
		window_detection.X.Dlimit=100;//60;//25;//30;
		window_detection.X.Ilimit=20;//8;
		window_detection.X.Plimit=100;//60;//25;//30;
		
		window_detection.Y.Max   = window_detection.X.Max;
		window_detection.Y.Min   = window_detection.X.Min;
		window_detection.Y.Dlimit=window_detection.X.Dlimit;
		window_detection.Y.Ilimit=window_detection.X.Ilimit;
		window_detection.Y.Plimit=window_detection.X.Plimit;
		
	
}



void Set_zero_system_state(void)
{
	flag_for_start_pixracer_altitude=0;
	counter_for_disable_pixracer_altitude=0;
	flag_for_pixracer_altitude=0;
	flag_for_change_altitude_setpoint=0;
	flag_for_start_optical=0;
	counter_for_landing=0;
  landing_flag=0;
	hover_flag=0;
	counter_for_sure_go_forward=0;
	counter_disable_enable=0;
	optical_on_off_flag=0;
	land_marker=0;
	counter_flag_for_accept_window=0;
	go_forward=0;
	move_on_line_flag=0;
	change_coe_flag_for_window=0;
	flag_for_back=0;
	counter_stop_velocity=0;
		  optical_par.integral_x = 0;
	    optical_par.integral_y = 0;

		  ORB_position.X.Ki=0;
	    ORB_position.Y.Ki=0;
	
		  ORB_position.X.integral_err=0;
			ORB_position.Y.integral_err=0;
	
	    ORB_position.X.I_save=0;
	    ORB_position.Y.I_save=0;
	
			ORB_position.X.Out=0;
      ORB_position.X.Out_float=0;	
	
    	ORB_position.Y.Out=0;
      ORB_position.Y.Out_float=0;	
						
			Altitude_Velocity.Out=0;
      Altitude_Velocity.integral_err=0;
      Altitude_Velocity.I_save=0;		
       Altitude_Velocity.Out_float=0;			
			Altitude_Velocity.Out_bias=0;
			
			Alt_Setpoint_state=0;				
			Throttle_bias=0;
			Motor_force=0;
	
			position_error=0;
			
			On_Ground_Altitude=Ultra.point;

      Velocity.X.Out=0;
      Velocity.X.Out_float=0;
			Velocity.X.I_save=0;
			Velocity.X.integral_err=0;
		  Velocity.X.err=0;
			
			Velocity.Y.Out=0;
      Velocity.Y.Out_float=0;
			Velocity.Y.I_save=0;
			Velocity.Y.integral_err=0;
			Velocity.Y.err=0;
						
			
			Altitude.Out =0 ;
			Altitude.Out_float =0;
			Altitude.integral_err =0;
			Altitude.I_save =0;
			Altitude.err =0;
			
			Marker_Position.X.Out=0;
			Marker_Position.X.Out_float=0;
			Marker_Position.X.integral_err=0;
			Marker_Position.X.I_save=0;
			
			Marker_Position.Y.Out=0;
			Marker_Position.Y.Out_float=0;
			Marker_Position.Y.integral_err=0;
			Marker_Position.Y.I_save=0;
			
			window_detection.X.Out=0;
			window_detection.X.Out_float=0;
			window_detection.X.integral_err=0;
			window_detection.X.I_save=0;
			
	
			window_detection.Y.Out=0;
			window_detection.Y.Out_float=0;
			window_detection.Y.integral_err=0;
			window_detection.Y.I_save=0;	
}


void Point2Controller(_IMU IMU,MPU_SENSOR Mpu)
{	
	
}





//void altitude_control(int Alt_Control_sw , int Alt_Control_hov_pit)
//	{

//	
//	  if(landing_flag==1) {
//			
//				       RC.RC_channel[5]=1000 ;//- Velocity.Y.Out_float;//roll
//							 RC.RC_channel[7]=1000 ;//+ Velocity.X.Out_float; //pitch
//							 RC.RC_channel[6]=309 ; //throtle
//							 RC.RC_channel[4]=1000; //yaw
//							 RC.RC_channel[1]=306;
//							 RC.RC_channel[2]=306; //sw
//							 RC.RC_channel[3]=306;
//							 RC.RC_channel[0]=306;  //thr
//		}


//	
//	}

	void altitude_control(int Alt_Control_THR)
	{
		Altitude.Out_bias = 1000;
		if(Alt_Control_THR ==1 ){
	  Altitude.setpoint = ALTITUDE_TAKE_OFF_MAX;
			
		if( flag_for_start_optical==1 )Altitude.setpoint =  50 ; 
	  Altitude.point  = Ultra.point ; 
	  Control(&Altitude);
//		if(fabs(Ultra.vel) > 250 && flag_for_start_optical==0 ) counter_for_disable_pixracer_altitude=500;
//	  counter_for_disable_pixracer_altitude--;
//	 if(counter_for_disable_pixracer_altitude<0) counter_for_disable_pixracer_altitude=0;
//			
//    if(counter_for_disable_pixracer_altitude>0 ) Altitude.Out=0;
	


      if ( counter_flag_for_accept_window==1 && hover_flag==1 ){
				flag_stop_controller_altitude=1 ;
				 }
			if ( counter_flag_for_accept_window==2 ){
				conter_stop_controller_altitude++ ; }
				if ( counter_flag_for_accept_window==2 && conter_stop_controller_altitude>100 ) { 
					flag_stop_controller_altitude=0 ;
					conter_stop_controller_altitude=0;}
      if ( counter_flag_for_accept_window==3 ){flag_stop_controller_altitude=1 ;
				conter_stop_controller_altitude=0;}
			if ( counter_flag_for_accept_window==4  ){
			conter_stop_controller_altitude++ ;}
			if ( counter_flag_for_accept_window==4 && conter_stop_controller_altitude>100 )  flag_stop_controller_altitude=0 ;													
				
			
		if (flag_stop_controller_altitude==1) Altitude.Out=0;
			
	  RC.RC_channel[6] =Altitude.Out + Altitude.Out_bias;
}
		else Altitude.Out =0;

	
	}
	
	

void Control_Altitude_Velocity(int Alt_Control_SW)  
{	
	Altitude_Velocity.point = z_vel.state;
	Altitude_Velocity.point = Altitude_Velocity.last_point + (( DT /(DT + FILTER_lpf_altitude_vel ))*(Altitude_Velocity.point - Altitude_Velocity.last_point )) ; 
	Altitude_Velocity.last_point = Altitude_Velocity.point;
	Altitude_Velocity.diff_point = (Mahony.Earth_acc_z)*1.0f;
	
	if( Alt_Control_SW == TRUE  && RC.THR_CUT==1	)
	{
		

//		if	(fabs(Ultra.point-ALTITUDE_TAKE_OFF_MAX)<10 ) flag_for_start_pixracer_altitude=1;
//		if(flag_for_start_pixracer_altitude==1) { 
//			Altitude_Velocity.Out_float = Altitude_Velocity.Out_float - 1.0;
//			if(Altitude_Velocity.Out_float<1010 ) 				flag_for_pixracer_altitude=1; 
//		}
//   		if(  flag_for_pixracer_altitude==0 && flag_for_start_pixracer_altitude==0)
//			{			
//			   	Altitude_Velocity.Out_float=1300.0f;		
//			}

		//   	 RC.RC_channel[6] = (int) (Altitude_Velocity.Out_float) ;
//
//				if(  flag_for_pixracer_altitude==1 )	{
					Altitude.Kp  = 4;//5 ;
					if	(fabs(Ultra.point-ALTITUDE_TAKE_OFF_MAX)<10 )  Altitude.Kp  = 1 ; 
					altitude_control(RC.THR_CUT);
			//		if(Ultra.point > 200 )  RC.RC_channel[6] =950;
					if(Ultra.point < 30  && flag_for_start_optical==1 )  RC.RC_channel[6] =700;
					if(Ultra.point < 10  && flag_for_start_optical==1 ) {
								
				       RC.RC_channel[5]=1000 ;//- Velocity.Y.Out_float;//roll
							 RC.RC_channel[7]=1000 ;//+ Velocity.X.Out_float; //pitch
							 RC.RC_channel[6]=309 ; //throtle
							 RC.RC_channel[4]=1000; //yaw
							 RC.RC_channel[1]=306;
							 RC.RC_channel[2]=306; //sw
							 RC.RC_channel[3]=306;
							 RC.RC_channel[0]=306;  //thr
					}
				}
//	}	
}

void Fuzzy_Gain(char controler)
{	
	
	if(controler==OPTICAL_GAIN_SET){
  if(	Ultra.point <250 ) { 
		Velocity.Y.Kp=Velocity.Y.Kp_save/2;
	  Velocity.X.Kp=Velocity.X.Kp_save/2 ;
		Velocity.Y.Kd=Velocity.Y.Kd_save/3 ;
	  Velocity.X.Kd=Velocity.X.Kd_save/3 ;}
	  if(	Ultra.point <150 ) { 
		Velocity.Y.Kp=Velocity.Y.Kp_save/1.5 ;
	  Velocity.X.Kp=Velocity.X.Kp_save/1.5 ;
		Velocity.Y.Kd=Velocity.Y.Kd_save/1.5 ;
	  Velocity.X.Kd=Velocity.X.Kd_save/1.5 ;}
		  if(	Ultra.point <50 ) { 
		Velocity.Y.Kp=Velocity.Y.Kp_save ;
	  Velocity.X.Kp=Velocity.X.Kp_save ;
		Velocity.Y.Kd=Velocity.Y.Kd_save ;
	  Velocity.X.Kd=Velocity.X.Kd_save ;}
	}
	
	
		
  if(controler==ORB_GAIN_SET){
	//	MPC_2.data[2]<
	 	 ORB_position.X.Kp=ORB_position.X.Kp_save; 
		 ORB_position.X.Ki= ORB_position.X.Ki_save;
		 ORB_position.X.Kd=ORB_position.X.Kd_save;

		 ORB_position.Y.Kp=ORB_position.Y.Kp_save;
		 ORB_position.Y.Ki=ORB_position.Y.Ki_save;
		 ORB_position.Y.Kd=ORB_position.Y.Kd_save;

 }
		 
		 
	if(controler==Altitude_Velocity_GAIN_SET)
	{
		Altitude_Velocity.Max=990;
		Altitude_Velocity.Min=300;
		Altitude_Velocity.Plimit=40;
		Altitude_Velocity.Dlimit=25;
	  Altitude_Velocity.Ilimit=950;

		Altitude_Velocity.Ki = Altitude_Velocity.Ki_save/6;		
		
		if(Ultra.point<8 )  // On The Groaund
		{
		Altitude_Velocity.Ilimit=930;
		Altitude_Velocity.Dlimit=0;	
		Altitude_Velocity.Ki=15.0f*Altitude_Velocity.Ki_save;		
		}
			if	(fabs(Ultra.point-ALTITUDE_TAKE_OFF_MAX)<20 )  // On The Groaund
		{
		Altitude_Velocity.Kp=Altitude_Velocity.Kp_save/2;		
		Altitude_Velocity.Ki=Altitude_Velocity.Ki_save/6;		
		Altitude_Velocity.Kd=Altitude_Velocity.Kd_save;	
		Altitude_Velocity.Max=1050;
//		Altitude_Velocity.Min=300;
		Altitude_Velocity.Plimit=10;
		Altitude_Velocity.Dlimit=20;
//		Altitude_Velocity.Ilimit=985;
	
		}		
	}
}
 



void Velocity_Control(_3D_Vector *Velocity)  //version3
{		

	if( RC.RC_SW==1 && flag_for_start_optical==1 )
	{
	
		Velocity->X.setpoint=0;		
	  Velocity->X.point = -x_vel.state ;

		Velocity->X.hpf=(Filter_hpf_optical/(Filter_hpf_optical + DT )) * (Velocity->X.point - Velocity->X.last_point +Velocity->X.last_hpf );
   	Velocity->X.last_hpf = 	Velocity->X.hpf;
		Velocity->X.last_point = Velocity->X.point;
		

		Velocity->X.point = Velocity->X.hpf;
		
		Velocity->X.Kp_save=3.5;
  	Velocity->X.Kd_save=0;
    Velocity->X.Ki=6;
		
		
		Fuzzy_Gain(OPTICAL_GAIN_SET);

				
			Control(&Velocity->X);	
		
      Velocity->X.Out_float = Velocity->X.last_Out_float + ( DT / (DT + FILTER_velocity_x_optical )) * ( Velocity->X.Out_float - Velocity->X.last_Out_float);
				
	  	Velocity->X.last_Out_float = Velocity->X.Out_float;

	
		  if(MPC.data[2]!=255) {
			Velocity->X.Out_float =0;
			}
					if(flag_for_back==2 && MPC.data[10]==255) 			Velocity->X.Out_float =0;
			
			
	}
	else
	{		
			Velocity->X.Out = 0;
			Velocity->X.Out_float = 0;
			Velocity->X.integral_err = 0;

	}

}

void Third_Person_control(int _Roll_setpoint,int _Pitch_setpoint ,int _Yaw_point,int corrected_Roll_setpoint,int corrected_Pitch_setpoint)
{
	float _R,_P,_Y;
	
	_R=	_Roll_setpoint*cos(ToRad(_Yaw_point)	)	  +	_Pitch_setpoint * sin(ToRad(_Yaw_point));
	_P=	_Roll_setpoint*(-sin(ToRad(_Yaw_point)))	+	_Pitch_setpoint * cos(ToRad(_Yaw_point));
	
	corrected_Roll_setpoint =(int)_R;
	corrected_Pitch_setpoint=(int)_P;	
}

void First_Person_control()
	
{
		float _R,_P,_Y;
	
//	_R=	_Roll->setpoint*cos((ToRad((Sytem_Yaw_Offset ))))		+	_Pitch->setpoint*sin((ToRad((Sytem_Yaw_Offset ))));
//	_P=	_Roll->setpoint*(-sin((ToRad((Sytem_Yaw_Offset )))))	+	_Pitch->setpoint*cos((ToRad((Sytem_Yaw_Offset))));
//	_Y=	_Yaw->setpoint;	
//	
//	_Roll->setpoint=_R;
//	_Pitch->setpoint=_P;	
//	_Yaw->setpoint=_Y;
}


void Window_detection(_3D_Vector *window_detection ,_MPC *MPC ) {
	
if( RC.RC_SW ==1 )  
	 {

		window_detection->X.Kp =2;//2.5;//1.3;
		window_detection->X.Ki=0;//0.1;
		window_detection->X.Kd=3.5;//1.3;
		

		if (MPC->data[5]==255){
														window_count_buff++;			 
														window_detection->X.point = MPC->data[3];
														window_detection->Y.point = MPC->data[6];
		                        }
		 
		if(window_count_buff%7==0){
													 window_detection->X.diff_point = (window_detection->X.point - window_detection->X.last_point);
													 window_detection->Y.diff_point = (window_detection->Y.point - window_detection->Y.last_point);
													 window_detection->X.last_point = window_detection->X.point  ;
													 window_detection->Y.last_point = window_detection->Y.point  ;
														 
													 window_detection->X.diff_point = window_detection->X.last_diffpoint + ((7*DT/(7*DT+F_CUT_DIFF_window)) * ( window_detection->X.diff_point  -window_detection->X.last_diffpoint));
													 window_detection->Y.diff_point = window_detection->Y.last_diffpoint + ((7*DT/(7*DT+F_CUT_DIFF_window)) * ( window_detection->Y.diff_point  -window_detection->Y.last_diffpoint));

													 window_detection->X.last_diffpoint =    window_detection->X.diff_point;
													 window_detection->Y.last_diffpoint =    window_detection->Y.diff_point;
		 }
		
		 window_detection->X.acc =  10*(((Mpu.acc_x - Mahony.pure_acc_x)*9.81f)/Mpu.Gravity); // alamat ha check beshan 
	   window_detection->Y.acc =  10*(((Mpu.acc_y - Mahony.pure_acc_y)*9.81f)/Mpu.Gravity); // 
	  
		 Update_Kalman1x1(&window_x,window_detection->X.diff_point, window_detection->X.acc , 7000 ,3);
		 Update_Kalman1x1(&window_y,window_detection->Y.diff_point, window_detection->Y.acc , 7000 ,3);
		 
//		window_detection->X.diff_point =  window_x.state;
//		window_detection->Y.diff_point =  window_y.state;
		 
			window_detection->X.diff_point =  -y_vel.state;
   		window_detection->Y.diff_point =  -x_vel.state;  //feedback + / - ?

	 
	  if ((Ultra.vel)<-225 && counter_flag_for_accept_window==0 )  { 
                                                               			change_coe_flag_for_window=1;
	                                                                	counter_flag_for_accept_window=1;
		                                                             }
		if ((Ultra.vel)>225  && counter_flag_for_accept_window==1 )  { 
                                                               			change_coe_flag_for_window=1;
	                                                                	counter_flag_for_accept_window=2;
		                                                             }
		if ((Ultra.vel)<-225 && counter_flag_for_accept_window==2 )  {
                                                               			change_coe_flag_for_window=1;
	                                                                	counter_flag_for_accept_window=3;
		                                                             }		
		if ((Ultra.vel)> 225 && counter_flag_for_accept_window==3 )  { 
                                                               			change_coe_flag_for_window=1;
		                                                                counter_flag_for_accept_window=4;
		                                                             }


  	window_detection->X.setpoint=0;
																																 
	
				if(flag_for_back==2) {

				window_detection->Y.setpoint= MPC->data[6];	
				window_detection->Y.point= 95;  // chon nmikhastam khruji dar manfi zarb beshe jaye point va setpoint ro avaz kardam
				
				if(fabs(	window_detection->Y.point - 	window_detection->Y.setpoint) <20 ) flag_for_start_optical=1;
			  window_detection->Y.Kp =10;//10;
		    window_detection->Y.Ki=0.5;//0.1;
		    window_detection->Y.Kd=4;//4;
				
			  window_detection->Y.Max   = 80;
    		window_detection->Y.Min   = -80;
    		window_detection->Y.Dlimit=60;
	    	window_detection->Y.Ilimit=20;
	    	window_detection->Y.Plimit=60;
		 
				
				
				}

	
	  Control(&window_detection->X);
	  Control(&window_detection->Y);	

// inke ye ja vayse be modate tulani bad bere jelo ta navasan nayofte////////////////////////////////////// 		
	
    if(flag_for_back==0){
			if(fabs(window_detection->X.setpoint -  	window_detection->X.point)<50 ) counter_for_sure_go_forward++;
	//	else counter_for_sure_go_forward=0;
	//	else {counter_for_sure_go_forward-- ;
	//		if(counter_for_sure_go_forward<0) counter_for_sure_go_forward=0;}
			
	//		inke faghat avalin bar y ja sabet bemune va bad shru kone b jolo umadan behtar az inke hamishe sharte hashie ro bezari 
			
		
		if(counter_for_sure_go_forward>300){
			counter_for_sure_go_forward=301;
		 window_detection->Y.Out_float=-45;}
		else 		 window_detection->Y.Out_float=0;
	 

// joloye panjere ba sorate bishtari bere tu /////////////////////////////////////////////////
		if(MPC->data[6]>230 && counter_for_sure_go_forward>300 ) go_forward=1;  //adade scale ro define kon
		if(go_forward==1){
		window_detection->Y.Out_float=-90;
		window_detection->X.Out_float=0;   ///inke khodesh bere va panjere uno monharef nakone
		}
	
	  if(change_coe_flag_for_window==1 ){	
		window_detection->X.Out_float=0;
	  move_on_line_flag=1;
		optical_par.integral_x =0 ;
		flag_for_back=1;     }}
		
//raftan b dakhele panjre////////////////////////////////////////////////////////////////////
			if(flag_for_back==1) {
			go_forward=0;
	    window_detection->Y.Out_float=150;				                 
		  optical_par.integral_x += x_vel.state * DT ;
			
//birun umadan az panjereh//////////////////////////////////////////////////////////////
			if( counter_flag_for_accept_window>2 )	{	
				    window_detection->Y.Out_float=100;	
			
			if(  optical_par.integral_x < -370 )  {
			  window_detection->Y.Out_float=0;
				flag_for_back=2;}
				

			}
			else {			window_detection->X.Out_float=0;}
		}
//vaysadan dar noghteye landing ////////////////////////////////////////////////////////

/////////////////////////////////////////////////////////////////////////////////////////
				//ghate barnameye panjere va run shodane marker
//				if(flag_for_back==2 && MPC->data[10]==255) {
//					 window_detection->Y.Out_float=0;
//					 window_detection->X.Out_float=0;
//				}
				


//		if(( go_forward ==1  )|| (flag_for_back==1  && counter_flag_for_accept_window<3) || (optical_on_off_flag==1 ))	window_detection->X.Out_float=0;
//	if(fabs(window_detection->X.setpoint -  	window_detection->X.point)<20 )	  window_detection->X.Out_float=0;
				
	 }
	 else{
		 window_detection->X.Out_float=0;
		 window_detection->Y.Out_float=0;
	 }
	
	
}
	
void ORB_Position_control(_3D_Vector *ORB_position,_MPC *MPC)
	{
   Fuzzy_Gain(ORB_GAIN_SET);
	 if( RC.RC_SW ==1 )   /// chek shavad thr_cut
	 {

//			ORB_position->X.diff_point = -optical_par.real_vel_X;
//   		ORB_position->Y.diff_point = +optical_par.real_vel_Y;	
//			ORB_position->X.diff_point = ORB_position->X.last_diffpoint + ((DT/(DT+F_CUT_DIFF)) * ( ORB_position->X.diff_point  -ORB_position->X.last_diffpoint));
//			ORB_position->Y.diff_point = ORB_position->Y.last_diffpoint + ((DT/(DT+F_CUT_DIFF)) * ( ORB_position->Y.diff_point  -ORB_position->Y.last_diffpoint));

//			 ORB_position->X.last_diffpoint =  ORB_position->X.diff_point;
//			 ORB_position->Y.last_diffpoint =  ORB_position->Y.diff_point;
			 
			Control(&ORB_position->X);	
		//	Control(&ORB_position->Y);
		
	  	//ORB_position->Y.Out_float = ORB_position->Y.last_Out_float + ( DT / (DT + FILTER_y_ORB )) * ( ORB_position->Y.Out_float - ORB_position->Y.last_Out_float);
      ORB_position->X.Out_float = ORB_position->X.last_Out_float + ( DT / (DT + FILTER_x_ORB )) * ( ORB_position->X.Out_float - ORB_position->X.last_Out_float);

      //Velocity.X.setpoint = ORB_position->X.Out_float;   //check shavad
		//	Velocity.Y.setpoint =	-ORB_position->Y.Out_float;
			 
//        Velocity.X.setpoint = 0;
//        Velocity.Y.setpoint =	0;
		
//	  	ORB_position->X.last_Out_float = ORB_position->X.Out_float;
//	  	ORB_position->Y.last_Out_float = ORB_position->Y.Out_float;
		
	 }
  	else
	 {	
		  ORB_position->X.Out_float =0;
		//	ORB_position->Y.Out_float = 0;
		 } 
	
}
	

void Marker_Position_Control(_3D_Vector *Marker_Position,_MPC *MPC)
	{
		
	 if( RC.RC_SW ==1 )   /// chek 
	 {
			uint8_t Marker_data_ready = MPC->data[10];
		 
				if(MPC->data[10]==255) counter_disable_enable ++ ;
			if(counter_disable_enable > 100 ){ optical_on_off_flag=1;
			}
	 if(MPC->data[10]!=255) counter_disable_enable=0;
		if(counter_disable_enable <1 ) optical_on_off_flag=0;

			Marker_Position->X.setpoint = 0;
			Marker_Position->Y.setpoint = 0;
		 
		 pitch[count] = Mahony.Pitch;
		 roll[count] = Mahony.Roll;
		 count++;
		 if (count>49) count =0;
		 int buff = count - 25;
		 if(buff<0) buff+=50;
		 pitch_correct = pitch[buff];
		 roll_correct = roll[buff];

		 
		 if(Marker_data_ready == 255)
		 {
			 marker_count_buff++;			 
			 Marker_Position->X.point_real =(float)2*Ultra.point*Field_of_view_tan_X*MPC->data[8]/640 ;
			 eslahe_data_marker=(Ultra.point*tan(ToRad(pitch_correct)));
			 Marker_Position->X.point      = (float)2*Ultra.point*Field_of_view_tan_X*MPC->data[8]/640 ;//- 1*(Ultra.point*tan(ToRad(pitch_correct)));
	     Marker_Position->Y.point_real = (float)2*Ultra.point*Field_of_view_tan_Y*MPC->data[9]/480;			
			 Marker_Position->Y.point      = (float)2*Ultra.point*Field_of_view_tan_Y*MPC->data[9]/480 ;//+ 1*Ultra.point*tan(ToRad(roll_correct));
		 }
		 
		 if(marker_count_buff%4==0){
			Marker_Position->X.diff_point = (Marker_Position->X.point  - Marker_Position->X.last_point)*10;
		 	Marker_Position->Y.diff_point = (Marker_Position->Y.point  - Marker_Position->Y.last_point)*10 ;
			Marker_Position->X.last_point = Marker_Position->X.point ;
	 		Marker_Position->Y.last_point = Marker_Position->Y.point ;
		 
		  Marker_Position->X.diff_point = Marker_Position->X.last_diffpoint + ((4*DT/(4*DT+F_CUT_DIFF)) * ( Marker_Position->X.diff_point  -Marker_Position->X.last_diffpoint));
			Marker_Position->Y.diff_point = Marker_Position->Y.last_diffpoint + ((4*DT/(4*DT+F_CUT_DIFF)) * ( Marker_Position->Y.diff_point  -Marker_Position->Y.last_diffpoint));

		 Marker_Position->X.last_diffpoint =  Marker_Position->X.diff_point;
		 Marker_Position->Y.last_diffpoint =  Marker_Position->Y.diff_point;}
		 
		 
	    Marker_Position->Y.acc =  10*(((Mpu.acc_x - Mahony.pure_acc_x)*9.81f)/Mpu.Gravity); // alamat ha check beshan 
	    Marker_Position->X.acc =  10*(((Mpu.acc_y - Mahony.pure_acc_y)*9.81f)/Mpu.Gravity); // 	 
		 
		  Update_Kalman1x1(&vel_marker_x,Marker_Position->X.diff_point, Marker_Position->X.acc , 1000 ,3);	
		  Update_Kalman1x1(&vel_marker_y,Marker_Position->Y.diff_point, Marker_Position->Y.acc , 1000 ,3);	
		 //	Marker_Position->X.diff_point = 		vel_marker_x.state;
		 //	Marker_Position->Y.diff_point = 		vel_marker_y.state;
  //  Marker_Position->X.point = Marker_Position->X.last_point + ((DT/(DT+F_CUT_DIFF)) * ( Marker_Position->X.point  - Marker_Position->X.last_point));
  //	Marker_Position->Y.point = Marker_Position->Y.last_point + ((DT/(DT+F_CUT_DIFF)) * ( Marker_Position->Y.point  - Marker_Position->Y.last_point));
				
//      if(fabs(Marker_Position->X.point ) <25) {
		Marker_Position->X.Kp=1;//0.02;
   	Marker_Position->X.Ki=0.1;
    Marker_Position->X.Kd=2.85;//0.1;
//				}
//			else {
//		Marker_Position->X.Kp=0.01;//0.02;
//   	Marker_Position->X.Ki=0.005;
//    Marker_Position->X.Kd=0.01;//0.1;
//				
//			}
//				
//      if(fabs(Marker_Position->Y.point ) < 25) {		  	
		Marker_Position->Y.Kp =Marker_Position->X.Kp;//0.03;//0.02;// 0.011;
		Marker_Position->Y.Ki =Marker_Position->X.Ki;//0.005;//0.001;// 0.001;//0.001;
		Marker_Position->Y.Kd =Marker_Position->X.Kd;//0.8 ;//0.1;//0.07;
//			}				
//			else {
//	  Marker_Position->Y.Kp =0.01;//0.02;// 0.011;
//		Marker_Position->Y.Ki = 0.005;//0.001;
//		Marker_Position->Y.Kd =0.01;//0.1;//0.07;
//			}
		  Control(&Marker_Position->X);	
			Control(&Marker_Position->Y);
			
			if(Marker_data_ready != 255  || flag_for_back!=2  )
		 {
			 	Marker_Position->Y.Out_float = 0;
			 Marker_Position->X.Out_float =0;
		 }
		
	  	Marker_Position->Y.Out_float = Marker_Position->Y.last_Out_float + ( DT / (DT + FILTER_y_ORB )) * ( Marker_Position->Y.Out_float - Marker_Position->Y.last_Out_float);
      Marker_Position->X.Out_float = Marker_Position->X.last_Out_float + ( DT / (DT + FILTER_x_ORB )) * ( Marker_Position->X.Out_float - Marker_Position->X.last_Out_float);

      //Velocity.X.setpoint = Marker_Position->X.Out_float;
			//Velocity.Y.setpoint =	-Marker_Position->Y.Out_float;
			 
		 		if(fabs(	Marker_Position->X.point ) < 17  && flag_for_back==2 && fabs(	Marker_Position->X.point)<17   && Marker_data_ready==255 ){
	     	counter_for_landing++;
					}
				else if(counter_for_landing>0)	 counter_for_landing --;
					
				if(counter_for_landing > 500 ) 	land_marker=1;
						
			RC.RC_channel[7] += (Marker_Position->X.Out_float);
			RC.RC_channel[5] -= (Marker_Position->Y.Out_float);
			
					
	  	Marker_Position->X.last_Out_float = Marker_Position->X.Out_float;
	  	Marker_Position->Y.last_Out_float = Marker_Position->Y.Out_float;
		}	
}
void marker_kalman_1x1_init()
{
	
	vel_marker_x.state=0;
	vel_marker_x.C=1;
	vel_marker_x.A=1;
	vel_marker_x.B=DT;
	vel_marker_x.P=1;
	
	vel_marker_y.state=0;
	vel_marker_y.C=1;
	vel_marker_y.A=1;
	vel_marker_y.B=DT;
	vel_marker_y.P=1;
	
}
void window_kalman_1x1_init()
{
	
	window_x.state=0;
	window_x.C=1;
	window_x.A=1;
	window_x.B=DT;
	window_x.P=1;
	
	window_y.state=0;
	window_y.C=1;
	window_y.A=1;
	window_y.B=DT;
	window_y.P=1;
	
}