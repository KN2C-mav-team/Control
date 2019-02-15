#include "main.h"
#include "Control.h"
#include "define.h"
#include "Kalman.h"
//#include "Optical_Flow.h"
int kooft=0;
int frq;
int resolution;
int mojjjj =1;

int Servo_min,Servo_max;
//TIM_HandleTypeDef* htim;

System_Status Roll;
System_Status Pitch;
System_Status Yaw;
System_Status Altitude;
System_Status Altitude_Velocity;
System_Status Altitude_take_off;

_3D_Vector Velocity;
_3D_Vector ORB_position;


int Alt_Control_run=FALSE;
int Alt_Setpoint_state=0;
int Throttle_bias=0,Motor_force=0;
float On_Ground_Altitude=0;

int position_error=0;

int MRU,MLU,MRD,MLD;
int last_MRU,last_MLU,last_MRD,last_MLD;


void Pwm_frq(TIM_HandleTypeDef* _htim , int _frq, int _resolution)
{
	frq=_frq;
	resolution = _resolution;
	_htim->Init.Period = (SystemCoreClock/2) / frq;
  HAL_TIM_Base_Init(_htim);
	HAL_TIM_Base_Start_IT(_htim);
//	HAL_TIM_PWM_Start(_htim,TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(_htim,TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(_htim,TIM_CHANNEL_3);
	HAL_TIM_PWM_Start(_htim,TIM_CHANNEL_4);
}

void Servo_init(TIM_HandleTypeDef* _htim , int _frq)
{
	_htim->Init.Prescaler = SystemCoreClock/(2*1000000) - 1;
	_htim->Init.Period = 1000000/_frq - 1;
  HAL_TIM_Base_Init(_htim);
	HAL_TIM_Base_Start_IT(_htim);
	HAL_TIM_PWM_Start(_htim,TIM_CHANNEL_1);
	
}


void Servo_Set_Angle(TIM_HandleTypeDef* _htim,int Angle)
{
	_htim->Instance->CCR1 = 1000 + (int)((float)(1000/90))*(Angle+15);
}

void Pwm_set(TIM_HandleTypeDef* _htim,int _pwm,int Motor_num)
{
	switch(Motor_num)
	{
		case 1:
				_htim->Instance->CCR1 = (_pwm*_htim->Init.Period) / resolution;
				break;
		case 2:
				_htim->Instance->CCR2 = (_pwm*_htim->Init.Period) / resolution;
				break;
		case 3:
				_htim->Instance->CCR3 = (_pwm*_htim->Init.Period) / resolution;
				break;
		case 4:
				_htim->Instance->CCR4 = (_pwm*_htim->Init.Period) / resolution;
				break;
			
	}
}

void Rc2Controller(_RC Rc)
{
	Yaw.setpoint   =  (Rc.Yaw/15.0f) + Yaw.setpoint; 
	Yaw.sorat_taghirat = (Yaw.setpoint - Yaw.last_setpoint )/DT;  // moj
	Yaw.last_setpoint = Yaw.setpoint ; 	                      // moj
	if ( fabs(Yaw.sorat_taghirat) > 40 ){
																				Yaw.flag = 1;
																			} 
	if ( fabs(Yaw.sorat_taghirat) <= 40 ){
																				Yaw.flag = 0;
																			}
		
	Roll.setpoint  = Rc.Roll ;
	Pitch.setpoint = Rc.Pitch;		
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
    Roll.Kp=10;//(float)EEPROM_Read_int16_t(EEPROM_Roll_Kp) / 10.0f ;
	  Roll.Ki=0;//(float)EEPROM_Read_int16_t(EEPROM_Roll_Ki) / 10.0f ;
   	Roll.Kd=5;//(float)EEPROM_Read_int16_t(EEPROM_Roll_Kd) / 10.0f ;

		Pitch.Kp=10;//(float)EEPROM_Read_int16_t(EEPROM_Pitch_Kp) / 10.0f ;
		Pitch.Ki=0;//(float)EEPROM_Read_int16_t(EEPROM_Pitch_Ki) / 10.0f ;
		Pitch.Kd=5;//(float)EEPROM_Read_int16_t(EEPROM_Pitch_Kd) / 10.0f ;
	
		Yaw.Kp=(float)EEPROM_Read_int16_t(EEPROM_Yaw_Kp) / 10.0f ;
    Yaw.Ki=(float)EEPROM_Read_int16_t(EEPROM_Yaw_Ki) / 10.0f ;
    Yaw.Kd=(float)EEPROM_Read_int16_t(EEPROM_Yaw_Kd) / 10.0f ;
	
		Altitude_Velocity.Kp_save=4;//(float)EEPROM_Read_int16_t(EEPROM_Altitude_Velocity_Kp) / 10.0f ;		//**//
		Altitude_Velocity.Ki_save=7;//(float)EEPROM_Read_int16_t(EEPROM_Altitude_Velocity_Ki) / 10.0f ;		//**//
		//Altitude_Velocity.Kd_save=(float)EEPROM_Read_int16_t(EEPROM_Altitude_Velocity_Kd) / 10.0f ;		//**//
		Altitude_Velocity.Kd_save=0;
		
		Altitude_take_off.Kp_save=(float)EEPROM_Read_int16_t(EEPROM_Altitude_take_off_Kp) / 10.0f ;		//moj
		Altitude_take_off.Ki_save=(float)EEPROM_Read_int16_t(EEPROM_Altitude_take_off_Ki) / 10.0f ;		//moj
		Altitude_take_off.Kd_save=(float)EEPROM_Read_int16_t(EEPROM_Altitude_take_off_Kd) / 10.0f ;		//moj

		Altitude_take_off.Kp=Altitude_take_off.Kp_save;		//**//
		Altitude_take_off.Ki=Altitude_take_off.Ki_save;		//**//
		Altitude_take_off.Kd=Altitude_take_off.Kd_save;		//**//
		
		Altitude_Velocity.Kp=Altitude_Velocity.Kp_save;		//**//
		Altitude_Velocity.Ki=Altitude_Velocity.Ki_save;		//**//
		Altitude_Velocity.Kd=Altitude_Velocity.Kd_save;		//**//
		
		

//	
		Velocity.X.Kp_save=22;//14 ; //zamanike fcut khruji controler 40 bud khub bud hala kardamesh 8 fek konam bas ghavi tar konam//(float)EEPROM_Read_int16_t(EEPROM_opti_x_Kp) / 1000.0f ;
		Velocity.X.Ki_save=4;//6;//(float)EEPROM_Read_int16_t(EEPROM_opti_x_Ki) / 1000.0f ;
  	Velocity.X.Kd_save=0;//(float)EEPROM_Read_int16_t(EEPROM_opti_x_Kd) / 1000.0f ;
    Velocity.X.Ki=4;//6;
		
		Velocity.Y.Kp_save=22;//14;//(float)EEPROM_Read_int16_t(EEPROM_opti_y_Kp) / 1000.0f ;
   	Velocity.Y.Ki_save =0;//6;//(float)EEPROM_Read_int16_t(EEPROM_opti_y_Ki) / 1000.0f ;
		Velocity.Y.Kd_save=0;//(float)EEPROM_Read_int16_t(EEPROM_opti_y_Kd) / 1000.0f ;
		Velocity.Y.Ki=0;//6;
			
		ORB_position.X.Kp_save=0.05;// baraye halatike khoruji zavie bashad1.15;//(float)EEPROM_Read_int16_t(EEPROM_Position_Kp) / 1000.0f ;
   	ORB_position.X.Ki_save=0.0001;//0.35;//(float)EEPROM_Read_int16_t(EEPROM_Position_Ki) / 1000.0f ;
    ORB_position.X.Kd_save=0;//(float)EEPROM_Read_int16_t(EEPROM_Position_Kd) / 1000.0f ;
		
  	ORB_position.Y.Kp_save=ORB_position.X.Kp_save;
		ORB_position.Y.Ki_save=ORB_position.X.Ki_save;
		ORB_position.Y.Kd_save=ORB_position.X.Kd_save;
		
	  if(((unsigned int)Roll.Kp > Roll_P_coefficient)   || ((unsigned int)Roll.Ki > Roll_I_coefficient) 	|| ((unsigned int)Roll.Kd > Roll_D_coefficient) )	
			Roll.error_coeficient=1;
		if(((unsigned int)Pitch.Kp > Pitch_P_coefficient) || ((unsigned int)Pitch.Ki > Pitch_I_coefficient) || ((unsigned int)Pitch.Kd > Pitch_D_coefficient))
			Pitch.error_coeficient=1;
		if(((unsigned int)Yaw.Kp > Yaw_P_coefficient)     || ((unsigned int)Yaw.Ki > Yaw_I_coefficient) 		|| ((unsigned int)Yaw.Kd > Yaw_D_coefficient) )		
			Yaw.error_coeficient=1;
		if(((unsigned int)Altitude.Kp > Altitude_P_coefficient) || ((unsigned int)Altitude.Ki > Altitude_I_coefficient) || ((unsigned int)Altitude.Kd > Altitude_D_coefficient) )		
			Altitude.error_coeficient=1;		
		if(((unsigned int)Altitude_Velocity.Kp > Altitude_Velocity_P_coefficient) || ((unsigned int)Altitude_Velocity.Ki > Altitude_Velocity_I_coefficient) || ((unsigned int)Altitude_Velocity.Kd > Altitude_Velocity_D_coefficient) )		
			Altitude.error_coeficient=1;

		
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
	
		Roll.Max=1000;
		Roll.Min=-1000;
		Roll.Plimit=1000;
		Roll.Dlimit=1000;
		Roll.Ilimit=200;

		Pitch.Max=Roll.Max;
		Pitch.Min=Roll.Min;
		Pitch.Plimit=Roll.Plimit;
		Pitch.Ilimit=Roll.Ilimit;
		Pitch.Dlimit=Roll.Dlimit;


		Yaw.Max=300;
		Yaw.Min=-300;
		Yaw.Dlimit=300;
		Yaw.Plimit=300;
		Yaw.Ilimit=200;
		
		Altitude_Velocity.Max=150;
		Altitude_Velocity.Min=-150;
		Altitude_Velocity.Plimit=100;
		Altitude_Velocity.Dlimit=50;
		Altitude_Velocity.Ilimit=100;
		

		
		Velocity.X.Max = 750;//700;
		Velocity.X.Min = -750;//-700;
		Velocity.X.Plimit = 450;
		Velocity.X.Ilimit = 150;//600;
		Velocity.X.Dlimit = 50;
		
		Velocity.Y.Max =450;
		Velocity.Y.Min = -450;;
		Velocity.Y.Plimit = Velocity.X.Plimit;
		Velocity.Y.Ilimit =0;;
		Velocity.Y.Dlimit = Velocity.X.Dlimit;	
		
		Altitude_take_off.Max=25;
		Altitude_take_off.Min=-25;
		Altitude_take_off.Plimit=25;
		Altitude_take_off.Dlimit=0;
		Altitude_take_off.Ilimit=20;
}


void Motor(int Speed,int Roll_in,int Pitch_in,int Yaw_in,int W)
{


//Speed=Speed*0.5*(4-cos(Roll.point)-cos(Pitch.point));
//Speed=Speed*0.5*(1/cos(Roll.point)+1/cos(Pitch.point));
	
//       + mode control motor
// 	             MLU
// 							  |
// 							  |
// 							  |
//   	--- 			  |				  --- 
//  /   	\				 				/	  	\
//  	MLD - - - -   - - - - MRU
//  \     /								\     /
//    ---				  |				 	---		
// 							  |
// 							  |
// 							  |
// 							 MRD
	
// 	MRU	=Speed-(Roll_in*0.3535) -(Yaw_in/2); 
// 	MLU	=Speed+(Pitch_in*0.3535)+(Yaw_in/2);

// 	MRD	=Speed-(Pitch_in*0.3535)+(Yaw_in/2);
// 	MLD	=Speed+(Roll_in*0.3535) -(Yaw_in/2);
	
	
	
// 	 X mode control motor
//
//    MLU        			MRU
// 		  \					     /
// 		   \				    /
// 	      \				   /
// 			   \		    /
// 		   		\	|||| /
// 			  	 ||||||
//  		  	/ |||| \
// 			   /			  \
// 		  	/		       \
// 		   /			      \
// 	    /					     \
// 	  MLD				 	  	MRD	 

	MRU	=Speed-(Roll_in/4)+(Pitch_in/4)+(Yaw_in/2);  //fajina -
	MLU	=Speed+(Roll_in/4)+(Pitch_in/4)-(Yaw_in/2);  //+

	MRD	=Speed-(Roll_in/4)-(Pitch_in/4)-(Yaw_in/2);   // +
	MLD	=Speed+(Roll_in/4)-(Pitch_in/4)+(Yaw_in/2);     //-
	
	if(MRU>=MAX_Motor_Speed)	MRU=MAX_Motor_Speed;
	if(MLU>=MAX_Motor_Speed)	MLU=MAX_Motor_Speed;
	if(MRD>=MAX_Motor_Speed)	MRD=MAX_Motor_Speed;
	if(MLD>=MAX_Motor_Speed)	MLD=MAX_Motor_Speed;
	
	if(MRU<=MIN_Motor_Speed)	MRU=MIN_Motor_Speed;
	if(MLU<=MIN_Motor_Speed)	MLU=MIN_Motor_Speed;
	if(MRD<=MIN_Motor_Speed)	MRD=MIN_Motor_Speed;
	if(MLD<=MIN_Motor_Speed)	MLD=MIN_Motor_Speed;
	
//	Pwm_set( &htim2, MRU , _MRU );
//	Pwm_set(  &htim2, MLU , _MLU );
//	Pwm_set(  &htim2, MRD , _MRD );
//	Pwm_set(  &htim2, MLD , _MLD );

}

void Read_Gain(int Mode,int *pdata)
{
	switch(Mode)
	{
		case _Roll_Gain:
			pdata[0]=EEPROM_Read_int16_t(EEPROM_Roll_Kp);
			pdata[1]=EEPROM_Read_int16_t(EEPROM_Roll_Ki);
			pdata[2]=EEPROM_Read_int16_t(EEPROM_Roll_Kd);
			break;
		case _Pitch_Gain:
			pdata[0]=EEPROM_Read_int16_t(EEPROM_Pitch_Kp);
			pdata[1]=EEPROM_Read_int16_t(EEPROM_Pitch_Ki);
			pdata[2]=EEPROM_Read_int16_t(EEPROM_Pitch_Kd);
			break;
		case _Yaw_Gain:
			pdata[0]=EEPROM_Read_int16_t(EEPROM_Yaw_Kp);
			pdata[1]=EEPROM_Read_int16_t(EEPROM_Yaw_Ki);
			pdata[2]=EEPROM_Read_int16_t(EEPROM_Yaw_Kd);
			break;
		case _Altitude_Velocity_Gain:
			pdata[0]=EEPROM_Read_int16_t(EEPROM_Altitude_Velocity_Kp);
			pdata[1]=EEPROM_Read_int16_t(EEPROM_Altitude_Velocity_Ki);
			pdata[2]=EEPROM_Read_int16_t(EEPROM_Altitude_Velocity_Kd);
			break;
		case _ORB_position_Gain:
		  pdata[0]=EEPROM_Read_int16_t(EEPROM_Position_Kp);
			pdata[1]=EEPROM_Read_int16_t(EEPROM_Position_Ki);
			pdata[2]=EEPROM_Read_int16_t(EEPROM_Position_Kd);
		break;
		case _opti_x_gain:
			pdata[0]=EEPROM_Read_int16_t(EEPROM_opti_x_Kp);
			pdata[1]=EEPROM_Read_int16_t(EEPROM_opti_x_Ki);
			pdata[2]=EEPROM_Read_int16_t(EEPROM_opti_x_Kd);
			break;
		case _opti_y_gain:
			pdata[0]=EEPROM_Read_int16_t(EEPROM_opti_y_Kp);
			pdata[1]=EEPROM_Read_int16_t(EEPROM_opti_y_Ki);
			pdata[2]=EEPROM_Read_int16_t(EEPROM_opti_y_Kd);
			break;
		case _Altitude_take_off_gain:
			pdata[0]=EEPROM_Read_int16_t(EEPROM_Altitude_take_off_Kp);
			pdata[1]=EEPROM_Read_int16_t(EEPROM_Altitude_take_off_Ki);
			pdata[2]=EEPROM_Read_int16_t(EEPROM_Altitude_take_off_Kd);
			break;

	}
}

void Set_Gain(int Mode,int *pdata)
{
	// Set Gain to zero if lower than 0.1 and then save it	
	
	if(pdata[0] < 1)
		pdata[0]=0;
	
	if(pdata[1] < 1)
		pdata[1]=0;
	
	if(pdata[2] < 1)
		pdata[2]=0;
	
	switch(Mode)
	{
		case _Roll_Gain:
			EEPROM_Write_int16_t(EEPROM_Roll_Kp,pdata[0]);
			EEPROM_Write_int16_t(EEPROM_Roll_Ki,pdata[1]);
			EEPROM_Write_int16_t(EEPROM_Roll_Kd,pdata[2]);
			break;
		case _Pitch_Gain:
			EEPROM_Write_int16_t(EEPROM_Pitch_Kp,pdata[0]);
			EEPROM_Write_int16_t(EEPROM_Pitch_Ki,pdata[1]);
			EEPROM_Write_int16_t(EEPROM_Pitch_Kd,pdata[2]);
			break;
		case _Yaw_Gain:
			EEPROM_Write_int16_t(EEPROM_Yaw_Kp,pdata[0]);
			EEPROM_Write_int16_t(EEPROM_Yaw_Ki,pdata[1]);
			EEPROM_Write_int16_t(EEPROM_Yaw_Kd,pdata[2]);
			break;
		case _Altitude_Velocity_Gain:
			EEPROM_Write_int16_t(EEPROM_Altitude_Velocity_Kp,pdata[0]);
			EEPROM_Write_int16_t(EEPROM_Altitude_Velocity_Ki,pdata[1]);
			EEPROM_Write_int16_t(EEPROM_Altitude_Velocity_Kd,pdata[2]);
			break;
		case _ORB_position_Gain:
			EEPROM_Write_int16_t(EEPROM_Position_Kp,pdata[0]);
			EEPROM_Write_int16_t(EEPROM_Position_Ki,pdata[1]);
			EEPROM_Write_int16_t(EEPROM_Position_Kd,pdata[2]);
		  break;
		case _opti_x_gain:
			EEPROM_Write_int16_t(EEPROM_opti_x_Kp,pdata[0]);
			EEPROM_Write_int16_t(EEPROM_opti_x_Ki,pdata[1]);
			EEPROM_Write_int16_t(EEPROM_opti_x_Kd,pdata[2]);
			break;
		case _opti_y_gain:
			EEPROM_Write_int16_t(EEPROM_opti_y_Kp,pdata[0]);
			EEPROM_Write_int16_t(EEPROM_opti_y_Ki,pdata[1]);
			EEPROM_Write_int16_t(EEPROM_opti_y_Kd,pdata[2]);
			break;
		case _Altitude_take_off_gain:
			EEPROM_Write_int16_t(EEPROM_Altitude_take_off_Kp,pdata[0]);
			EEPROM_Write_int16_t(EEPROM_Altitude_take_off_Ki,pdata[1]);
			EEPROM_Write_int16_t(EEPROM_Altitude_take_off_Kd,pdata[2]);
			break;

	}
}

void Set_zero_system_state(void)
{
				optical_par.integral_x = 0;
	optical_par.integral_y = 0;
			Roll.Ki=0;
			Pitch.Ki=0;
			Yaw.Ki=0;

		  ORB_position.X.Ki=0;
	    ORB_position.Y.Ki=0;
	
			Roll.integral_err=0;
			Pitch.integral_err=0;
			Yaw.integral_err=0;
	
		  ORB_position.X.integral_err=0;
			ORB_position.Y.integral_err=0;
	
	    ORB_position.X.I_save=0;
	    ORB_position.Y.I_save=0;
	
			ORB_position.X.Out=0;
      ORB_position.X.Out_float=0;	
	
    	ORB_position.Y.Out=0;
      ORB_position.Y.Out_float=0;	
						
			Altitude_Velocity.Out=0;		
       Altitude_Velocity.Out_float=0;			
			Altitude_Velocity.Out_bias=0;
			
			Alt_Setpoint_state=0;				
			Throttle_bias=0;
			Motor_force=0;
					
			Yaw.setpoint=Yaw.point;  
			Yaw.last_setpoint=Yaw.point;
			
			Yaw.offset=Yaw.point;    
			Roll.offset=Roll.point;
			Pitch.offset=Pitch.point;	
			
			position_error=0;
			
			On_Ground_Altitude=Ultra.point;

      Velocity.X.Out_float=0;
      Velocity.Y.Out_float=0;
}


void Point2Controller(_IMU IMU,MPU_SENSOR Mpu)
{
	Yaw.last_point = Yaw.point;
	Yaw.point = -1 * IMU.Yaw;
	
	Roll.last_point = Roll.point;
	Roll.point = -1 *  IMU.Pitch;
	
	Pitch.last_point = Pitch.point;
	Pitch.point = 1 *  IMU.Roll;
	
	Roll.diff_point      = -1 *    ToDeg(Mpu.gyro_x);
  Pitch.diff_point     =  1 *    ToDeg(Mpu.gyro_y);
  Yaw.diff_point       = -1 *    ToDeg(Mpu.gyro_z);		
	
	
}




void Control_Altitude_Velocity(int Alt_Control_SW)  
{	
	Altitude_Velocity.point = z_vel.state;
	Altitude_Velocity.point = Altitude_Velocity.last_point + (( DT /(DT + FILTER_lpf_altitude_vel ))*(Altitude_Velocity.point - Altitude_Velocity.last_point )) ; 
	Altitude_Velocity.last_point = Altitude_Velocity.point;
	Altitude_Velocity.diff_point = (Mahony.pure_acc_z)*100.0f;
	
	if(Alt_Control_SW==FALSE)
	{
		if(Alt_Control_run == TRUE)
		{
			Alt_Control_run = FALSE;
			Alt_Setpoint_state=0;
			Altitude_Velocity.I_save=0;
			Altitude_Velocity.integral_err=0;
				Altitude_Velocity.Out_float=0;
			Altitude_Velocity.Out=0;
		}				
	  else {Altitude_Velocity.Out_float=0;
		    	Altitude_Velocity.Out=0;
		}
	}
	else if( Alt_Control_SW == TRUE)
	{
		Fuzzy_Gain(Altitude_Velocity_GAIN_SET);
		if(Alt_Control_run == FALSE)
			{
					Alt_Control_run=TRUE;
					Altitude_Velocity.setpoint=0;
			}			
			

			if( Alt_Setpoint_state == 0){
																		if( (RC.Throttle > (0.5f-THROTTLE_CENTER_THR)*Throttle_range) && (RC.Throttle < (0.5f+THROTTLE_CENTER_THR)*Throttle_range))
																		{ Alt_Setpoint_state=1;		  
																			}
			                           	}
				
				
			if( Alt_Setpoint_state == 1 )
			{
					if( (RC.Throttle < (0.5f-THROTTLE_CENTER_THR)*Throttle_range) )
						Altitude_Velocity.setpoint = 1.2f*Altitude_Integral_Speed * (RC.Throttle - (0.5f*Throttle_range)); 
					else if( (RC.Throttle > (0.5f+THROTTLE_CENTER_THR)*Throttle_range) )
						Altitude_Velocity.setpoint = Altitude_Integral_Speed * (RC.Throttle - (0.5f*Throttle_range)); 
					else	Altitude_Velocity.setpoint=0;
					
					if(Altitude_Velocity.setpoint >= Max_Real_Altitude_Velocity_Setpoint)
						Altitude_Velocity.setpoint = Max_Real_Altitude_Velocity_Setpoint; 
					else if(Altitude_Velocity.setpoint <= -Max_Real_Altitude_Velocity_Setpoint)
						Altitude_Velocity.setpoint= -Max_Real_Altitude_Velocity_Setpoint; 		

					if(Ultra.point >= Max_Altitude_Range) 	
						Altitude_Velocity.setpoint= Max_Altitude_Range - Ultra.point;		   	}				

//   		if(  RC.THR_CUT==1 &&  Alt_Setpoint_state == 0	)
//			{			
//				Altitude_Velocity.setpoint=(ALTITUDE_TAKE_OFF_MAX - Ultra.point)/3;
//				if(Altitude_Velocity.setpoint >= 65)	Altitude_Velocity.setpoint=65;
//				if(Altitude_Velocity.setpoint <= -65)	Altitude_Velocity.setpoint=-65;
//				
//			}
//				if(	RC.THR_CUT==0 &&  Alt_Setpoint_state == 0) 
//			{	
//				Altitude_Velocity.setpoint=(Ground_ALTITUDE - Ultra.point);
//				if(Altitude_Velocity.setpoint >= 65)	Altitude_Velocity.setpoint=65;
//				
//				if(Altitude_Velocity.setpoint <= -65)	Altitude_Velocity.setpoint=-65;		
//				
//				if(Ultra.point >= Max_Altitude_Range)	Altitude_Velocity.setpoint = -Max_Altitude_Velocity_Setpoint;		
//       if( Ultra.point <13 ){
//				Motor(MIN_Motor_Speed,0,0,0,0);				
//			 }
//			}
			
				Control(&Altitude_Velocity);
			
					if(	RC.THR_CUT==0 &&  Alt_Setpoint_state == 0 &&  Ultra.point<13) {
						Altitude_Velocity.Out =0;
						Altitude_Velocity.Out_float=0;
					}
	}	
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
		Altitude_Velocity.Kp=Altitude_Velocity.Kp_save;		
		Altitude_Velocity.Ki=Altitude_Velocity.Ki_save;		
		Altitude_Velocity.Kd=0;		
		if(Ultra.point<15 )  // On The Groaund
		{
		Altitude_Velocity.Kp=Altitude_Velocity.Kp_save;		
		Altitude_Velocity.Ki=15.0f*Altitude_Velocity.Ki_save;		
		Altitude_Velocity.Kd=0;
		}
	}
}
 

void Velocity_Control(_3D_Vector *Velocity,System_Status *_Roll,System_Status *_Pitch)
{		
	if( RC.RC_SW ==1 )
	{
		optical_par.integral_x += x_vel.state * DT ;
	  optical_par.integral_y += y_vel.state * DT;		
		Velocity->X.setpoint=-8;
		if(optical_par.integral_x>250){  //50=250cm //70=280    i-50=(2/3 ) (x-250)
		Velocity->X.setpoint=0;
		}
	  Velocity->Y.setpoint=  ORB_position.X.Out_float; //mosbat manfi bayad check beshe  

		
	  Velocity->X.point = -x_vel.state ;
	  Velocity->Y.point = -y_vel.state;	
		
		Velocity->Y.diff_point =-optical_par.acc_Y;
  	Velocity->X.diff_point =-optical_par.acc_X;
		Velocity->Y.diff_point=Velocity->Y.last_diffpoint+( DT/(DT + FILTER_diffpoint_y_optical))*(Velocity->Y.diff_point - Velocity->Y.last_diffpoint);
	  Velocity->X.diff_point=Velocity->X.last_diffpoint+( DT/(DT + FILTER_diffpoint_x_optical))*(Velocity->X.diff_point - Velocity->X.last_diffpoint);

	  Velocity->Y.last_diffpoint = Velocity->Y.diff_point;
		Velocity->X.last_diffpoint = Velocity->X.diff_point;
		Fuzzy_Gain(OPTICAL_GAIN_SET);
		
			Control(&Velocity->X);	
			Control(&Velocity->Y);
		
	  	Velocity->Y.Out_float = Velocity->Y.last_Out_float + ( DT / (DT + FILTER_velocity_y_optical )) * ( Velocity->Y.Out_float - Velocity->Y.last_Out_float);
      Velocity->X.Out_float = Velocity->X.last_Out_float + ( DT / (DT + FILTER_velocity_x_optical )) * ( Velocity->X.Out_float - Velocity->X.last_Out_float);
		
	  	_Roll->setpoint  = _Roll->setpoint + Velocity->X.Out_float;
	 		_Pitch->setpoint = _Pitch->setpoint + Velocity->Y.Out_float;
		
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
      _Roll->setpoint  = _Roll->setpoint   +  Velocity->X.Out_float;	
	  	_Pitch->setpoint = _Pitch->setpoint	 +  Velocity->Y.Out_float;	

	}
	
}

void Third_Person_control(System_Status *_Roll,System_Status *_Pitch,System_Status *_Yaw)
{
	float _R,_P,_Y;
	
	_R=	_Roll->setpoint*cos((ToRad(((-_Yaw->point) - _Yaw->offset))))		+	_Pitch->setpoint*sin((ToRad(((-_Yaw->point) - _Yaw->offset))));
	_P=	_Roll->setpoint*(-sin((ToRad(((-_Yaw->point) - _Yaw->offset)))))	+	_Pitch->setpoint*cos((ToRad(((-_Yaw->point) - _Yaw->offset))));
	_Y=	_Yaw->setpoint;	
	
	Roll.setpoint=_R;
	Pitch.setpoint=_P;	
	Yaw.setpoint=_Y;	
}

void First_Person_control(System_Status *_Roll,System_Status *_Pitch,System_Status *_Yaw,float Sytem_Yaw_Offset)
{
		float _R,_P,_Y;
	
	_R=	_Roll->setpoint*cos((ToRad((Sytem_Yaw_Offset ))))		+	_Pitch->setpoint*sin((ToRad((Sytem_Yaw_Offset ))));
	_P=	_Roll->setpoint*(-sin((ToRad((Sytem_Yaw_Offset )))))	+	_Pitch->setpoint*cos((ToRad((Sytem_Yaw_Offset))));
	_Y=	_Yaw->setpoint;	
	
	_Roll->setpoint=_R;
	_Pitch->setpoint=_P;	
	_Yaw->setpoint=_Y;
}

void ORB_Position_control(_3D_Vector *ORB_position,_MPC *MPC,System_Status *_Roll,System_Status *_Pitch)
	{
   Fuzzy_Gain(ORB_GAIN_SET);
	 if( RC.RC_SW ==1 )   /// chek shavad thr_cut
////		if( RC.THR_CUT ==1 && ORB_Position.data_check<100 )
////		if( RC.THR_CUT ==1  && (fabs(ORB_position->X.point- ORB_position->X.setpoint )> 40 || fabs(ORB_position->Y.point - ORB_position->Y.setpoint )> 40 ) )
	 {
		//  ORB_Position.RUN_FLAG=1;

		 
		  //ORB_position->Z.point=ORB_Position.POS_Z ;
//		  if(ORB_position->X.flag==0){
//																	ORB_position->X.flag = 1;	
//																	ORB_position->X.setpoint = ORB_position->X.point ;
//																	}
//			 if(ORB_position->Y.flag==0){
//																	ORB_position->Y.flag = 1;	
//																	ORB_position->Y.setpoint = ORB_position->Y.point ;
//																	}
//			 if(fabs(ORB_position->X.point - ORB_position->X.setpoint )> 25 || fabs(ORB_position->Y.point - ORB_position->Y.setpoint )> 13 ){
//		   ORB_position->X.Kp=0.8*ORB_position->X.Kp;
//	   	 ORB_position->X.Ki=ORB_position->X.Ki;
//	  	 ORB_position->X.Kd=0.8*ORB_position->X.Kd;
//				 
//			 ORB_position->Y.Kp=ORB_position->X.Kp;
//	     ORB_position->Y.Ki=ORB_position->X.Ki;
//	     ORB_position->Y.Kd=ORB_position->X.Kd; 
//			 }
//			 if(fabs(ORB_position->X.point - ORB_position->X.setpoint )> 45 || fabs(ORB_position->Y.point - ORB_position->Y.setpoint )> 20 ){
//		   ORB_position->X.Kp=0.5*ORB_position->X.Kp;
//	   	 ORB_position->X.Ki=ORB_position->X.Ki;
//	  	 ORB_position->X.Kd=0.5*ORB_position->X.Kd;
//				 
//			 ORB_position->Y.Kp=ORB_position->X.Kp;
//	     ORB_position->Y.Ki=ORB_position->X.Ki;
//	     ORB_position->Y.Kd=ORB_position->X.Kd; 
//			 }
		 
//		 	 if((ORB_position->X.setpoint - ORB_position->X.point  ) > 350 ){
//				  ORB_position->X.point=0;
//				  ORB_position->X.setpoint=350;

//			 }
//			 if (( ORB_position->Y.setpoint - ORB_position->Y.point)> 350 ){
//				  ORB_position->Y.point=0;
//				  ORB_position->Y.setpoint=350;
//			 }
//			 
//			 if((ORB_position->X.setpoint - ORB_position->X.point  ) < -350 ){
//				  ORB_position->X.point=0;
//				  ORB_position->X.setpoint=-350;

//			 }
//			 if (( ORB_position->Y.setpoint - ORB_position->Y.point) < -350 ){
//				  ORB_position->Y.point=0;
//				  ORB_position->Y.setpoint=-350;
//			 }
			 
//			ORB_position->X.diff_point = -optical_par.real_vel_X;
//   		ORB_position->Y.diff_point = +optical_par.real_vel_Y;	
//			ORB_position->X.diff_point = ORB_position->X.last_diffpoint + ((DT/(DT+F_CUT_DIFF)) * ( ORB_position->X.diff_point  -ORB_position->X.last_diffpoint));
//			ORB_position->Y.diff_point = ORB_position->Y.last_diffpoint + ((DT/(DT+F_CUT_DIFF)) * ( ORB_position->Y.diff_point  -ORB_position->Y.last_diffpoint));

//			 ORB_position->X.last_diffpoint =  ORB_position->X.diff_point;
//			 ORB_position->Y.last_diffpoint =  ORB_position->Y.diff_point;
			 
			Control(&ORB_position->X);	
		//	Control(&ORB_position->Y);
		
	  //	ORB_position->Y.Out_float = ORB_position->Y.last_Out_float + ( DT / (DT + FILTER_y_ORB )) * ( ORB_position->Y.Out_float - ORB_position->Y.last_Out_float);
    //  ORB_position->X.Out_float = ORB_position->X.last_Out_float + ( DT / (DT + FILTER_x_ORB )) * ( ORB_position->X.Out_float - ORB_position->X.last_Out_float);

//      Velocity.X.setpoint = ORB_position->X.Out_float;
//			Velocity.Y.setpoint =	-ORB_position->Y.Out_float;
//			 
//        Velocity.X.setpoint = 0;
//        Velocity.Y.setpoint =	0;

			
	  //	_Roll->setpoint  = _Roll->setpoint    +   ORB_position->X.Out_float ;// chek shavad k x ba rolle 
	 	//	_Pitch->setpoint = _Pitch->setpoint   +   ORB_position->Y.Out_float ;  // chek shavad k + she ya -
		
//	  	ORB_position->X.last_Out_float = ORB_position->X.Out_float;
//	  	ORB_position->Y.last_Out_float = ORB_position->Y.Out_float;
		
	 }
  	else
	 {	
		  ORB_position->X.Out_float =0;}
//		//  ORB_Position.RUN_FLAG=0;
//		 
//      ORB_position->X.flag = 0;
//      ORB_position->Y.flag = 0;
//		 
//			ORB_position->X.Out = 0;
//			ORB_position->X.Out_float = 0;
//			ORB_position->X.integral_err = 0;
//			
//			ORB_position->Y.Out = 0;
//			ORB_position->Y.Out_float = 0;
//			ORB_position->Y.integral_err = 0;		
//		 
//    	_Roll->setpoint  = _Roll->setpoint  +  ORB_position->X.Out_float ;	
//	 		_Pitch->setpoint = _Pitch->setpoint +  ORB_position->Y.Out_float ;	
////      Velocity.X.setpoint = 0;
////			Velocity.Y.setpoint =	0;		 
	
}