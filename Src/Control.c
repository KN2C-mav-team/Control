#include "Control.h"
#include "define.h"
#include "Kalman.h"

int kooft=0;
int frq;
int resolution;

int Servo_min,Servo_max;
//TIM_HandleTypeDef* htim;

System_Status Roll;
System_Status Pitch;
System_Status Yaw;
System_Status Altitude;
System_Status Altitude_Velocity;

_3D_Vector Position;

int Alt_Control_run=FALSE;
int Alt_Setpoint_state=0;
int Throttle_bias=0,Motor_force=0;
float On_Ground_Altitude=0;

int position_error=0;


void Pwm_frq(TIM_HandleTypeDef* _htim , int _frq, int _resolution)
{
	frq=_frq;
	resolution = _resolution;
	_htim->Init.Period = (SystemCoreClock/2) / frq;
  HAL_TIM_Base_Init(_htim);
	HAL_TIM_Base_Start_IT(_htim);
	HAL_TIM_PWM_Start(_htim,TIM_CHANNEL_1);
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
	Yaw.setpoint   =  -(Rc.Yaw/15.0f) + Yaw.setpoint;  
	Roll.setpoint  =  Rc.Roll;
	Pitch.setpoint =  Rc.Pitch;		
}



int Quad_On(_RC Rc)
{
//	#ifndef Quad
			if((float)RC.Throttle < (0.1f*(float)Throttle_range)  &&  (float)RC.Pitch>(0.7f*(float)angle_range + Pitch_offset) && (float)RC.Roll<(-0.7f*(float)angle_range + Roll_offset) &&(float)RC.Yaw < (-0.7f*(float)(angle_range/2.0f)))
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
			if((float)RC.Throttle < (0.1f*(float)Throttle_range)  &&  (float)RC.Pitch<(-0.7f*(float)angle_range + Pitch_offset))
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
	  Roll.Kp=(float)EEPROM_Read_int16_t(EEPROM_Roll_Kp) / 10.0f ;
		Roll.Ki=(float)EEPROM_Read_int16_t(EEPROM_Roll_Ki) / 10.0f ;
		Roll.Kd=(float)EEPROM_Read_int16_t(EEPROM_Roll_Kd) / 10.0f ;	

		Pitch.Kp=(float)EEPROM_Read_int16_t(EEPROM_Pitch_Kp) / 10.0f ;
		Pitch.Ki=(float)EEPROM_Read_int16_t(EEPROM_Pitch_Ki) / 10.0f ;
		Pitch.Kd=(float)EEPROM_Read_int16_t(EEPROM_Pitch_Kd) / 10.0f ;
			
		Yaw.Kp=(float)EEPROM_Read_int16_t(EEPROM_Yaw_Kp) / 10.0f ;
		Yaw.Ki=(float)EEPROM_Read_int16_t(EEPROM_Yaw_Ki) / 10.0f ;
		Yaw.Kd=(float)EEPROM_Read_int16_t(EEPROM_Yaw_Kd) / 10.0f ;
		
		Altitude.Kp_save=(float)EEPROM_Read_int16_t(EEPROM_Altitude_Kp) / 10.0f ;
		Altitude.Ki_save=(float)EEPROM_Read_int16_t(EEPROM_Altitude_Ki) / 10.0f ;
		Altitude.Kd_save=(float)EEPROM_Read_int16_t(EEPROM_Altitude_Kd) / 10.0f ;
		
		Altitude.Kp=Altitude.Kp_save;
		Altitude.Ki=Altitude.Ki_save;
		Altitude.Kd=Altitude.Kd_save;
	
		Altitude_Velocity.Kp_save=(float)EEPROM_Read_int16_t(EEPROM_Altitude_Velocity_Kp) / 10.0f ;		//**//
		Altitude_Velocity.Ki_save=(float)EEPROM_Read_int16_t(EEPROM_Altitude_Velocity_Ki) / 10.0f ;		//**//
		Altitude_Velocity.Kd_save=(float)EEPROM_Read_int16_t(EEPROM_Altitude_Velocity_Kd) / 10.0f ;		//**//

		Altitude_Velocity.Kp=Altitude_Velocity.Kp_save;		//**//
		Altitude_Velocity.Ki=Altitude_Velocity.Ki_save;		//**//
		Altitude_Velocity.Kd=Altitude_Velocity.Kd_save;		//**//
		
		
		Position.X.Kp=(float)EEPROM_Read_int16_t(EEPROM_Position_Kp) / 100.0f ;
		Position.X.Ki=(float)EEPROM_Read_int16_t(EEPROM_Position_Ki) / 100.0f ;
		Position.X.Kd=(float)EEPROM_Read_int16_t(EEPROM_Position_Kd) / 100.0f ;
	
		Position.Y.Kp=(float)EEPROM_Read_int16_t(EEPROM_Position_Kp) / 100.0f ;
		Position.Y.Ki=(float)EEPROM_Read_int16_t(EEPROM_Position_Ki) / 100.0f ;
		Position.Y.Kd=(float)EEPROM_Read_int16_t(EEPROM_Position_Kd) / 100.0f ;
	
	
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
		if(((unsigned int)Position.X.Kp > Position_P_coefficient) || ((unsigned int)Position.X.Ki > Position_I_coefficient) || ((unsigned int)Position.X.Kd > Position_D_coefficient) )		
			Position.X.error_coeficient=1;
		if(((unsigned int)Position.Y.Kp > Position_P_coefficient) || ((unsigned int)Position.Y.Ki > Position_I_coefficient) || ((unsigned int)Position.Y.Kd > Position_D_coefficient) )		
			Position.Y.error_coeficient=1;
		
		
	
		Roll.Max=1000;
		Roll.Min=-1000;
		Roll.Plimit=1000;
		Roll.Dlimit=1000;
		Roll.Ilimit=200;

		Pitch.Max=Roll.Max;
		Pitch.Min=Roll.Min;
		Pitch.Plimit=Roll.Plimit;
		Pitch.Dlimit=Roll.Ilimit;
		Pitch.Ilimit=Roll.Dlimit;


		Yaw.Max=300;
		Yaw.Min=-300;
		Yaw.Dlimit=300;
		Yaw.Plimit=300;
		Yaw.Ilimit=200;

		Altitude.Max=300;
		Altitude.Min=-300;
		Altitude.Plimit=100;
		Altitude.Dlimit=100;
		Altitude.Ilimit=300;
		
		Altitude_Velocity.Max=Altitude.Max;
		Altitude_Velocity.Min=Altitude.Min;
		Altitude_Velocity.Plimit=Altitude.Plimit;
		Altitude_Velocity.Dlimit=Altitude.Dlimit;
		Altitude_Velocity.Ilimit=Altitude.Ilimit;
		
		Position.X.Max=6;
		Position.X.Min=-6;
		Position.X.Plimit=6;
		Position.X.Dlimit=6;
		Position.X.Ilimit=6;
		
		Position.Y.Max=Position.X.Max;
		Position.Y.Min=Position.X.Min;
		Position.Y.Plimit=Position.X.Plimit;
		Position.Y.Dlimit=Position.X.Dlimit;
		Position.Y.Ilimit=Position.X.Ilimit;			
}


void Motor(int Speed,int Roll_in,int Pitch_in,int Yaw_in,int W)
{
	int MRU,MLU,MRD,MLD;

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
//       MLU        				MRU
// 		  \					     /
// 		   \				    /
// 	      \				   /
// 			   \		    /
// 		   		\	     /
// 			  	 |    |
//  		  	/      \
// 			   /			  \
// 		  	/		       \
// 		   /			      \
// 	    /					     \
// 	 MLD					  	MRD	 

	MRU	=Speed-(Roll_in/4)+(Pitch_in/4)-(Yaw_in/2);
	MLU	=Speed+(Roll_in/4)+(Pitch_in/4)+(Yaw_in/2);

	MRD	=Speed-(Roll_in/4)-(Pitch_in/4)+(Yaw_in/2);
	MLD	=Speed+(Roll_in/4)-(Pitch_in/4)-(Yaw_in/2);
	
	
//	
//	if(fabs(Roll.point)<15.0 && fabs(Pitch.point)<15.0 )
//	{	
//		MRU=MRU*0.5*(1/(cos(ToRad(Roll.point))*cos(ToRad(Roll.point)))+1/(cos(ToRad(Pitch.point))*cos(ToRad(Pitch.point))));
//		MLU=MLU*0.5*(1/(cos(ToRad(Roll.point))*cos(ToRad(Roll.point)))+1/(cos(ToRad(Pitch.point))*cos(ToRad(Pitch.point))));
//		MRD=MRD*0.5*(1/(cos(ToRad(Roll.point))*cos(ToRad(Roll.point)))+1/(cos(ToRad(Pitch.point))*cos(ToRad(Pitch.point))));
//		MLD=MLD*0.5*(1/(cos(ToRad(Roll.point))*cos(ToRad(Roll.point)))+1/(cos(ToRad(Pitch.point))*cos(ToRad(Pitch.point))));
//	}	
//	else if (fabs(Roll.point)>15.0)
//	{
//		MRU=MRU*0.5*(1.071+1/(cos(ToRad(Pitch.point))*cos(ToRad(Pitch.point))));	// (1/(cos(ToRad(15.0))*cos(ToRad(15.0)))) =>  1.071
//		MLU=MLU*0.5*(1.071+1/(cos(ToRad(Pitch.point))*cos(ToRad(Pitch.point))));
//		MRD=MRD*0.5*(1.071+1/(cos(ToRad(Pitch.point))*cos(ToRad(Pitch.point))));
//		MLD=MLD*0.5*(1.071+1/(cos(ToRad(Pitch.point))*cos(ToRad(Pitch.point))));
//	}
//	else if (fabs(Pitch.point)>15.0)
//	{
//		MRU=MRU*0.5*(1.071+1/(cos(ToRad(Roll.point))*cos(ToRad(Roll.point))));
//		MLU=MLU*0.5*(1.071+1/(cos(ToRad(Roll.point))*cos(ToRad(Roll.point))));
//		MRD=MRD*0.5*(1.071+1/(cos(ToRad(Roll.point))*cos(ToRad(Roll.point))));
//		MLD=MLD*0.5*(1.071+1/(cos(ToRad(Roll.point))*cos(ToRad(Roll.point))));
//	}
//	else
//	{
//		MRU=MRU*1.071;
//		MLU=MLU*1.071;
//		MRD=MRD*1.071;
//		MLD=MLD*1.071;
//	}
	
	if(MRU>=MAX_Motor_Speed)	MRU=MAX_Motor_Speed;
	if(MLU>=MAX_Motor_Speed)	MLU=MAX_Motor_Speed;
	if(MRD>=MAX_Motor_Speed)	MRD=MAX_Motor_Speed;
	if(MLD>=MAX_Motor_Speed)	MLD=MAX_Motor_Speed;
	
	if(MRU<=MIN_Motor_Speed)	MRU=MIN_Motor_Speed;
	if(MLU<=MIN_Motor_Speed)	MLU=MIN_Motor_Speed;
	if(MRD<=MIN_Motor_Speed)	MRD=MIN_Motor_Speed;
	if(MLD<=MIN_Motor_Speed)	MLD=MIN_Motor_Speed;
	
	Pwm_set( &htim2, MRU , _MRU );
	Pwm_set(  &htim2, MLU , _MLU );
	Pwm_set(  &htim2, MRD , _MRD );
	Pwm_set(  &htim2, MLD , _MLD );
      
//	if(MRU>MIN_Motor_Speed  &&  MRU<MIN_Motor_Speed+50 )		MRU=MIN_Motor_Speed+50;
//	if(MLU>MIN_Motor_Speed  &&  MLU<MIN_Motor_Speed+50 )		MLU=MIN_Motor_Speed+50;
//	if(MRD>MIN_Motor_Speed  &&  MRD<MIN_Motor_Speed+50 )		MRD=MIN_Motor_Speed+50;
//	if(MLD>MIN_Motor_Speed  &&  MLD<MIN_Motor_Speed+50 )		MLD=MIN_Motor_Speed+50;
	

//	if(Run_flag==_FLY_MODE  &&  (float)Throttle> 0.05*(float)Throttle_range)
//	{
//		PWM_Set( MRU_NUM , MRU );
//		PWM_Set( MLU_NUM , MLU );
//		PWM_Set( MRD_NUM , MRD );
//		PWM_Set( MLD_NUM , MLD );
//        
//        
//        MRU_PWM = MRU;
//        MRD_PWM = MRD;
//        MLU_PWM = MLU;
//        MLD_PWM = MLD;
//	}
//	else
//	{
//		PWM_Set( MRU_NUM , MIN_Motor_Speed );    
//		PWM_Set( MLU_NUM , MIN_Motor_Speed );
//		PWM_Set( MRD_NUM , MIN_Motor_Speed );
//		PWM_Set( MLD_NUM , MIN_Motor_Speed );
//	}

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
		case _Altitude_Gain:
			pdata[0]=EEPROM_Read_int16_t(EEPROM_Altitude_Kp);
			pdata[1]=EEPROM_Read_int16_t(EEPROM_Altitude_Ki);
			pdata[2]=EEPROM_Read_int16_t(EEPROM_Altitude_Kd);
			break;
		case _Altitude_Velocity_Gain:
			pdata[0]=EEPROM_Read_int16_t(EEPROM_Altitude_Velocity_Kp);
			pdata[1]=EEPROM_Read_int16_t(EEPROM_Altitude_Velocity_Ki);
			pdata[2]=EEPROM_Read_int16_t(EEPROM_Altitude_Velocity_Kd);
			break;
		case _Position_Gain:
			pdata[0]=EEPROM_Read_int16_t(EEPROM_Position_Kp);
			pdata[1]=EEPROM_Read_int16_t(EEPROM_Position_Ki);
			pdata[2]=EEPROM_Read_int16_t(EEPROM_Position_Kd);
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
		case _Altitude_Gain:
			EEPROM_Write_int16_t(EEPROM_Altitude_Kp,pdata[0]);
			EEPROM_Write_int16_t(EEPROM_Altitude_Ki,pdata[1]);
			EEPROM_Write_int16_t(EEPROM_Altitude_Kd,pdata[2]);
			break;
		case _Altitude_Velocity_Gain:
			EEPROM_Write_int16_t(EEPROM_Altitude_Velocity_Kp,pdata[0]);
			EEPROM_Write_int16_t(EEPROM_Altitude_Velocity_Ki,pdata[1]);
			EEPROM_Write_int16_t(EEPROM_Altitude_Velocity_Kd,pdata[2]);
			break;
		case _Position_Gain:
			EEPROM_Write_int16_t(EEPROM_Position_Kp,pdata[0]);
			EEPROM_Write_int16_t(EEPROM_Position_Ki,pdata[1]);
			EEPROM_Write_int16_t(EEPROM_Position_Kd,pdata[2]);
			break;
	}
}

void Set_zero_system_state(void)
{
			Roll.Ki=0;
			Pitch.Ki=0;
			Yaw.Ki=0;
			Altitude.Ki=0;
			Altitude_Velocity.Ki=0;
			Position.X.Ki = 0;
			Position.Y.Ki = 0;
	
//			Roll.integral_err=0;
//			Pitch.integral_err=0;
			Yaw.integral_err=0;
			Altitude.integral_err=0;
			Altitude_Velocity.integral_err=0;
			Position.X.integral_err=0;
			Position.Y.integral_err=0;
	
				
			Position.X.Out = 0;
			Position.X.Out_float = 0;
			Position.X.setpoint= 0;
			
			Position.Y.Out = 0;
			Position.Y.Out_float = 0;
			Position.Y.setpoint= 0;		

			Altitude.Out=0;				
			Altitude.Out_bias=0;
			
			Altitude_Velocity.Out=0;				
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
			
			On_Ground_Altitude=Altitude.point;
}


void Point2Controller(_IMU IMU,MPU_SENSOR Mpu)
{
	Yaw.last_point = Yaw.point;
	Yaw.point = IMU.Yaw;
	
	Roll.last_point = Roll.point;
	Roll.point = -1 *  IMU.Roll;
	
	Pitch.last_point = Pitch.point;
	Pitch.point = -1 *  IMU.Pitch;
	
	Roll.diff_point      = -1 *    ToDeg(Mpu.gyro_y);
  Pitch.diff_point     = -1 *    ToDeg(Mpu.gyro_x);
  Yaw.diff_point       =  1 *    ToDeg(Mpu.gyro_z);		
	
//	if(Ultra.point>=50)
//	{
//		Altitude.point	=	Cam_Position.Modified_POS_Z;
//		Altitude.diff_point	=	Cam_Position.POS_Z_Diff	;
//	}					
//	else 
//	{
		Altitude.point 			=	Ultra.point;
		Altitude.diff_point = z_vel.state;
//	}
	
	Altitude_Velocity.point = z_vel.state;
	Altitude_Velocity.diff_point = (Mahony.pure_acc_z)*100.0f;
	
	if( (Altitude.point>=25) )
	{
		Position.X.point = Cam_Position.Modified_POS_X;
		Position.Y.point = Cam_Position.Modified_POS_Y;
		
		Position.X.diff_point = Cam_Position.POS_X_Diff;
		Position.Y.diff_point = Cam_Position.POS_Y_Diff;	
	}
	else if( Altitude.point <=18)
	{
		Position.X.point = 0;
		Position.Y.point = 0;
		
		Position.X.diff_point = 0;
		Position.Y.diff_point = 0;		
	}
}


void Control_Altitude(int Alt_Control_SW)
{
	if(Alt_Control_SW==FALSE)
	{
		if(Alt_Control_run == TRUE)
		{
			Alt_Control_run = FALSE;
			Alt_Setpoint_state=0;
			Throttle_bias = Motor_force - RC.Throttle;  
		}		
		
		if( (RC.Throttle + Throttle_bias >= 0) )
			Motor_force = RC.Throttle + Throttle_bias;
	  else Motor_force=0;
	}
	else if(Alt_Control_SW == TRUE)
	{
			Fuzzy_Gain(Altitude_GAIN_SET);
		
			if(Alt_Control_run == FALSE)
			{
					Alt_Control_run=TRUE;

				  if(Altitude.point >= 20)
						Altitude.setpoint_real= Altitude.point;	
					else if(Altitude.point < 10)
						Altitude.setpoint_real= 35 ;	
					
					Altitude.Out_bias = Motor_force;
			}			
			if( Alt_Setpoint_state == 0)
				if( (RC.Throttle > (0.5f-THROTTLE_CENTER_THR)*Throttle_range) && (RC.Throttle < (0.5f+THROTTLE_CENTER_THR)*Throttle_range))
						Alt_Setpoint_state=1;
		  
			if( Alt_Setpoint_state == 1 )
			{
					if( (RC.Throttle < (0.5f-THROTTLE_CENTER_THR)*Throttle_range) )
						Altitude.setpoint_real = Altitude.setpoint_real + 1.2f * Altitude_Integral_Speed * (RC.Throttle - ((0.5f-THROTTLE_CENTER_THR)*Throttle_range)) * DT;
					else if( (RC.Throttle > (0.5f+THROTTLE_CENTER_THR)*Throttle_range) )
						Altitude.setpoint_real = Altitude.setpoint_real + Altitude_Integral_Speed * (RC.Throttle - ((0.5f+THROTTLE_CENTER_THR)*Throttle_range)) * DT; 
					
						if(Altitude.setpoint_real >= Max_Real_Altitude_Setpoint)
							Altitude.setpoint_real = Max_Real_Altitude_Setpoint; 
						else if (Altitude.setpoint_real <= 0) Altitude.setpoint_real=0;
			}
			
			if(Altitude.setpoint_real<=0) Altitude.setpoint_real=0;
			
			if((Altitude.setpoint_real-Altitude.point) >= Max_Altitude_Setpoint)				
			{
				Altitude.setpoint = Altitude.point + Max_Altitude_Setpoint;
				Altitude.setpoint_real = Altitude.point + Max_Altitude_Setpoint;
			}				
			else if((Altitude.setpoint_real-Altitude.point) <= -Max_Altitude_Setpoint)
			{
				Altitude.setpoint = Altitude.point - Max_Altitude_Setpoint;
				Altitude.setpoint_real = Altitude.point - Max_Altitude_Setpoint;
			}			
			else Altitude.setpoint=Altitude.setpoint_real;			
								
			Control(&Altitude);
			
			if( Altitude.Out + Altitude.Out_bias >= 0 )
				Motor_force= Altitude.Out + Altitude.Out_bias;	
			else Motor_force=0;	
	}	
}

void Control_Altitude_Velocity(int Alt_Control_SW)  
{	
	if(Alt_Control_SW==FALSE)
	{
		if(Alt_Control_run == TRUE)
		{
			Alt_Control_run = FALSE;
			Alt_Setpoint_state=0;
			Throttle_bias = Motor_force - RC.Throttle;  
		}				
		if( (RC.Throttle + Throttle_bias >= 0) )
			Motor_force = RC.Throttle + Throttle_bias;
	  else Motor_force=0;
	}
	else if( Alt_Control_SW == TRUE)
	{
		Fuzzy_Gain(Altitude_Velocity_GAIN_SET);
		if(Alt_Control_run == FALSE)
			{
					Alt_Control_run=TRUE;
					Altitude_Velocity.Out_bias = Motor_force;
					Altitude_Velocity.setpoint=0;
			}			
			
			// Auto Take-oFF
			if( RC.THR_CUT == 1)
			{
				Altitude_Velocity.setpoint=(65.0f - Altitude.point);
				if(Altitude_Velocity.setpoint >= 25)	Altitude_Velocity.setpoint=25;
				if(Altitude_Velocity.setpoint <= -25)	Altitude_Velocity.setpoint=-25;				
			}
			else 
			{
				Altitude_Velocity.setpoint=0;				
			}
			
			// Auto Land
			if((float)RC.Throttle < (0.1f*(float)Throttle_range) )
			{
				Altitude_Velocity.setpoint=(On_Ground_Altitude - Altitude.point);
				if(Altitude_Velocity.setpoint >= 25)	Altitude_Velocity.setpoint=25;
				if(Altitude_Velocity.setpoint <= -25)	Altitude_Velocity.setpoint=-25;		
				
				if(Altitude.point >= Max_Altitude_Range)	Altitude_Velocity.setpoint = -Max_Altitude_Velocity_Setpoint;				
			}
			
			if( Alt_Setpoint_state == 0)
				if( (RC.Throttle > (0.5f-THROTTLE_CENTER_THR)*Throttle_range) && (RC.Throttle < (0.5f+THROTTLE_CENTER_THR)*Throttle_range))
						Alt_Setpoint_state=1;		  
				
				
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
						Altitude_Velocity.setpoint = -Max_Real_Altitude_Velocity_Setpoint; 		

					if(Altitude.point >= Max_Altitude_Range) 	
						Altitude_Velocity.setpoint= Max_Altitude_Range - Altitude.point;
			}				
						
			Control(&Altitude_Velocity);
			
			if( (Altitude_Velocity.Out + Altitude_Velocity.Out_bias) >= 0 )
				Motor_force= Altitude_Velocity.Out + Altitude_Velocity.Out_bias;	
			else Motor_force=0;
	}	
}


void Fuzzy_Gain(char controler)
{		
	if(controler==Altitude_GAIN_SET)
	{
		if(Altitude.point<20)  // On HOVER
		{
				Altitude.Kp=Altitude.Kp_save;		
				Altitude.Ki=0.8f*Altitude.Ki_save;		
				Altitude.Kd=Altitude.Kd_save;			
		}
		if(Altitude.point>25)  // ON Air
		{
				Altitude.Kp=Altitude.Kp_save;		
				Altitude.Ki=Altitude.Ki_save;		
				Altitude.Kd=Altitude.Kd_save;			
		}				
		if(Altitude.point<5)  // On The Groaund
		{
				Altitude.Kp=Altitude.Kp_save;		
				Altitude.Ki=15.0f*Altitude.Ki_save;		
				Altitude.Kd=Altitude.Kd_save;
		}
		if((Altitude.setpoint<5) && (Altitude.point<15))  // For landing
		{
				Altitude.Kp=Altitude.Kp_save;		
				Altitude.Ki=15.0f*Altitude.Ki_save;		
				Altitude.Kd=Altitude.Kd_save;
		}			
	}
	else if(controler==Altitude_Velocity_GAIN_SET)
	{
		Altitude_Velocity.Kp=Altitude_Velocity.Kp_save;		
		Altitude_Velocity.Ki=Altitude_Velocity.Ki_save;		
		Altitude_Velocity.Kd=Altitude_Velocity.Kd_save;	
		
		if(((Altitude.point<5) && ((Altitude_Velocity.setpoint<-5) || (Altitude_Velocity.setpoint>5) ) ))  // On The Groaund
		{
				Altitude_Velocity.Kp=Altitude_Velocity.Kp_save;		
				Altitude_Velocity.Ki=15.0f*Altitude_Velocity.Ki_save;		
				Altitude_Velocity.Kd=Altitude_Velocity.Kd_save;
		}
	}
}




void Position_Control(_3D_Vector *Position,System_Status *_Roll,System_Status *_Pitch,System_Status *_Yaw)
{		
	// Check Data reliability
	if( Ultra.point - Cam_Position.Modified_POS_Z >=40)
	{		
		if(position_error<200)
			position_error++;		
	}
	else
	{
		if(position_error>0)
			position_error--;
	}
	

	// Run Controller
	if( (RC.THR_CUT ==1)  &&  (position_error<100)  &&  (Altitude.point>15) )
	{
			Control(&Position->X);	
			Control(&Position->Y);
					
			_Roll->setpoint  = _Roll->setpoint  - Position->Y.Out_float;
			_Pitch->setpoint = _Pitch->setpoint - Position->X.Out_float;
	}
	else
	{		
			Position->X.Out = 0;
			Position->X.Out_float = 0;
			Position->X.integral_err = 0;
			
			Position->Y.Out = 0;
			Position->Y.Out_float = 0;
			Position->Y.integral_err = 0;			
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

