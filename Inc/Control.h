#ifndef H_Control_H
#define H_Control_H



#include "stm32f4xx_hal.h"
#include "RC.h"
#include "EEPROM.h"
#include "Define.h"
#include "IMU.h"
#include "Mpu6050.h"
#include "Odometery.h"




#define _MRU    2
#define _MRD    4
#define _MLU    1
#define _MLD    3

#define MAX_Motor_Speed 		2040    //2040
#define RDY_Motor_Speed 		1430     //1352
#define MIN_Motor_Speed 		1000    //1000

#define Initial_Mass_Force	290		//**//		

#define 	_Roll_Gain  							0
#define 	_Pitch_Gain 							1
#define 	_Yaw_Gain   							2
#define		_Altitude_Gain						3
#define		_Altitude_Velocity_Gain		4
#define		_Position_Gain						5
#define   _opti_x_gain              6
#define   _opti_y_gain              7
#define   _Altitude_take_off_gain   8



// ***************  coefficient *************************

#define Roll_P_coefficient  40.0f
#define Roll_I_coefficient 	10.0f
#define Roll_D_coefficient  15.0f

#define Pitch_P_coefficient 40.0f
#define Pitch_I_coefficient 10.0f
#define Pitch_D_coefficient 15.0f

#define Yaw_P_coefficient  30.0f
#define Yaw_I_coefficient  10.0f
#define Yaw_D_coefficient  15.0f

#define Altitude_P_coefficient  3.0f
#define Altitude_I_coefficient  3.0f
#define Altitude_D_coefficient  3.0f

#define Altitude_Velocity_P_coefficient  3.0f
#define Altitude_Velocity_I_coefficient  3.0f
#define Altitude_Velocity_D_coefficient  3.0f

#define Position_P_coefficient				 	 1.0f
#define Position_I_coefficient				 	 1.0f
#define Position_D_coefficient				 	 1.0f

#define opti_x_P_coefficient				 	 1.0f
#define opti_x_I_coefficient				 	 3.0f
#define opti_x_D_coefficient				 	 1.0f

#define opti_y_P_coefficient				 	 1.0f
#define opti_y_I_coefficient				 	 3.0f
#define opti_y_D_coefficient				 	 1.0f

#define Altitude_take_off_P_coefficient  3.0f
#define Altitude_take_off_I_coefficient  3.0f
#define Altitude_take_off_D_coefficient  3.0f





//********************************************************

#define Max_Altitude_Setpoint				50
#define Max_Real_Altitude_Setpoint	250				//**//

#define Altitude_GAIN_SET						 0				//**//
#define Altitude_Velocity_GAIN_SET	 1				//**//

#define Altitude_Integral_Speed 		(0.5f)		//**//
#define THROTTLE_CENTER_THR					(0.125f) 	//**//

//********************************************************

#define Max_Altitude_Velocity_Setpoint				50				//**//
#define Max_Real_Altitude_Velocity_Setpoint		50				//**//
#define	Max_Altitude_Range										250

#define ALTITUDE_LIMIT                        300     // moj
#define ALTITUDE_TAKE_OFF_MAX                 100.0f      //MOJ

#define Altitude_Velocity_GAIN_SET						1					//**//

//******************************************************
#define F_CUT_VEL_X_OPTI                      4
#define FILTER_velocity_x_optical          1/(2*PI*F_CUT_VEL_X_OPTI)
#define F_CUT_VEL_Y_OPTI                      4
#define FILTER_velocity_y_optical          1/(2*PI*F_CUT_VEL_Y_OPTI)

//*********************************************************

#define F_CUT_OUTPUT_SIGNAL                    30          //MOJ: filter ruye vorudie driver
#define FILTER_output_signal_motor          1/(2*PI*F_CUT_OUTPUT_SIGNAL)// MOJ : **//





struct  _System_Status
{
   float diff_point;
	 float sorat_taghirat;     //khodam farsi neveshtam , kari nadashte bashid dg 
	 int   flag;               // moj
   float setpoint,setpoint_real;
	 float point,last_point,point_real;
	 float err,diff_err,integral_err,last_err;
	 
	 float last_setpoint,last_diffpoint;
	 
	 float Kp,Ki,Kd;
	 float Kp_save,Ki_save,Kd_save;		//**//
	 float dt;
	 float Max,Min,Ilimit,Dlimit,Plimit;
	 float P_save,I_save,D_save;
	 int   Out,Out_bias;	 
	 float Out_float,last_Out_float;
	 float offset;
	 char  error_coeficient;
};

typedef struct _System_Status System_Status;

struct	__3D_Vector
{
	System_Status X;
	System_Status Y;
	System_Status Z;	
};


typedef struct __3D_Vector _3D_Vector;

extern System_Status Roll;
extern System_Status Pitch;
extern System_Status Yaw;
extern System_Status Altitude;
extern System_Status Altitude_Velocity;
extern System_Status Altitude_take_off;


extern _3D_Vector Position;
extern _3D_Vector Velocity;


extern int Throttle_bias,Motor_force;
extern int position_error;

extern int MRU,MLU,MRD,MLD;
extern int last_MRU,last_MLU,last_MRD,last_MLD;


void Pwm_frq(TIM_HandleTypeDef* _htim , int _frq , int _resolution);
void Servo_init(TIM_HandleTypeDef* _htim , int _frq);
void Servo_Set_Angle(TIM_HandleTypeDef* _htim,int Angle);
void Pwm_set(TIM_HandleTypeDef* _htim,int _pwm,int Motor_num);
void Motor(int Speed,int Roll_in,int Pitch_in,int Yaw_in,int W);





void Rc2Controller(_RC Rc);
void Point2Controller(_IMU IMU,MPU_SENSOR Mpu);


void Read_Gain(int Mode,int *pdata);
void Set_Gain(int Mode,int *pdata);
void Set_zero_system_state(void);

int  Control(System_Status *In);
void control_init_(void);

void Control_Altitude(int Alt_Control_SW);
void Control_Altitude_Velocity(int Alt_Control_SW);
void Fuzzy_Gain(char controler);
void Position_Control(_3D_Vector *Position,System_Status *_Roll,System_Status *_Pitch,System_Status *_Yaw);
void Velocity_Control(_3D_Vector *Velocity,System_Status *_Roll,System_Status *_Pitch);
void Third_Person_control(System_Status *_Roll,System_Status *_Pitch,System_Status *_Yaw);
void First_Person_control(System_Status *_Roll,System_Status *_Pitch,System_Status *_Yaw,float Yaw_Offset);


int Quad_On(_RC Rc);
int Quad_Off(_RC Rc);

#endif 

