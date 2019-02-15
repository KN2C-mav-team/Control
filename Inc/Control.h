#ifndef H_Control_H
#define H_Control_H



#include "stm32f4xx_hal.h"
#include "RC.h"
#include "EEPROM.h"
#include "Define.h"
#include "IMU.h"
#include "Mpu6050.h"
#include "Odometery.h"
#include "par_optical.h"
#include  "math.h"

#define		_Altitude_Velocity_Gain		4
#define   _opti_x_gain              6
#define   _opti_y_gain              7
#define   _Altitude_take_off_gain   8
#define		_ORB_position_Gain				9


// ***************  coefficient *************************

#define Altitude_P_coefficient  3.0f
#define Altitude_I_coefficient  3.0f
#define Altitude_D_coefficient  3.0f

#define Altitude_Velocity_P_coefficient  3.0f
#define Altitude_Velocity_I_coefficient  3.0f
#define Altitude_Velocity_D_coefficient  3.0f



#define opti_x_P_coefficient				 	 1.0f
#define opti_x_I_coefficient				 	 1.0f
#define opti_x_D_coefficient				 	 0.50f

#define opti_y_P_coefficient				 	 1.0f
#define opti_y_I_coefficient				 	 1.0f
#define opti_y_D_coefficient				 	 0.50f

#define Altitude_take_off_P_coefficient  3.0f
#define Altitude_take_off_I_coefficient  3.0f
#define Altitude_take_off_D_coefficient  3.0f

//#define ORB_position_P_coefficient		   0.1f
//#define ORB_position_I_coefficient	  	 0.1f
//#define ORB_position_D_coefficient		 	 0.1f

#define ORB_position_P_coefficient		   0.10f
#define ORB_position_I_coefficient	  	 0.10f
#define ORB_position_D_coefficient		 	 0.10f




//********************************************************

#define Max_Altitude_Setpoint				50
#define Max_Real_Altitude_Setpoint	250				//**//

#define Altitude_GAIN_SET						 0				//**//
#define Altitude_Velocity_GAIN_SET	 1				//**//
#define ORB_GAIN_SET						 	   2
#define OPTICAL_GAIN_SET						 3


#define Altitude_Integral_Speed 		(0.5f)		//**//
#define THROTTLE_CENTER_THR					(0.125f) 	//**//

//********************************************************

#define Max_Altitude_Velocity_Setpoint				15				//**//
#define Max_Real_Altitude_Velocity_Setpoint		50				//**//
#define	Max_Altitude_Range										250

#define ALTITUDE_LIMIT                        300     // moj
#define ALTITUDE_TAKE_OFF_MAX                125.0f  //MOJ
#define Ground_ALTITUDE                       4.0f    //moj

#define Altitude_Velocity_GAIN_SET						1					//**//

#define Field_of_view_tan_X                  ( tan(ToRad(31)))
#define Field_of_view_tan_Y                   (tan(ToRad(25)))


//******************************************************
#define F_CUT_VEL_X_OPTI                      8
#define FILTER_velocity_x_optical          1/(2*PI*F_CUT_VEL_X_OPTI)
#define F_CUT_VEL_Y_OPTI                      8
#define FILTER_velocity_y_optical          1/(2*PI*F_CUT_VEL_Y_OPTI)
#define F_CUT_DIFF_Y_OPTI                     10
#define FILTER_diffpoint_x_optical         1/(2*PI*F_CUT_DIFF_Y_OPTI)
#define FILTER_diffpoint_y_optical         1/(2*PI*F_CUT_DIFF_Y_OPTI)
#define F_CUT_DFFF_ferequency                 2.5
#define F_CUT_DIFF                          1/(2*PI*F_CUT_DFFF_ferequency) 
#define F_CUT_lpf_al_vel                    4
#define FILTER_lpf_altitude_vel             1/(2*PI*F_CUT_lpf_al_vel)
#define F_CUT_DFFF_window_ferequency                     2
#define F_CUT_DIFF_window            1/(2*PI*F_CUT_DFFF_window_ferequency) 
//*********************************************************
#define F_CUT_X_ORB                     10
#define FILTER_x_ORB          1/(2*PI*F_CUT_X_ORB)
#define F_CUT_Y_ORB                     10
#define FILTER_y_ORB          1/(2*PI*F_CUT_Y_ORB)

//*********************************************************

#define F_CUT_OUTPUT_SIGNAL                    30          //MOJ: filter ruye vorudie driver
#define FILTER_output_signal_motor          1/(2*PI*F_CUT_OUTPUT_SIGNAL)// MOJ : **//

#define F_CUT_FILTERALTITUDE                        5
#define FILTERALTITUDE                       1/(2*PI*F_CUT_FILTERALTITUDE)


#define Filter_hpf_optical               0.7


extern int counter_flag_for_accept_window;


struct  _System_Status
{
   float diff_point,last_diff_point;
	 int   flag;               // moj
   float setpoint,setpoint_real;
	 float point,last_point,point_real;
	 float err,diff_err,integral_err,last_err;
	float hpf, last_hpf;
	 
	 float last_setpoint,last_diffpoint;
	float acc; 
	 float Kp,Ki,Kd;
	 float Kp_save,Ki_save,Kd_save;		//**//
	 float dt;
	 float Max,Min,Ilimit,Dlimit,Plimit;
	 float P_save,I_save,D_save;
	 int   Out,Out_bias;	 
	 float Out_float,last_Out_float;
	 float offset;
	 float scale_number ; 
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

extern System_Status Altitude;
extern System_Status Altitude_Velocity;
extern System_Status Altitude_take_off;

extern _3D_Vector Velocity;
extern _3D_Vector ORB_position;
extern _3D_Vector Marker_Position;
extern _3D_Vector window_detection;


extern _Kalman1x1 vel_marker_x,vel_marker_y,window_x,window_y;

extern int Throttle_bias,Motor_force;
extern int position_error;

void Rc2Controller(_RC Rc);
void Point2Controller(_IMU IMU,MPU_SENSOR Mpu);

void Set_zero_system_state(void);

int  Control(System_Status *In);
void control_init_(void);

void Control_Altitude_Velocity(int Alt_Control_SW);
void altitude_control(int Alt_Control_THR);
void Fuzzy_Gain(char controler);
void Velocity_Control(_3D_Vector *Velocity);
void First_Person_control();
void Third_Person_control(int _Roll_setpoint,int _Pitch_setpoint ,int _Yaw_point,int corrected_Roll_setpoint,int corrected_Pitch_setpoint);
void ORB_Position_control(_3D_Vector *ORB_position,_MPC *MPC);
void Window_detection(_3D_Vector *window_detection ,_MPC *MPC );
void Marker_Position_Control(_3D_Vector *Marker_Position,_MPC *MPC);
void marker_kalman_1x1_init();
void window_kalman_1x1_init();


int Quad_On(_RC Rc);
int Quad_Off(_RC Rc);

#endif 

