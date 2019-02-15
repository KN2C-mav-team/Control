#include "main.h"
#include "Telemetri.h"

#include "IMU.h"
#include "MS5611.h"
#include "RC.h"
#include "NRF.h"


char UART_bufferf[100];
char data_r = 0;
uint8_t station_data = 0;
char print_[50];

uint64_t counter = 0;
int lock_time =0;

void UART_send_buffer(int length)
{
 // HAL_UART_Transmit(&huart4,(uint8_t*)UART_bufferf,length,100);	
}



void Station_Data_R(void)
{	
	char station_print_enter = 0;
	int data[10];	
	
	if(data_r == 1)
		{
			switch(station_data)
				{
					
					case 'A':
						 print_[ACCEL]=(print_[ACCEL]==0) ? 1:0;
						 break;
					case 'b':
						 print_[BAT]=(print_[BAT]==0) ? 1:0;
						 break;
					case 'G':
						 print_[GYRO]=(print_[GYRO]==0) ? 1:0;
						 break;
					case 'g':
						 print_[GRAVITY]=(print_[GRAVITY]==0) ? 1:0;
						 break;
					case 'e':
						 print_[Euler_angles]=(print_[Euler_angles]==0) ? 1:0;
						 break;
					case 'C':
						 print_[ms_coefficient]=(print_[ms_coefficient]==0) ? 1:0;
						 break;
					case 'c':
						 print_[ms_detail]=(print_[ms_detail]==0) ? 1:0;
						 break;
					case 'z':
						 print_[Vel_Z]=(print_[Vel_Z]==0) ? 1:0;
						 break;
					case 'p':
						 print_[PTAM_DATA]=(print_[PTAM_DATA]==0) ? 1:0;
						 break;	
					case 'o':
						 print_[Optical_Flow]=(print_[Optical_Flow]==0) ? 1:0;
						 break;	
					case 's':
							RC.init = 0;
						 break;
					case 'q':
						 print_[opti_y_signal]=(print_[opti_y_signal]==0) ? 1:0;
						 break;
					case 'w':
						 print_[opti_x_signal]=(print_[opti_x_signal]==0) ? 1:0;
						 break;
					case 'R':
						 print_[RC_CHANNEL]=(print_[RC_CHANNEL]==0) ? 1:0;
						 break;
					case 'r':
						 print_[RC_ATITUDE]=(print_[RC_ATITUDE]==0) ? 1:0;
						 break;
					case 'h':
						 print_[Altitude_Detail]=(print_[Altitude_Detail]==0) ? 1:0;
						 break;
					case '0':
						 print_[Roll_Control_signal]=(print_[Roll_Control_signal]==0) ? 1:0;
						 break;
					case '1':
						 print_[Roll_Gain]=(print_[Roll_Gain]==0) ? 1:0;
						 break;
					case '2':
						 print_[Pitch_Gain]=(print_[Pitch_Gain]==0) ? 1:0;
						 break;
					case '3':
						 print_[Yaw_Gain]=(print_[Yaw_Gain]==0) ? 1:0;
						 break;	
	    		case '5':
						 print_[Altitude_Velocity_Gain]=(print_[Altitude_Velocity_Gain]==0) ? 1:0;
						 break;
					case '6':
						 print_[Altitude_Velocity_Control_signal]=(print_[Altitude_Velocity_Control_signal]==0) ? 1:0;
						 break;	
					case '8':
						 print_[Yaw_Control_signal]=(print_[Yaw_Control_signal]==0) ? 1:0;
						 break;
					case '9':
						 print_[Pitch_Control_signal]=(print_[Pitch_Control_signal]==0) ? 1:0;
						 break;
					case '`':
						 print_[ORB_position_Gain]=(print_[ORB_position_Gain]==0) ? 1:0;
						 break;	
					case ']':
						 print_[opti_x_read]=(print_[opti_x_read]==0) ? 1:0;
						 break;	
					case '[':
						 print_[opti_y_read]=(print_[opti_y_read]==0) ? 1:0;
						 break;	
					case '<':
						 print_[Altitude_take_off_Gain]=(print_[Altitude_take_off_Gain]==0) ? 1:0;
						 break;		
					case '-':
						 print_[Position_Control_signal]=(print_[Position_Control_signal]==0) ? 1:0;
						 break;		
					case '>':
						 print_[Altitude_take_off_Gain_set]=(print_[Altitude_take_off_Gain_set]==0) ? 1:0;
						 if(print_[Altitude_take_off_Gain_set] == 0)
					{
						data[0] = (int)(Altitude_take_off_P_coefficient * (RC.HOV_THR*10)) ;
						data[1] = (int)(Altitude_take_off_I_coefficient * (RC.RC_TRIM*10)) ;
						data[2] = (int)(Altitude_take_off_D_coefficient * (RC.HOV_PIT*10)) ;
						Set_Gain(_Altitude_take_off_gain,data);
					}		
						 break;	
					case '!':
						 print_[Roll_Gain_set]=(print_[Roll_Gain_set]==0) ? 1:0;
						 if(print_[Roll_Gain_set] == 0)
							{
								data[0] = (int)(Roll_P_coefficient * (RC.RC_channel[0])/1000) ;
								data[1] = (int)(Roll_I_coefficient * (RC.RC_channel[1])/1000 );
								data[2] = (int)(Roll_D_coefficient * (RC.RC_channel[2])/1000 );
								Set_Gain(_Roll_Gain,data);
							}
						 break;
					case '@':
						 print_[Pitch_Gain_set]=(print_[Pitch_Gain_set]==0) ? 1:0;
						 if(print_[Pitch_Gain_set] == 0)
							{
								data[0] = (int)(Pitch_P_coefficient * (RC.RC_channel[0]/1000)) ;
								data[1] = (int)(Pitch_I_coefficient * (RC.RC_channel[1]/1000)) ;
								data[2] = (int)(Pitch_D_coefficient * (RC.RC_channel[2]/1000));
								Set_Gain(_Pitch_Gain,data);
							}
						 break;
					case '#':
						 print_[Yaw_Gain_set]=(print_[Yaw_Gain_set]==0) ? 1:0;
						 if(print_[Yaw_Gain_set] == 0)
							{
								data[0] = (int)(Yaw_P_coefficient * (RC.HOV_THR*10)) ;
								data[1] = (int)(Yaw_I_coefficient * (RC.RC_TRIM*10)) ;
								data[2] = (int)(Yaw_D_coefficient * (RC.HOV_PIT*10)) ;
								Set_Gain(_Yaw_Gain,data);
							}				 
						 break;
							
					case '%':
						 print_[Altitude_Velocity_Gain_set]=(print_[Altitude_Velocity_Gain_set]==0) ? 1:0;									
						 if(print_[Altitude_Velocity_Gain_set] == 0)
							{
								data[0] = (int)(Altitude_Velocity_P_coefficient * (RC.HOV_THR*10)) ;
								data[1] = (int)(Altitude_Velocity_I_coefficient * (RC.RC_TRIM*10)) ;
								data[2] = (int)(Altitude_Velocity_D_coefficient * (RC.HOV_PIT*10)) ;
								Set_Gain(_Altitude_Velocity_Gain,data);
							}				 
						 break;	
					case '~':
						 print_[ORB_position_Gain_set]=(print_[ORB_position_Gain_set]==0) ? 1:0;									
						 if(print_[ORB_position_Gain_set] == 0)
							{
								data[0] = (int)(ORB_position_P_coefficient * (RC.HOV_THR*100)) ;
								data[1] = (int)(ORB_position_I_coefficient * (RC.HOV_PIT*100)) ;	////****////
								data[2] = (int)(ORB_position_D_coefficient * (RC.RC_TRIM*100)) ;
								Set_Gain(_ORB_position_Gain,data);
							}				 
						 break;
					case '}':	
						 print_[opti_x_set]=(print_[opti_x_set]==0) ? 1:0;									
						 if(print_[opti_x_set] == 0)
							{
								data[0] = (int)(opti_x_P_coefficient * (RC.HOV_THR*100)) ;
								data[1] = (int)(opti_x_I_coefficient * (RC.RC_TRIM*100)) ;
								data[2] = (int)(opti_x_D_coefficient * (RC.HOV_PIT*100)) ;
								Set_Gain(_opti_x_gain,data);
							}							  
						 break;
					case '{':	
						 print_[opti_y_set]=(print_[opti_y_set]==0) ? 1:0;									
						 if(print_[opti_y_set] == 0)
							{
								data[0] = (int)(opti_y_P_coefficient * (RC.HOV_THR*100)) ;
								data[1] = (int)(opti_y_I_coefficient * (RC.RC_TRIM*100)) ;
								data[2] = (int)(opti_y_D_coefficient * (RC.HOV_PIT*100)) ;
								Set_Gain(_opti_y_gain,data);
							}							  
						 break;
				
					default:
						 break;
				}
				
				data_r = 0;
		}
			
	//***************************************************************
	
	        if(print_[ACCEL]==1)	                       
			{
					print2pc("%d,%d,%d,",(int)Mpu.acc_x,(int)Mpu.acc_y,(int)Mpu.acc_z);
					station_print_enter = 1;
			}
	
	        if(print_[GYRO]==1)	 
			{
					print2pc("%d,%d,%d,",(int)Mpu.gyro_x_real,(int)Mpu.gyro_y_real,(int)Mpu.gyro_z_real);
					station_print_enter = 1;
			}
			
	        if(print_[GRAVITY]==1)	 
			{
					print2pc("%d,%d,%d",(int)Mpu.Gravity,(int)(Mpu.g_print*1000),(int)(Mahony.Accel_weight*100));
					station_print_enter = 1;
			}
			
	        if(print_[Euler_angles]==1)	 
			{
					print2pc("%d,%d,%d,",(int)(Mahony.Roll * 10),(int)(Mahony.Pitch * 10),(int)(Mahony.Yaw * 10));
					station_print_enter = 1;
			}
	        if(print_[ms_coefficient]==1)	 
			{
					print2pc("%d,%d,%d,%d,%d,%d",(int)MS.coefficient[1],(int)MS.coefficient[2],(int)MS.coefficient[3],(int)MS.coefficient[4],(int)MS.coefficient[5],(int)MS.coefficient[6]);
					station_print_enter = 1;
			}
	        if(print_[ms_detail]==1)	 
			{
					//print2pc("%d,%d,%d,%ld,%ld,%ld",MS.D1,MS.D2,(int)MS.TEMP,(long int)MS.OFF,(long int)MS.SENS,(long int)MS.P);
					print2pc("%d,%d,%0.2f,%0.2f,%0.3f,",(int)MS.TEMP,(int)MS.P,(MS.P0),(MS.T0),(MS.Height ));
					station_print_enter = 1;
			}
			
        	if(print_[Vel_Z]==1)	 
			{
	 		   	print2pc("%0.2f,%0.2f",(( MS.last_Height - MS.Height)/0.024f),Mahony.pure_acc_z);
					station_print_enter = 1;
			}
	        if(print_[RC_CHANNEL]==1)	 
			{
	 		    print2pc("%d,%d,%d,%d,%d,%d,%d,%d,",(int)RC.RC_channel[0],(int)RC.RC_channel[1],RC.RC_channel[2],RC.RC_channel[3],RC.RC_channel[4],RC.RC_channel[5],RC.RC_channel[6],RC.RC_channel[7]);
					station_print_enter = 1;
			}
        	if(print_[RC_ATITUDE]==1)	 
			{
	 		    print2pc("%d,%d,%d,%d,",(int)RC.Throttle,(int)(RC.Roll),(int)(RC.Pitch),(int)(RC.Yaw));
					station_print_enter = 1;
			}
	        if(print_[Altitude_Detail]==1)	 
			{
	 		    print2pc("%0.2f,",Ultra.point);
					station_print_enter = 1;
			}
				
	        if(print_[Roll_Gain]==1)	 
			{
//					data[0] = Roll.Kp*10;
//					data[1] = Roll.Ki*10;
//					data[2] = Roll.Kd*10;
				  Read_Gain(_Roll_Gain,data);
	 		    print2pc("%d,%d,%d",data[0],data[1],data[2]);
					station_print_enter = 1;
			}
	        if(print_[Pitch_Gain]==1)	 
			{
//			  	data[0] = Pitch.Kp*10;
//					data[1] = Pitch.Ki*10;
//					data[2] = Pitch.Kd*10;
				Read_Gain(_Pitch_Gain,data);
	 		    print2pc("%d,%d,%d",data[0],data[1],data[2]);
					station_print_enter = 1;
				
			}
	        if(print_[Yaw_Gain]==1)	 
			{
					data[0] = Yaw.Kp*10;	
					data[1] = Yaw.Ki*10;
					data[2] = Yaw.Kd*10;	
	 		    print2pc("%d,%d,%d",data[0],data[1],data[2]);
					station_print_enter = 1;
			}
					if(print_[Altitude_take_off_Gain]==1)	 
			{
					data[0] = Altitude_take_off.Kp*10;	
					data[1] =Altitude_take_off.Ki*10;	
					data[2] =Altitude_take_off.Kd*10;	
	 		    print2pc("%d,%d,%d",data[0],data[1],data[2]);
					station_print_enter = 1;
			}
          if(print_[Altitude_Velocity_Gain]==1)	 
			{
    				data[0] = Altitude_Velocity.Kp*10;	
    				data[1] = Altitude_Velocity.Ki*10;	
    				data[2] = Altitude_Velocity.Kd*10;	
		        print2pc("%d,%d,%d",data[0],data[1],data[2]);
					  station_print_enter = 1;
			}
			
					if(print_[opti_x_read]==1)	 
			{
        data[0] =   Velocity.Y.Kp*10;
				data[1] = Velocity.Y.Ki*10;
				data[2] = Velocity.Y.Kd*10;
		    print2pc("%d,%d,%d",data[0],data[1],data[2]);
				station_print_enter = 1;
			}
		if(print_[opti_y_read]==1)	 
			{
        data[0] = Velocity.Y.Kp*10;
				data[1] = Velocity.Y.Ki*10;
				data[2] = Velocity.Y.Kd*10;
	 		  print2pc("%d,%d,%d",data[0],data[1],data[2]);
			  station_print_enter = 1;
			}
		if(print_[ORB_position_Gain]==1)	 
			{
			  	data[0] = ORB_position.X.Kp*100;
					data[1] = ORB_position.X.Ki*100;
					data[2] = ORB_position.X.Kd*100;
	 		    print2pc("%d,%d,%d",data[0],data[1],data[2]);
					station_print_enter = 1;
				
			}
			
	      	if(print_[ORB_position_Gain_set] == 1)
							{
								data[0] = (int)(ORB_position_P_coefficient * (RC.HOV_THR*100)) ;
								data[1] = (int)(ORB_position_I_coefficient * (RC.HOV_PIT*100)) ;	////****////
								data[2] = (int)(ORB_position_D_coefficient * (RC.RC_TRIM*100)) ;
								 print2pc("%d,%d,%d",data[0],data[1],data[2]);
				       	station_print_enter = 1;
							}	
					if(print_[Altitude_Velocity_Gain_set]==1)	 
			{
					data[0] = (Altitude_Velocity_P_coefficient * RC.HOV_THR*10);
					data[1] = (Altitude_Velocity_I_coefficient * RC.RC_TRIM*10);
					data[2] = (Altitude_Velocity_D_coefficient * RC.HOV_PIT*10);
	        print2pc("%d,%d,%d",data[0],data[1],data[2]);
					station_print_enter = 1;
			}
	        if(print_[Roll_Gain_set]==1)	 
			{
					data[0] = (Roll_P_coefficient * RC.RC_channel[0]/1000);
					data[1] = (Roll_I_coefficient * RC.RC_channel[1]/1000);
					data[2] = (Roll_D_coefficient * RC.RC_channel[2]/1000);
	 		    print2pc("%d,%d,%d",data[0],data[1],data[2]);
					station_print_enter = 1;
			}
	        if(print_[Pitch_Gain_set]==1)	 
			{
					data[0] = (Pitch_P_coefficient * RC.RC_channel[0]/1000);
					data[1] = (Pitch_I_coefficient * RC.RC_channel[1]/1000);
					data[2] = (Pitch_D_coefficient * RC.RC_channel[2]/1000);
	 		    print2pc("%d,%d,%d",data[0],data[1],data[2]);
					station_print_enter = 1;
			}
	        if(print_[Yaw_Gain_set]==1)	 
			{
					data[0] = (Yaw_P_coefficient * RC.HOV_THR*10);
					data[1] = (Yaw_I_coefficient * RC.RC_TRIM*10);
					data[2] = (Yaw_D_coefficient * RC.HOV_PIT*10);
	 		    print2pc("%d,%d,%d",data[0],data[1],data[2]);
					station_print_enter = 1;
			}	

					if(print_[Altitude_take_off_Gain_set]==1)	 
			{
					data[0] = (Altitude_take_off_P_coefficient * RC.HOV_THR*10);
					data[1] = (Altitude_take_off_I_coefficient * RC.RC_TRIM*10);
					data[2] = (Altitude_take_off_D_coefficient * RC.HOV_PIT*10);
	 		    print2pc("%d,%d,%d",data[0],data[1],data[2]);
					station_print_enter = 1;
			}			
			
			
	if(print_[Roll_Control_signal]==1)	 
			{
					data[0] = Roll.setpoint * 10;
					data[1] = Roll.point * 10;
				  data[2] = Roll.err * 10;
				  data[3] = Roll.Out * 10;
	 		    print2pc("%d,%d,%d",data[0],data[1],data[2]);
					station_print_enter = 1;
			}
	if(print_[Pitch_Control_signal]==1)	 
			{
					data[0] = Pitch.setpoint * 10;
					data[1] = Pitch.point * 10;
				  data[2] = Pitch.err * 10;
	 		    print2pc("%d,%d,%d",data[0],data[1],data[2]);
					station_print_enter = 1;
			}
			
		if(print_[opti_x_signal]==1)	 
			{
				
					data[0] =(int)Roll.setpoint;
					data[1] = (int)(Velocity.X.point * 10);
				  data[2] = (int)(10*Velocity.X.Out_float);
					print2pc("%d,%d,%d",data[0],data[1],data[2]);
					station_print_enter = 1;
			}		
		if(print_[opti_y_signal]==1)	 
			{
					data[0] =(int)Pitch.setpoint;
					data[1] = (int)(Velocity.Y.point * 10);
				  data[2] = (int)(10*Velocity.Y.Out_float);
					print2pc("%d,%d,%d",data[0],data[1],data[2]);
					station_print_enter = 1;		
			}			
	if(print_[BAT]==1)	 
			{
				  print2pc("%0.2f,",(float)(Bat_/21.6f));
					station_print_enter = 1;
			}
	
	if(station_print_enter == 1)
		{
					print2pcs("\r");
					station_print_enter = 0;
		}				
	//****************************************************************Bat	
}

