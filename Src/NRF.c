#include "main.h"
#include "NRF.h"
#include "Control.h"


char print__[50];
NRF_BOARD NRF;

void Nrf_Init(NRF_BOARD* Nrf,I2C_HandleTypeDef hi2c,uint8_t Add)
{
	Nrf->I2C = hi2c;
	Nrf->Add = Add;
	Nrf_Empty_Data(Nrf);
}

void NrF_Fill_Data(NRF_BOARD* Nrf , uint8_t num , ... )
{
	va_list arguments; 	
	ch2int conv;
	va_start( arguments, num ); 
	
	for(int i=0;i < num;i++)
		{
			conv.real=(uint16_t)va_arg( arguments, int );
			Nrf->Check_Sum = Nrf->Check_Sum + (uint8_t)conv.byte[0] + (uint8_t)conv.byte[1]; 
			Nrf->data[Nrf->Len + 2] = conv.byte[0];
			Nrf->data[Nrf->Len + 2 + 1] = conv.byte[1];
			Nrf->Len = Nrf->Len + 2;
		}
		
	va_end ( arguments );
}

void Nrf_Empty_Data(NRF_BOARD* Nrf)
{
	for(int i=0;i<32;i++)
		Nrf->data[i]=0;
	Nrf->Num = 0;
	Nrf->Check_Sum = 0;
	Nrf->Len = 2;
	Nrf->data[0] = 0xff;
	Nrf->data[1] = 0xff;
	Nrf->data[2] = Nrf->Len;
	Nrf->data[3] = Nrf->Num;
}

void Nrf_Send_Data(NRF_BOARD* Nrf)
{
	Nrf_Tele(Nrf);
	Nrf->data[2] = Nrf->Len + 1;
	Nrf->data[Nrf->Len + 2] = ~(Nrf->Check_Sum) + 1;
	if(HAL_I2C_Master_Transmit(&Nrf->I2C,Nrf->Add,(uint8_t*)&Nrf->data,32,1) ==HAL_OK) 
	{
		HAL_I2C_Master_Receive(&Nrf->I2C,Nrf->Add, (uint8_t *)&Nrf->g_data,1,1);
		Nrf->fail =0;
	}
	else 
		Nrf->fail =1;
	Nrf_Empty_Data(Nrf);	
}

void Nrf_(NRF_BOARD* Nrf,int State)
{
	if(State%10 == 7)
		Nrf_Send_Data(Nrf);
	
}

void Nrf_Tele(NRF_BOARD* Nrf)
{
	int data[10];	
	switch(Nrf->g_data)
		{
			case 'A':
				 print__[ACCEL]=(print__[ACCEL]==0) ? 1:0;
				 break;
			case 'G':
				 print__[GYRO]=(print__[GYRO]==0) ? 1:0;
				 break;
			case 'g':
				 print__[GRAVITY]=(print__[GRAVITY]==0) ? 1:0;
				 break;
			case 'e':
				 print__[Euler_angles]=(print__[Euler_angles]==0) ? 1:0;
				 break;
			case 'C':
				 print__[ms_coefficient]=(print__[ms_coefficient]==0) ? 1:0;
				 break;
			case 'c':
				 print__[ms_detail]=(print__[ms_detail]==0) ? 1:0;
				 break;
			case 'z':
				 print__[Vel_Z]=(print__[Vel_Z]==0) ? 1:0;
				 break;
			case 'R':
				 print__[RC_CHANNEL]=(print__[RC_CHANNEL]==0) ? 1:0;
				 break;
			case 'r':
				 print__[RC_ATITUDE]=(print__[RC_ATITUDE]==0) ? 1:0;
				 break;
			case 't':
				 print__[Time]=(print__[Time]==0) ? 1:0;
				 break;
			case 'h':
				 print__[Altitude_Detail]=(print__[Altitude_Detail]==0) ? 1:0;
				 break;
			case '1':
				 print__[Roll_Gain]=(print__[Roll_Gain]==0) ? 1:0;
				 break;
			case '2':
				 print__[Pitch_Gain]=(print__[Pitch_Gain]==0) ? 1:0;
				 break;
			case '3':
				 print__[Yaw_Gain]=(print__[Yaw_Gain]==0) ? 1:0;
				 break;		
			case '5':
				 print__[Altitude_Velocity_Gain]=(print__[Altitude_Velocity_Gain]==0) ? 1:0;
				 break;	
			case '=':
				 print__[ORB_position_Gain]=(print__[ORB_position_Gain]==0) ? 1:0;
				 break;	
			case ']':
				 print__[opti_x_read]=(print__[opti_x_read]==0) ? 1:0;
				 break;	
			case '[':
				 print__[opti_y_read]=(print__[opti_y_read]==0) ? 1:0;
				 break;	
			case '<':
				 print__[Altitude_take_off_Gain]=(print__[Altitude_take_off_Gain]==0) ? 1:0;
				 break;	
			case '>':
				 print__[Altitude_take_off_Gain_set]=(print__[Altitude_take_off_Gain_set]==0) ? 1:0;
			 if(print__[Altitude_take_off_Gain_set] == 0)
					{
						data[0] = (int)(Altitude_take_off_P_coefficient * (RC.HOV_THR*10)) ;
						data[1] = (int)(Altitude_take_off_I_coefficient * (RC.RC_TRIM*10)) ;
						data[2] = (int)(Altitude_take_off_D_coefficient * (RC.HOV_PIT*10)) ;
						Set_Gain(_Altitude_take_off_gain,data);
					}				
				 break;	
			
			
			case '!':
				 print__[Roll_Gain_set]=(print__[Roll_Gain_set]==0) ? 1:0;				
				 if(print__[Roll_Gain_set] == 0)
					{
						data[0] = (int)(Roll_P_coefficient * (RC.HOV_THR*10)) ;
						data[1] = (int)(Roll_I_coefficient * (RC.RC_TRIM*10)) ;
						data[2] = (int)(Roll_D_coefficient * (RC.HOV_PIT*10)) ;
						Set_Gain(_Roll_Gain,data);
					}					
				 break;
			case '@':				 
				 print__[Pitch_Gain_set]=(print__[Pitch_Gain_set]==0) ? 1:0;
				 if(print__[Pitch_Gain_set] == 0)
					{
						data[0] = (int)(Pitch_P_coefficient * (RC.HOV_THR*10)) ;
						data[1] = (int)(Pitch_I_coefficient * (RC.RC_TRIM*10)) ;
						data[2] = (int)(Pitch_D_coefficient * (RC.HOV_PIT*10));
						Set_Gain(_Pitch_Gain,data);
					}					
				 break;
			case '#':	
				 print__[Yaw_Gain_set]=(print__[Yaw_Gain_set]==0) ? 1:0;									
				 if(print__[Yaw_Gain_set] == 0)
					{
						data[0] = (int)(Yaw_P_coefficient * (RC.HOV_THR*10)) ;
						data[1] = (int)(Yaw_I_coefficient * (RC.RC_TRIM*10)) ;
						data[2] = (int)(Yaw_D_coefficient * (RC.HOV_PIT*10)) ;
						Set_Gain(_Yaw_Gain,data);
					}							  			 
				 break;	
			case '%':
				 print__[Altitude_Velocity_Gain_set]=(print__[Altitude_Velocity_Gain_set]==0) ? 1:0;									
				 if(print__[Altitude_Velocity_Gain_set] == 0)
					{
						data[0] = (int)(Altitude_Velocity_P_coefficient * (RC.HOV_THR*10)) ;
						data[1] = (int)(Altitude_Velocity_I_coefficient * (RC.HOV_PIT*10)) ;
						data[2] = (int)(Altitude_Velocity_D_coefficient * (RC.RC_TRIM*10)) ;
						Set_Gain(_Altitude_Velocity_Gain,data);
					}				 
				 break;	
			case '+':
				 print__[ORB_position_Gain_set]=(print__[ORB_position_Gain_set]==0) ? 1:0;									
				 if(print__[ORB_position_Gain_set] == 0)
					{
						data[0] = (int)(ORB_position_P_coefficient * (RC.HOV_THR*1000)) ;
						data[1] = (int)(ORB_position_I_coefficient * (RC.RC_TRIM*1000)) ;	////****////
						data[2] = (int)(ORB_position_D_coefficient * (RC.HOV_PIT*1000)) ;
						Set_Gain(_ORB_position_Gain,data);
					}				 
				 break;
			case '}':	
				 print__[opti_x_set]=(print__[opti_x_set]==0) ? 1:0;									
				 if(print__[opti_x_set] == 0)
					{
						data[0] = (int)(opti_x_P_coefficient * (RC.HOV_THR*1000)) ;
						data[1] = (int)(opti_x_I_coefficient * (RC.RC_TRIM*1000)) ;
						data[2] = (int)(opti_x_D_coefficient * (RC.HOV_PIT*1000)) ;
						Set_Gain(_opti_x_gain,data);
					}							  
				 break;
			case '{':	
				 print__[opti_y_set]=(print__[opti_y_set]==0) ? 1:0;									
				 if(print__[opti_y_set] == 0)
					{
						data[0] = (int)(opti_y_P_coefficient * (RC.HOV_THR*1000)) ;
						data[1] = (int)(opti_y_I_coefficient * (RC.RC_TRIM*1000)) ;
						data[2] = (int)(opti_y_D_coefficient * (RC.HOV_PIT*1000)) ;
						Set_Gain(_opti_y_gain,data);
					}							  
				 break;
					
			case 'b':
				 print__[BAT]=(print__[BAT]==0) ? 1:0;
				 break;
			
			case '0':
				 print__[Roll_Control_signal]=(print__[Roll_Control_signal]==0) ? 1:0;
				 break;
			case '9':
				 print__[Pitch_Control_signal]=(print__[Pitch_Control_signal]==0) ? 1:0;
				 break;
			case '8':
				 print__[Yaw_Control_signal]=(print__[Yaw_Control_signal]==0) ? 1:0;
				 break;			
			case '6':
				 print__[Altitude_Velocity_Control_signal]=(print__[Altitude_Velocity_Control_signal]==0) ? 1:0;
				 break;
			case '-':
				 print__[Position_Control_signal]=(print__[Position_Control_signal]==0) ? 1:0;
				 break;		
			case 'p':
				 print__[PTAM_DATA]=(print__[PTAM_DATA]==0) ? 1:0;
				 break;	
			case 'o':
				 print__[Optical_Flow]=(print__[Optical_Flow]==0) ? 1:0;
				 break;	
			case 's':
					RC.init = 0;
				 break;
			case 'q':
				 print__[opti_y_signal]=(print__[opti_y_signal]==0) ? 1:0;
				 break;
			case 'w':
				 print__[opti_x_signal]=(print__[opti_x_signal]==0) ? 1:0;
				 break;
					
		}
		
		//***************************************************************
		
		if(print__[ACCEL]==1)	                       
			{
				NrF_Fill_Data(Nrf,3,(int)(Mpu.acc_x),(int)(Mpu.acc_y),(int)(Mpu.acc_z));
			}
		if(print__[GYRO]==1)	                       
			{
				NrF_Fill_Data(Nrf,3,(int)(ToDeg(Mpu.gyro_x)),(int)(ToDeg(Mpu.gyro_y)),(int)(ToDeg(Mpu.gyro_z)));
			}
		if(print__[GRAVITY]==1)	                       
			{
				NrF_Fill_Data(Nrf,2,(int)(Mpu.acc_z),(int)(Mahony.Earth_acc_z*1000));
			}
		if(print__[Euler_angles]==1)	 
			{
				NrF_Fill_Data(Nrf,3,(int)(Mahony.Roll * 10),(int)(Mahony.Pitch * 10),(int)(Mahony.Yaw * 10));
			}
		if(print__[RC_CHANNEL]==1)	 
			{	
 		    NrF_Fill_Data(Nrf,8,RC.RC_channel[0],RC.RC_channel[1],RC.RC_channel[2],RC.RC_channel[3],RC.RC_channel[4],RC.RC_channel[5],RC.RC_channel[6],RC.RC_channel[7]);
			}
		if(print__[RC_ATITUDE]==1)	 
			{
	 		    NrF_Fill_Data(Nrf,4,(int)RC.Throttle,(int)(RC.Roll*10),(int)(RC.Pitch*10),(int)(RC.Yaw*10));
			}
		if(print__[ms_detail]==1)	 
			{
	 		    NrF_Fill_Data(Nrf,1,(int)MS.P);
			}
		if(print__[Altitude_Detail]==1)	 
			{
	 		    NrF_Fill_Data(Nrf,1,(int)(Ultra.point));
			}
		if(print__[Roll_Gain]==1)	 
			{
					Read_Gain(_Roll_Gain,data);
//					data[0] = Roll.Kp*10;//(Roll_P_coefficient * RC.HOV_THR *10);
//					data[1] = Roll.Ki*10;//Roll_I_coefficient * RC.RC_TRIM *10;
//					data[2] = Roll.Kd*10;//Roll_D_coefficient * RC.HOV_PIT *10;
	 		    NrF_Fill_Data(Nrf,3,data[0],data[1],data[2]);
			}
		if(print__[Pitch_Gain]==1)	 
			{
					Read_Gain(_Pitch_Gain,data);
//			  	data[0] = Pitch.Kp*10;//(Roll_P_coefficient * RC.HOV_THR *10);
//					data[1] = Pitch.Ki*10;//Roll_I_coefficient * RC.RC_TRIM *10;
//					data[2] = Pitch.Kd*10;//Roll_D_coefficient * RC.HOV_PIT *10;
	 		    NrF_Fill_Data(Nrf,3,data[0],data[1],data[2]);
				
			}
		if(print__[Yaw_Gain]==1)	 
			{
					Read_Gain(_Yaw_Gain,data);
//					data[0] = Yaw.Kp*10;	//(Roll_P_coefficient * RC.HOV_THR *10);
//					data[1] = Yaw.Ki*10;	//Roll_I_coefficient * RC.RC_TRIM *10;
//					data[2] = Yaw.Kd*10;	//Roll_D_coefficient * RC.HOV_PIT *10;
	 		    NrF_Fill_Data(Nrf,3,data[0],data[1],data[2]);
			}
   	if(print__[Altitude_Velocity_Gain]==1)	 
			{
					Read_Gain(_Altitude_Velocity_Gain,data);
//					data[0] = Altitude_Velocity.Kp*10;	//(Roll_P_coefficient * RC.HOV_THR *10);
//					data[1] = Altitude_Velocity.Ki*10;	//Roll_I_coefficient * RC.RC_TRIM *10;
//					data[2] = Altitude_Velocity.Kd*10;	//Roll_D_coefficient * RC.HOV_PIT *10;
	 		    NrF_Fill_Data(Nrf,3,data[0],data[1],data[2]);
			}
			
		if(print__[ORB_position_Gain]==1)	 
			{
					Read_Gain(_ORB_position_Gain,data);
//					data[0] = Position.X.Kp*10;		//(Roll_P_coefficient * RC.HOV_THR *10);
//					data[1] = Position.X.Ki*10;		//Roll_I_coefficient * RC.RC_TRIM *10;
//					data[2] = Position.X.Kd*10;		//Roll_D_coefficient * RC.HOV_PIT *10;
	 		    NrF_Fill_Data(Nrf,3,data[0],data[1],data[2]);
			}

		if(print__[opti_x_read]==1)	 
			{
					Read_Gain(_opti_x_gain,data);
//					data[0] = Roll.Kp*10;//(Roll_P_coefficient * RC.HOV_THR *10);
//					data[1] = Roll.Ki*10;//Roll_I_coefficient * RC.RC_TRIM *10;
//					data[2] = Roll.Kd*10;//Roll_D_coefficient * RC.HOV_PIT *10;
	 		    NrF_Fill_Data(Nrf,3,data[0],data[1],data[2]);
			}
		if(print__[opti_y_read]==1)	 
			{
					Read_Gain(_opti_y_gain,data);
//					data[0] = Roll.Kp*10;//(Roll_P_coefficient * RC.HOV_THR *10);
//					data[1] = Roll.Ki*10;//Roll_I_coefficient * RC.RC_TRIM *10;
//					data[2] = Roll.Kd*10;//Roll_D_coefficient * RC.HOV_PIT *10;
	 		    NrF_Fill_Data(Nrf,3,data[0],data[1],data[2]);
			}
		if(print__[Roll_Gain_set]==1)	 
			{
					data[0] = (Roll_P_coefficient * RC.HOV_THR *10);
					data[1] = (Roll_I_coefficient * RC.RC_TRIM *10);
					data[2] = (Roll_D_coefficient * RC.HOV_PIT *10);
	 		    NrF_Fill_Data(Nrf,3,data[0],data[1],data[2]);
			}
		if(print__[Pitch_Gain_set]==1)	 
			{
					data[0] = (Pitch_P_coefficient * RC.HOV_THR*10);
					data[1] = (Pitch_I_coefficient * RC.RC_TRIM*10);
					data[2] = (Pitch_D_coefficient * RC.HOV_PIT*10);
	 		    NrF_Fill_Data(Nrf,3,data[0],data[1],data[2]);
			}
		if(print__[Yaw_Gain_set]==1)	 
			{
					data[0] = (Yaw_P_coefficient * RC.HOV_THR*10);
					data[1] = (Yaw_I_coefficient * RC.RC_TRIM*10);
					data[2] = (Yaw_D_coefficient * RC.HOV_PIT*10);
	 		    NrF_Fill_Data(Nrf,3,data[0],data[1],data[2]);
			}			
		if(print__[Altitude_Velocity_Gain_set]==1)	 
			{
					data[0] = (Altitude_Velocity_P_coefficient * RC.HOV_THR*10);
					data[1] = (Altitude_Velocity_I_coefficient * RC.HOV_PIT*10);
					data[2] = (Altitude_Velocity_D_coefficient * RC.RC_TRIM*10);
	 		    NrF_Fill_Data(Nrf,3,data[0],data[1],data[2]);
			}
		if(print__[Altitude_take_off_Gain_set]==1)	 
			{
					data[0] = (Altitude_take_off_P_coefficient * RC.HOV_THR*10);
					data[1] = (Altitude_take_off_I_coefficient * RC.RC_TRIM*10);
					data[2] = (Altitude_take_off_D_coefficient * RC.HOV_PIT*10);
	 		    NrF_Fill_Data(Nrf,3,data[0],data[1],data[2]);
			}
		if(print__[Altitude_take_off_Gain]==1)	 
			{
					Read_Gain(_Altitude_take_off_gain,data);
	 		    NrF_Fill_Data(Nrf,3,data[0],data[1],data[2]);
			}
		if(print__[ORB_position_Gain_set]==1)	 
			{
					data[0] = (ORB_position_P_coefficient * RC.HOV_THR*1000);
					data[1] = (ORB_position_I_coefficient * RC.RC_TRIM*1000);   ////****////
					data[2] = (ORB_position_D_coefficient * RC.HOV_PIT*1000);
	 		    NrF_Fill_Data(Nrf,3,data[0],data[1],data[2]);
			}
			
		if(print__[Roll_Control_signal]==1)	 
			{
//					data[0] = Roll.setpoint * 10;
//					data[1] = Roll.point * 10;
//				  data[2] = Roll.err * 10;
//				  data[3] = Roll.Out * 10;
					data[0] = MRU;
					data[1] = MLD;
				  data[2] = MRD;
				  data[3] = MLU;
	 		    NrF_Fill_Data(Nrf,4,data[0],data[1],data[2],data[3]);
			}
		if(print__[Pitch_Control_signal]==1)	 
			{
					data[0] = Pitch.setpoint * 10;
					data[1] = Pitch.point * 10;
				  data[2] = Pitch.err * 10;
	 		    NrF_Fill_Data(Nrf,3,data[0],data[1],data[2]);
			}
		if(print__[Yaw_Control_signal]==1)	 
			{
					data[0] = Yaw.setpoint * 10;
					data[1] = Yaw.point * 10;
				  data[2] = Yaw.err * 10;
	 		    NrF_Fill_Data(Nrf,3,data[0],data[1],data[2]);
			}
			
		if(print__[Altitude_Velocity_Control_signal]==1)	 		//**//
			{
					data[0] = Ultra.point;   
					data[1] = Altitude_Velocity.point;
				  data[2] = Altitude_Velocity.Out_float;
			  	data[3] = Altitude_Velocity.P_save;
			  	data[4] = Altitude_Velocity.I_save;
					data[5] = Altitude_Velocity.D_save;
										
				 NrF_Fill_Data(Nrf,6,data[0],data[1],data[2],data[3],data[4],data[5]);
			}

		if(print__[Position_Control_signal]==1)	 		//**//
			{  
					data[0] = ORB_position.X.point; 
					data[1] = ORB_position.Y.point;
					data[2] = ORB_position.X.Out_float*100;   
					data[3] = ORB_position.Y.Out_float*100;
										
	 		    NrF_Fill_Data(Nrf,4,data[0],data[1],data[2],data[3]);
			}
		if(print__[opti_x_set]==1)	 
			{
					data[0] = (opti_x_P_coefficient * RC.HOV_THR*1000);
					data[1] = (opti_x_I_coefficient * RC.RC_TRIM*1000);
					data[2] = (opti_x_D_coefficient * RC.HOV_PIT*1000);
	 		    NrF_Fill_Data(Nrf,3,data[0],data[1],data[2]);
			}		
		if(print__[opti_y_set]==1)	 
			{
					data[0] = (opti_y_P_coefficient * RC.HOV_THR*1000);
					data[1] = (opti_y_I_coefficient * RC.RC_TRIM*1000);
					data[2] = (opti_y_D_coefficient * RC.HOV_PIT*1000);
	 		    NrF_Fill_Data(Nrf,3,data[0],data[1],data[2]);
			}	
		if(print__[opti_x_signal]==1)	 
			{
	 		    NrF_Fill_Data(Nrf,6,(int)Roll.setpoint,(int)(Velocity.X.point * 10),(int)(Velocity.X.P_save* 100),(int)(Velocity.X.I_save* 100),(int)(Velocity.X.D_save* 100),(int)(10*Velocity.X.Out_float));
					//NrF_Fill_Data(Nrf,6,(int)(Roll.setpoint*100),(int)(Velocity.X.P_save* 100),(int)(Velocity.X.I_save* 100),(int)(Pitch.setpoint*100),(int)(100*Velocity.Y.P_save),(int)(100*Velocity.Y.I_save));
			}		
		if(print__[opti_y_signal]==1)	 
			{
	 		    NrF_Fill_Data(Nrf,6,(int)Pitch.setpoint,(int)(10*Velocity.Y.point),(int)(100*Velocity.Y.P_save),(int)(100*Velocity.Y.I_save),(int)(100*Velocity.Y.D_save),(int)(10*Velocity.Y.Out_float));
			}				
		if(print__[BAT]==1)	 
			{
				  NrF_Fill_Data(Nrf,1,(int)(10*(float)(Bat_/1.0f)));
			}
		if(print__[Vel_Z]==1)	 
			{
				  NrF_Fill_Data(Nrf,4,(int)(z_vel.state),(int)(Ultra.vel),(int)(Ultra.real),(int)(Mahony.Earth_acc_z*100));
			}
		if(print__[Time]==1)	 
			{
				  NrF_Fill_Data(Nrf,1,time);
			}
		if(print__[PTAM_DATA]==1)	 
			{
				 // NrF_Fill_Data(Nrf,5,(int)(10*Position.X.P_save),(int)(10*Position.X.I_save),(int)(10*Position.X.D_save),(int)(10*Position.X.Out_float),(int)Cam_Position.Modified_POS_Z);
			}
			if(print__[Optical_Flow]==1)	 
			{
				  //NrF_Fill_Data(Nrf,4,(int)(10*optical.Vel_X),(int)Ultra.point,(int)(optical.Gyro_X *10),(int)(optical.Gyro_X *10 - 10*optical.Vel_X));
				//	NrF_Fill_Data(Nrf,3,(int)(10*optical.Vel_),(int)(optical.Gyro_Y),(int)(optical.Gyro_X));
					//NrF_Fill_Data(Nrf,4,(int)(10*(optical.Vel_X)),(int)(10*(optical.Vel_X - (optical.Gyro_X/3))),(int)(10*(optical.Vel_Y)),(int)(10*(optical.Vel_Y - (optical.Gyro_Y/(-3)))));	
				  //NrF_Fill_Data(Nrf,6,(int)((optical.Vel_X)),(int)((optical.Vel_Y)),(int)((optical.Gyro_X)),(int)(optical.Gyro_Y),(int)optical.Vel_correction_X,(int)optical.Vel_correction_Y);	
					//NrF_Fill_Data(Nrf,2,(int)optical.Vel_correction_Y,(int)optical.Vel_Y);
					//NrF_Fill_Data(Nrf,4,(int)x_vel.state,(int)y_vel.state,(int)optical.real_Vel_X,(int)optical.real_Vel_Y);
				
		//		NrF_Fill_Data(Nrf,2,(int)(x_vel.state ),(int)(y_vel.state));
			//	NrF_Fill_Data(Nrf,4,(int)(x_vel.state * 10),(int)(y_vel.state* 10),(int)(optical.max_acc_x*10),(int)(optical.max_acc_y*10));
					//NrF_Fill_Data(Nrf,2,(int)y_vel.state,(int)optical.real_Vel_Y);
				
					//NrF_Fill_Data(Nrf,2,(int)y_vel.state,(int)optical.Vel_correction_Y);
			}
	
	
}
