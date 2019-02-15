#include "RC.h"
#include "NRF.h"
_RC RC;

void RC_Read(_RC* Rc,int State)
{
	char data[20];
	ch2int con;
	
	if(State%10 == 3)
	{
		data[19]=1;
		if( HAL_I2C_Master_Transmit(&Rc->I2C,30,(uint8_t*)&data[19],1,1) == HAL_OK)
		{
				HAL_I2C_Master_Receive(&Rc->I2C,30, (uint8_t *)&data[0],16,1);
				for(int i=0;i<8;i++)
					{
						con.byte[0] = data[i*2];
						con.byte[1] = data[i*2 + 1];
						Rc->RC_channel[i] = con.real;
					}
				RC_2_SetPoint(Rc);
				Rc->fail =0;
		}	
		else
				Rc->fail =1;
			
	}
	
}

void RC_2_SetPoint(_RC* Rc)
{	
	if( ( Rc->RC_channel[Throttle_channel] > (Rc->channel_offset[Throttle_channel] - RC_noise) ) && ( (Rc->RC_channel[Throttle_channel]) < (Rc->channel_offset[Throttle_channel] + Rc->channel_scale[Throttle_channel] + RC_noise ) ) )
				Rc->Throttle = Throttle_range*((float)(Rc->RC_channel[Throttle_channel]-Rc->channel_offset[Throttle_channel])/((float)Rc->channel_scale[Throttle_channel]));
	
	if( ( Rc->RC_channel[Roll_channel] > (Rc->channel_offset[Roll_channel]  - RC_noise) ) &&  ( (Rc->RC_channel[Roll_channel]) < (Rc->channel_offset[Roll_channel] + Rc->channel_scale[Roll_channel]+ RC_noise ) ) )
		{
				Rc->Roll=2*angle_range*(((float)(Rc->RC_channel[Roll_channel]-Rc->channel_offset[Roll_channel])/(float)Rc->channel_scale[Roll_channel])-0.5f);
				Rc->Roll+=Roll_offset;                                                               
		}
				
	if( ( Rc->RC_channel[Pitch_channel] > (Rc->channel_offset[Pitch_channel] - RC_noise)) && ( Rc->RC_channel[Pitch_channel] < (Rc->channel_offset[Pitch_channel] + Rc->channel_scale[Pitch_channel]+ RC_noise) ) )
		{
				Rc->Pitch= -2*angle_range*(((float)(Rc->RC_channel[Pitch_channel]-Rc->channel_offset[Pitch_channel])/(float)Rc->channel_scale[Pitch_channel])-0.5f);
				Rc->Pitch+=Pitch_offset;                  
		}
				
	if( (Rc->RC_channel[Yaw_channel] >Rc->channel_offset[Yaw_channel] - RC_noise) && (Rc->RC_channel[Yaw_channel] < Rc->channel_offset[Yaw_channel] + Rc->channel_scale[Yaw_channel]+ RC_noise))
        Rc->Yaw=angle_range*(((float)(Rc->RC_channel[Yaw_channel]-Rc->channel_offset[Yaw_channel])/(float)Rc->channel_scale[Yaw_channel])-0.5f);
	
//	if(Rc->RC_channel[HOV_PIT_channel] > Rc->channel_offset[HOV_PIT_channel]  - RC_noise && Rc->RC_channel[HOV_PIT_channel] < Rc->channel_offset[HOV_PIT_channel] + Rc->channel_scale[HOV_PIT_channel]+ RC_noise)    
//				Rc->THR_CUT=(((float)(Rc->RC_channel[HOV_PIT_channel]-Rc->channel_offset[HOV_PIT_channel])/(float)Rc->channel_scale[HOV_PIT_channel]));       
	
	if(Rc->RC_channel[RC_SW_channel] > Rc->channel_offset[RC_SW_channel]  - RC_noise && Rc->RC_channel[RC_SW_channel] < Rc->channel_offset[RC_SW_channel] + Rc->channel_scale[RC_SW_channel]+ RC_noise)
				Rc->RC_SW=(((float)(Rc->RC_channel[RC_SW_channel]-Rc->channel_offset[RC_SW_channel])/(float)Rc->channel_scale[RC_SW_channel]));
	
	if(Rc->RC_channel[HOV_THR_channel] > Rc->channel_offset[HOV_THR_channel]  - RC_noise && Rc->RC_channel[HOV_THR_channel] < Rc->channel_offset[HOV_THR_channel] + Rc->channel_scale[HOV_THR_channel]+ RC_noise)
				Rc->HOV_THR=((float)(Rc->RC_channel[HOV_THR_channel]-Rc->channel_offset[HOV_THR_channel])/(float)Rc->channel_scale[HOV_THR_channel]);
	
	if(Rc->RC_channel[RC_TRIM_channel] > Rc->channel_offset[RC_TRIM_channel]  - RC_noise && Rc->RC_channel[RC_TRIM_channel] < Rc->channel_offset[RC_TRIM_channel] + Rc->channel_scale[RC_TRIM_channel]+ RC_noise)
			  Rc->RC_TRIM=(((float)(Rc->RC_channel[RC_TRIM_channel]-Rc->channel_offset[RC_TRIM_channel])/(float)Rc->channel_scale[RC_TRIM_channel])); 
	
	if(Rc->RC_channel[HOV_PIT_channel] > Rc->channel_offset[HOV_PIT_channel]  - RC_noise && Rc->RC_channel[HOV_PIT_channel] < Rc->channel_offset[HOV_PIT_channel] + Rc->channel_scale[HOV_PIT_channel] + RC_noise)
				Rc->HOV_PIT=((float)(Rc->RC_channel[HOV_PIT_channel]-Rc->channel_offset[HOV_PIT_channel])/(float)Rc->channel_scale[HOV_PIT_channel]);
	
	
	Rc->RC_SW   = (Rc->RC_SW> 0.5f )? 1 : 0;    
	Rc->THR_CUT = (Rc->RC_TRIM > 0.5f)? 1 : 0;
	Rc->Yaw = (fabs(Rc->Yaw)> YawThreshold) ? Rc->Yaw  : 0;
	
	
  if(fabs(Rc->Yaw) > (angle_range/2))   
				Rc->Yaw=Rc->Yaw*fsign(((float)angle_range/2.0f));
	
	if(Rc->Throttle>Throttle_range)
				Rc->Throttle=Rc->last_Throttle;
  else if(Rc->Throttle<=0)    
				Rc->Throttle=0;
	
	if(fabs(Rc->Roll) > 2*angle_range) 
				Rc->Roll  = 2*angle_range*fsign(Rc->Roll);
	
	if(fabs(Rc->Pitch)>2*angle_range)          
				Rc->Pitch = 2*angle_range*fsign(Rc->Pitch);
	
	Rc->Throttle=Rc->last_Throttle+(DT / ( FILTER_RC + DT))* (Rc->Throttle-Rc->last_Throttle);
	
  Rc->Roll=Rc->last_Roll+ (DT / ( FILTER_RC + DT)) * (Rc->Roll-Rc->last_Roll);
	
  Rc->Pitch=Rc->last_Pitch+ (DT / ( FILTER_RC + DT)) * (Rc->Pitch-Rc->last_Pitch);
	
  Rc->Yaw = Rc->last_Yaw+ (DT / ( FILTER_RC + DT)) * (Rc->Yaw-Rc->last_Yaw);


	Rc->last_Throttle=Rc->Throttle;
  Rc->last_Roll=Rc->Roll;
  Rc->last_Pitch=Rc->Pitch;
  Rc->last_Yaw=Rc->Yaw;
}


void RC_Calib(_RC* Rc,char calib)
{
	char data[20];
	ch2int con;
	int channel_min[8];
	int channel_max[8];
	
	if(calib == 1)
	{		
		for(int i=0;i<8;i++)
		{
				channel_min[i]=500000;
				channel_max[i]=0;
		}

		Rc->init = 1;
		print2pcs("RC calib:(for End please send 's')");
		HAL_Delay(1500);
		while(Rc->init == 1)
		{

			data[19]=1;
			if( HAL_I2C_Master_Transmit(&Rc->I2C,30,(uint8_t*)&data[19],1,1) == HAL_OK)
			{
					HAL_I2C_Master_Receive(&Rc->I2C,30, (uint8_t *)&data[0],16,1);
					for(int i=0;i<8;i++)
					{
						con.byte[0] = data[i*2];
						con.byte[1] = data[i*2 + 1];
						Rc->RC_channel[i] = con.real;
						
						if(channel_max[i] < Rc->RC_channel[i])
							channel_max[i] = Rc->RC_channel[i];
						
						if(channel_min[i] > Rc->RC_channel[i])
							channel_min[i]  = Rc->RC_channel[i];
					}		
			}
			print2pc("%d,%d,%d,%d,%d,%d,%d,%d\r",Rc->RC_channel[0],Rc->RC_channel[1],Rc->RC_channel[2],Rc->RC_channel[3],Rc->RC_channel[4],Rc->RC_channel[5],Rc->RC_channel[6],Rc->RC_channel[7]);
			HAL_Delay(24);
			Nrf_(&NRF,7);
			LEDY_TGL;
	  }
		for(int i=0;i<8;i++)
		{
				Rc->channel_offset[i]=channel_min[i];
				Rc->channel_scale[i] =channel_max[i]-channel_min[i];  
		}
		print2pc("channel offset:%d,%d,%d,%d,%d,%d,%d,%d\r",Rc->channel_offset[0],Rc->channel_offset[1],Rc->channel_offset[2],Rc->channel_offset[3],Rc->channel_offset[4],Rc->channel_offset[5],Rc->channel_offset[6],Rc->channel_offset[7]);
		print2pc("channel scale :%d,%d,%d,%d,%d,%d,%d,%d\r",Rc->channel_scale[0],Rc->channel_scale[1],Rc->channel_scale[2],Rc->channel_scale[3],Rc->channel_scale[4],Rc->channel_scale[5],Rc->channel_scale[6],Rc->channel_scale[7]);
		
		//        EEPROM WRITE
		EEPROM_Write_int16_t(EEPROM_channel_offset_0,Rc->channel_offset[0]);
		EEPROM_Write_int16_t(EEPROM_channel_offset_1,Rc->channel_offset[1]);
		EEPROM_Write_int16_t(EEPROM_channel_offset_2,Rc->channel_offset[2]);
		EEPROM_Write_int16_t(EEPROM_channel_offset_3,Rc->channel_offset[3]);
		EEPROM_Write_int16_t(EEPROM_channel_offset_4,Rc->channel_offset[4]);
		EEPROM_Write_int16_t(EEPROM_channel_offset_5,Rc->channel_offset[5]);
		EEPROM_Write_int16_t(EEPROM_channel_offset_6,Rc->channel_offset[6]);
		EEPROM_Write_int16_t(EEPROM_channel_offset_7,Rc->channel_offset[7]);
		
		EEPROM_Write_int16_t(EEPROM_channel_scale_0,Rc->channel_scale[0]);
		EEPROM_Write_int16_t(EEPROM_channel_scale_1,Rc->channel_scale[1]);
		EEPROM_Write_int16_t(EEPROM_channel_scale_2,Rc->channel_scale[2]);
		EEPROM_Write_int16_t(EEPROM_channel_scale_3,Rc->channel_scale[3]);
		EEPROM_Write_int16_t(EEPROM_channel_scale_4,Rc->channel_scale[4]);
		EEPROM_Write_int16_t(EEPROM_channel_scale_5,Rc->channel_scale[5]);
		EEPROM_Write_int16_t(EEPROM_channel_scale_6,Rc->channel_scale[6]);
		EEPROM_Write_int16_t(EEPROM_channel_scale_7,Rc->channel_scale[7]);
			
	}
}	
	
void RC_Read_EEPROM(_RC* Rc)
{
	
	Rc->channel_offset[0] = EEPROM_Read_int16_t(EEPROM_channel_offset_0);
	Rc->channel_offset[1] = EEPROM_Read_int16_t(EEPROM_channel_offset_1);
	Rc->channel_offset[2] = EEPROM_Read_int16_t(EEPROM_channel_offset_2);
	Rc->channel_offset[3] = EEPROM_Read_int16_t(EEPROM_channel_offset_3);
	Rc->channel_offset[4] = EEPROM_Read_int16_t(EEPROM_channel_offset_4);
	Rc->channel_offset[5] = EEPROM_Read_int16_t(EEPROM_channel_offset_5);
	Rc->channel_offset[6] = EEPROM_Read_int16_t(EEPROM_channel_offset_6);
	Rc->channel_offset[7] = EEPROM_Read_int16_t(EEPROM_channel_offset_7);
	
	Rc->channel_scale[0]  = EEPROM_Read_int16_t(EEPROM_channel_scale_0);
	Rc->channel_scale[1]  = EEPROM_Read_int16_t(EEPROM_channel_scale_1);
	Rc->channel_scale[2]  = EEPROM_Read_int16_t(EEPROM_channel_scale_2);
	Rc->channel_scale[3]  = EEPROM_Read_int16_t(EEPROM_channel_scale_3);
	Rc->channel_scale[4]  = EEPROM_Read_int16_t(EEPROM_channel_scale_4);
	Rc->channel_scale[5]  = EEPROM_Read_int16_t(EEPROM_channel_scale_5);
	Rc->channel_scale[6]  = EEPROM_Read_int16_t(EEPROM_channel_scale_6);
	Rc->channel_scale[7]  = EEPROM_Read_int16_t(EEPROM_channel_scale_7);
	
	print2pc("RC:\rchannel offset:%d,%d,%d,%d,%d,%d,%d,%d\r",Rc->channel_offset[0],Rc->channel_offset[1],Rc->channel_offset[2],Rc->channel_offset[3],Rc->channel_offset[4],Rc->channel_offset[5],Rc->channel_offset[6],Rc->channel_offset[7]);
	print2pc("channel scale :%d,%d,%d,%d,%d,%d,%d,%d\r",Rc->channel_scale[0],Rc->channel_scale[1],Rc->channel_scale[2],Rc->channel_scale[3],Rc->channel_scale[4],Rc->channel_scale[5],Rc->channel_scale[6],Rc->channel_scale[7]);
		

}	

void RC_Init(_RC* Rc,I2C_HandleTypeDef hi2cx ,char calib)
{
	Rc->I2C = hi2cx;
	RC_Calib(Rc,calib);
	RC_Read_EEPROM(Rc);
}


float fsign(float x)
{
	if(x>0)
		return 1.0f;
	else if(x<0)
		return -1.0f;
	else return 0;
}
