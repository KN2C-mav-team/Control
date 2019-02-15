#include "main.h"
#include "RC.h"
#include "Control.h"
_RC RC;
_SBus SBUS;

unsigned int SBUS_Channel_Data[18];
unsigned char	 SBUS_Failsafe_Active = 0;
unsigned char	 SBUS_Lost_Frame = 0;
unsigned char	 SBUS_Current_Channel = 0;
unsigned char	 SBUS_Current_Channel_Bit = 0;
unsigned char	 SBUS_Current_Packet_Bit = 0;
uint8_t	 SBUS_Packet_Data[25];
unsigned char	 SBUS_Packet_Position = 0;

int moj_rc_flag=1;

void RC_2_SetPoint(_RC* Rc)
{	
	char channel_err=0;
	
	if( ( Rc->RC_channel[Throttle_channel] > (Rc->channel_offset[Throttle_channel] - RC_noise) ) && ( (Rc->RC_channel[Throttle_channel]) < (Rc->channel_offset[Throttle_channel] + Rc->channel_scale[Throttle_channel] + RC_noise ) ) )
				Rc->Throttle = Throttle_range*((float)(Rc->RC_channel[Throttle_channel]-Rc->channel_offset[Throttle_channel])/((float)Rc->channel_scale[Throttle_channel]));
	else
		channel_err+=1;
	
	
	if( ( Rc->RC_channel[Roll_channel] > (Rc->channel_offset[Roll_channel]  - RC_noise) ) &&  ( (Rc->RC_channel[Roll_channel]) < (Rc->channel_offset[Roll_channel] + Rc->channel_scale[Roll_channel]+ RC_noise ) ) )
		{
				Rc->Roll=2*angle_range*(((float)(Rc->RC_channel[Roll_channel]-Rc->channel_offset[Roll_channel])/(float)Rc->channel_scale[Roll_channel])-0.5f);
				Rc->Roll+=Roll_offset;                                                               
		}
	else
		channel_err+=1;	
	
	
	if( ( Rc->RC_channel[Pitch_channel] > (Rc->channel_offset[Pitch_channel] - RC_noise)) && ( Rc->RC_channel[Pitch_channel] < (Rc->channel_offset[Pitch_channel] + Rc->channel_scale[Pitch_channel]+ RC_noise) ) )
		{
				Rc->Pitch= -2*angle_range*(((float)(Rc->RC_channel[Pitch_channel]-Rc->channel_offset[Pitch_channel])/(float)Rc->channel_scale[Pitch_channel])-0.5f);
				Rc->Pitch+=Pitch_offset;                  
		}
	else
		channel_err+=1;	
	
	
	if( (Rc->RC_channel[Yaw_channel] >Rc->channel_offset[Yaw_channel] - RC_noise) && (Rc->RC_channel[Yaw_channel] < Rc->channel_offset[Yaw_channel] + Rc->channel_scale[Yaw_channel]+ RC_noise))
        Rc->Yaw=angle_range*(((float)(Rc->RC_channel[Yaw_channel]-Rc->channel_offset[Yaw_channel])/(float)Rc->channel_scale[Yaw_channel])-0.5f);
	else
		channel_err+=1;
	
	
//	if(Rc->RC_channel[HOV_PIT_channel] > Rc->channel_offset[HOV_PIT_channel]  - RC_noise && Rc->RC_channel[HOV_PIT_channel] < Rc->channel_offset[HOV_PIT_channel] + Rc->channel_scale[HOV_PIT_channel]+ RC_noise)    
//				Rc->THR_CUT=(((float)(Rc->RC_channel[HOV_PIT_channel]-Rc->channel_offset[HOV_PIT_channel])/(float)Rc->channel_scale[HOV_PIT_channel]));       
	
	if(Rc->RC_channel[RC_SW_channel] > Rc->channel_offset[RC_SW_channel]  - RC_noise && Rc->RC_channel[RC_SW_channel] < Rc->channel_offset[RC_SW_channel] + Rc->channel_scale[RC_SW_channel]+ RC_noise)
				Rc->RC_SW=(((float)(Rc->RC_channel[RC_SW_channel]-Rc->channel_offset[RC_SW_channel])/(float)Rc->channel_scale[RC_SW_channel]));
	else
		channel_err+=1;
	
	
	if(Rc->RC_channel[HOV_THR_channel] > Rc->channel_offset[HOV_THR_channel]  - RC_noise && Rc->RC_channel[HOV_THR_channel] < Rc->channel_offset[HOV_THR_channel] + Rc->channel_scale[HOV_THR_channel]+ RC_noise)
				Rc->HOV_THR=((float)(Rc->RC_channel[HOV_THR_channel]-Rc->channel_offset[HOV_THR_channel])/(float)Rc->channel_scale[HOV_THR_channel]);
	else
		channel_err+=1;
	
	
	if(Rc->RC_channel[RC_TRIM_channel] > Rc->channel_offset[RC_TRIM_channel]  - RC_noise && Rc->RC_channel[RC_TRIM_channel] < Rc->channel_offset[RC_TRIM_channel] + Rc->channel_scale[RC_TRIM_channel]+ RC_noise)
			  Rc->RC_TRIM=(((float)(Rc->RC_channel[RC_TRIM_channel]-Rc->channel_offset[RC_TRIM_channel])/(float)Rc->channel_scale[RC_TRIM_channel])); 
	else
		channel_err+=1;
	
	
	if(Rc->RC_channel[HOV_PIT_channel] > Rc->channel_offset[HOV_PIT_channel]  - RC_noise && Rc->RC_channel[HOV_PIT_channel] < Rc->channel_offset[HOV_PIT_channel] + Rc->channel_scale[HOV_PIT_channel] + RC_noise)
				Rc->HOV_PIT=((float)(Rc->RC_channel[HOV_PIT_channel]-Rc->channel_offset[HOV_PIT_channel])/(float)Rc->channel_scale[HOV_PIT_channel]);
	else
		channel_err+=1;
	
	
	if(channel_err > 4)
		Rc->Invalid_Data_counter += 1;
	else
		Rc->Invalid_Data_counter =  0;
	
	
	
	if(Rc->Invalid_Data_counter > 4)
	{
		Rc->fail = RC_Invalid_Data;
		Rc->Invalid_Data_counter =5;
	}
	else
		Rc->fail =  0;
	
	
	
	Rc->RC_SW   = (Rc->RC_channel[3]> 1000 )? 1 : 0;    
	Rc->THR_CUT = (Rc->RC_channel[2]< 1000 )? 1 : 0;
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
		HAL_Delay(1500);
		while(Rc->init == 1)
		{
			recieve_sbus_radio(&SBUS,&RC);
			for(int i=0;i<8;i++)
					{		
						if(channel_max[i] < Rc->RC_channel[i])
							channel_max[i] = Rc->RC_channel[i];
						
						if(channel_min[i] > Rc->RC_channel[i])
							channel_min[i]  = Rc->RC_channel[i];
					}		
	  }
		for(int i=0;i<8;i++)
		{
				Rc->channel_offset[i]=channel_min[i];
				Rc->channel_scale[i] =channel_max[i]-channel_min[i];  
		}
		
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
	
	Rc->channel_offset[0] = 306;
	Rc->channel_offset[1] = 306;
	Rc->channel_offset[2] = 306;
	Rc->channel_offset[3] = 306;
	Rc->channel_offset[4] = 306;
	Rc->channel_offset[5] = 306;
	Rc->channel_offset[6] = 306;
	Rc->channel_offset[7] = 306;
	
	Rc->channel_scale[0]  = 1390;
	Rc->channel_scale[1]  = 1390;
	Rc->channel_scale[2]  = 1390;
	Rc->channel_scale[3]  = 1390;
	Rc->channel_scale[4]  = 1390;
	Rc->channel_scale[5]  = 1390;
	Rc->channel_scale[6]  = 1390;
	Rc->channel_scale[7]  = 1390;

}	

void RC_Init(_RC* Rc,char calib)
{
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




void recieve_sbus_radio(_SBus* sbus , _RC* Rc)
{
  if( sbus->flag==1){
		sbus->flag=0;

		for(int co=0;co<25 ;co++){
  switch (sbus->state)
  {
    case 0:
      if(sbus->buffer[co] == 0x0F)
        sbus->state++;
      else
        sbus->state=0;
      break;
    case 1:
      if(sbus->buffer[co] == 0x00 && sbus->m == 22)
		//if( sbus->m == 22)
      {
        sbus->state++;
        break;
      }
      else
      {
        sbus->data[sbus->m] = sbus->buffer[co];
        sbus->m++;
        sbus->state=1;
      }
      if(sbus->m > 22)
      {
        sbus->m = 0;
        sbus->state=0;
        break;
      }
      break;
    case 2:
      sbus->state=0;
      sbus->m=0;
	  	//sbus->ready_data =1;
		  sbus->channel[0] = (sbus->data[0]    | sbus->data[1]<<8)                         & 0x07FF; //roll
      sbus->channel[1] = (sbus->data[1]>>3 | sbus->data[2]<<5)                         & 0x07FF; //pitch
      sbus->channel[2] = (sbus->data[2]>>6 | sbus->data[3]<<2 | sbus->data[4]<<10)     & 0x07FF; //throtle
      sbus->channel[3] = (sbus->data[4]>>1 | sbus->data[5]<<7)                         & 0x07FF; //yaw
      sbus->channel[4] = (sbus->data[5]>>4 | sbus->data[6]<<4)                         & 0x07FF; //
      sbus->channel[5] = (sbus->data[6]>>7 | sbus->data[7]<<1 | sbus->data[8]<<9 )     & 0x07FF; //pot
      sbus->channel[6] = (sbus->data[8]>>2 | sbus->data[9]<<6)                         & 0x07FF; //pot 
      sbus->channel[7] = (sbus->data[9]>>5 | sbus->data[10]<<3)                        & 0x07FF;
	    sbus->channel[8] = (sbus->data[11]   | sbus->data[12]<<8)                        & 0x07FF;
			Rc->RC_channel[0]= sbus->channel[7];
			Rc->RC_channel[1]= sbus->channel[4];
			Rc->RC_channel[2]= sbus->channel[5];
			Rc->RC_channel[3]= sbus->channel[6];
			Rc->RC_channel[4]= sbus->channel[3];
			Rc->RC_channel[5]= sbus->channel[0];
			Rc->RC_channel[6]= sbus->channel[2];
			Rc->RC_channel[7]= sbus->channel[1];
		
		
			if(fabs(RC.RC_channel[6]-RC.last_RC_channel[6]>400))	RC.RC_channel[6]=RC.last_RC_channel[6];
			RC.last_RC_channel[6]=RC.RC_channel[6];
		
//		 Rc->RC_channel[0]= Rc->last_RC_channel[0] + (DT /(DT + FILTER_RC))*( Rc->RC_channel[0] - Rc->last_RC_channel[0]);
//	    Rc->last_RC_channel[0] =  Rc->RC_channel[0];
//			
//					 Rc->RC_channel[1]= Rc->last_RC_channel[1] + (DT /(DT + FILTER_RC))*( Rc->RC_channel[1] - Rc->last_RC_channel[1]);
//	    Rc->last_RC_channel[1] =  Rc->RC_channel[1];
//			
//					 Rc->RC_channel[2]= Rc->last_RC_channel[2] + (DT /(DT + (FILTER_RC*3)))*( Rc->RC_channel[2] - Rc->last_RC_channel[2]);
//	    Rc->last_RC_channel[2] =  Rc->RC_channel[2];
//			
//					 Rc->RC_channel[3]= Rc->last_RC_channel[3] + (DT /(DT + FILTER_RC))*( Rc->RC_channel[3] - Rc->last_RC_channel[3]);
//	    Rc->last_RC_channel[3] =  Rc->RC_channel[3];
//			
//					 Rc->RC_channel[4]= Rc->last_RC_channel[4] + (DT /(DT + FILTER_RC))*( Rc->RC_channel[4] - Rc->last_RC_channel[4]);
//	    Rc->last_RC_channel[4] =  Rc->RC_channel[4];
//			
//					 Rc->RC_channel[5]= Rc->last_RC_channel[5] + (DT /(DT + FILTER_RC))*( Rc->RC_channel[5] - Rc->last_RC_channel[5]);
//	    Rc->last_RC_channel[5] =  Rc->RC_channel[5];
//			
//					 Rc->RC_channel[6]= Rc->last_RC_channel[6] + (DT /(DT + FILTER_RC))*( Rc->RC_channel[6] - Rc->last_RC_channel[6]);
//	    Rc->last_RC_channel[6] =  Rc->RC_channel[6];
//			
//					 Rc->RC_channel[7]= Rc->last_RC_channel[7] + (DT /(DT + FILTER_RC))*( Rc->RC_channel[7] - Rc->last_RC_channel[7]);
//	    Rc->last_RC_channel[7] =  Rc->RC_channel[7];
			
	   	RC_2_SetPoint(Rc);
      break;
      
      
 }
}
	}}




void SBUS_Packet_fly_mode(){
	           	
				SBUS_Channel_Data[0] =  RC.RC_channel[5] + Velocity.Y.Out+window_detection.X.Out ;//roll
				SBUS_Channel_Data[1] =  RC.RC_channel[7] + Velocity.X.Out ; //pitch
				

				SBUS_Channel_Data[2] =  RC.RC_channel[6] ; //throtle
				SBUS_Channel_Data[3] =  RC.RC_channel[4] + YAW.Out  ; //yaw



				SBUS_Channel_Data[4] =  RC.RC_channel[1];
				SBUS_Channel_Data[5] =  RC.RC_channel[2]; //sw
				SBUS_Channel_Data[6] =  RC.RC_channel[3];
				//SBUS_Channel_Data[7] =  RC.RC_channel[0];  //thr
			//  SBUS_Channel_Data[8] = 0;//RC.RC_channel[7];

				SBUS_Failsafe_Active = 0;
				SBUS_Lost_Frame = 1;
				
				for(SBUS_Packet_Position = 0; SBUS_Packet_Position < 25; SBUS_Packet_Position++) SBUS_Packet_Data[SBUS_Packet_Position] = 0x00;  //Zero out packet data
				
				SBUS_Current_Packet_Bit = 0;
				SBUS_Packet_Position = 0;
				SBUS_Packet_Data[SBUS_Packet_Position] = 0x0F;  //Start Byte
				SBUS_Packet_Position++;

				for(SBUS_Current_Channel = 0; SBUS_Current_Channel < 16; SBUS_Current_Channel++)
				{
					for(SBUS_Current_Channel_Bit = 0; SBUS_Current_Channel_Bit < 11; SBUS_Current_Channel_Bit++)
					{
						if(SBUS_Current_Packet_Bit > 7)
						{
							SBUS_Current_Packet_Bit = 0;  //If we just set bit 7 in a previous step, reset the packet bit to 0 and
							SBUS_Packet_Position++;       //Move to the next packet byte
						}
						SBUS_Packet_Data[SBUS_Packet_Position] |= (((SBUS_Channel_Data[SBUS_Current_Channel]>>SBUS_Current_Channel_Bit) & 0x01)<<SBUS_Current_Packet_Bit);  //Downshift the channel data bit, then upshift it to set the packet data byte
						SBUS_Current_Packet_Bit++;
					}
				}
				if(SBUS_Channel_Data[16] > 1023) SBUS_Packet_Data[23] |= (1<<0);  //Any number above 1023 will set the digital servo bit
				if(SBUS_Channel_Data[17] > 1023) SBUS_Packet_Data[23] |= (1<<1);
				if(SBUS_Lost_Frame != 0) SBUS_Packet_Data[23] |= (1<<2);          //Any number above 0 will set the lost frame and failsafe bits
				if(SBUS_Failsafe_Active != 0) SBUS_Packet_Data[23] |= (1<<3);
				SBUS_Packet_Data[24] = 0x00;  //End byte
				
				HAL_UART_Transmit(&huart3,(uint8_t*)SBUS_Packet_Data, 25 , 1);


}