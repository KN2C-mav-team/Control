#include "Height.h"
#include "IMU.h"
_Ultra Ultra;

void Read_Srf(TIM_HandleTypeDef _htim,_Ultra* ultra)
{
	
	uint16_t count=0;
	ultra->fail = ultra->fail + 1;
	
	if(ultra->State == 0)
	{	
		
		if(ultra->ready == 1)
		{
			if(ultra->Begine < ultra->End)
				ultra->real = (((float)(ultra->End - ultra->Begine) /200)* 3.3f); //200;
			else
				ultra->real = (((float)(ultra->End - ultra->Begine + 0xffff) /200)* 3.3f);
			
			if(ultra->real < Max_Ultra_Thr)
			{
				ultra->ready = 2;
				
				if(ultra->End > ultra->last_Time)
					ultra->Diff_Time = ((float)(ultra->End - ultra->last_Time)/1000)/1000;
				else
					ultra->Diff_Time = ((float)(ultra->End - ultra->last_Time + 0xffff )/1000)/1000;
				
				
				
				ultra->lpf_Gain  = (ultra->real - ultra->last_point)/ultra->Diff_Time;
				ultra->last_Time = ultra->End;
				
				
				ultra_filter_lpf(ultra);
						
			}
			else
				ultra->ready = 0;
			
			
		}	
		Ultra_Trig_ON;
		count = _htim.Instance->CNT ;	
		ultra->Begine = _htim.Instance->CNT ;	
		while(fabs((uint16_t)_htim.Instance->CNT - count)< 30);
		Ultra_Trig_OFF;
		ultra->State = 1;	
		ultra->fail = 0;

		
	}	
	
	if(ultra->State == 3)
		ultra->State = 0;
	
	if(ultra->fail >16)
		ultra->State = 0;
}



void  ultra_filter_lpf(_Ultra* ultra)
{
	float filter,f_cut;
  // (6,100) (0.1,250)  
	//ultra->lpf_Gain =ultra->vel;
	
	if(fabs(ultra->lpf_Gain) < 500)
			f_cut=6;
	else
	{
			f_cut = (800 - fabs(ultra->lpf_Gain))/84;
			if(f_cut < 0.1f)
				f_cut = 0.1f;
	}
	
	filter =1/(2*3.14f*f_cut);			
	ultra->point = ultra->last_point +(ultra->Diff_Time/(filter + ultra->Diff_Time))*(ultra->real-ultra->last_point);  
	
	ultra->last_diff = ultra->diff;				
	ultra->diff =(float)(ultra->point - ultra->last_point)/ultra->Diff_Time; 
	ultra->last_point= ultra->point;

}




float Vel_z(_Kalman1x1 *Kalman_state,_Ultra *ultra,float acc)
{
	if(ultra->ready == 2)
	{
		ultra->ready = 0;
		
		
		
		
		//ultra->last_real = ultra->real;
	}
	
	
	return 1.0;
}

