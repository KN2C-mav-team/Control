#include "main.h"
#include "MS5611.h"


MS5611 MS;
float P_sum=0,i=0;
float pk=1.0;
int r=5;   ///5
float q=0.05;  //0.05
float Kk=0;



void MS5611_Read_Prom(MS5611 *sen)
{
	int i;
	char data[2];
	for(i=0;i<7;i++)
	{		
		
		HAL_I2C_Mem_Read(&sen->I2C,ms5607_ADD,ms5607_CMD_PROM_RD + i*2,1,(uint8_t *)&data,2,1000);
		sen->coefficient[i] = data[0] * 256 + data[1];
		HAL_Delay(60);

	}
}



void MS5611_init(MS5611 *sen,I2C_HandleTypeDef *hi2cx)
{
	sen->I2C = *hi2cx;
	MS5611_Read_Prom(sen);
	MS5611_calib(sen);
}



void MS5611_calcute(MS5611 *sen)
{
	sen->dt = sen->D2 - sen->coefficient[5]*16*16;
	sen->TEMP = 2000 + (((float)sen->dt/1024)*((float)sen->coefficient[6]/1024))/8;
	sen->OFF = (sen->coefficient[2] *16384)  + ((sen->coefficient[4] * sen->dt))/(128*4);
	sen->SENS = sen->coefficient[1]*(32768/4) + (sen->coefficient[3]*sen->dt )/(256*4);
	sen->P  = ((sen->SENS/2048 ) * (sen->D1/1024) - sen->OFF)/8192;
	//sen->P=sen->last_P+ (DT / ( FILTER_P + DT)) * (sen->P - sen->last_P);
	//sen->last_P =sen->P;
	
}


void MS5611_calib(MS5611 *sen)
{
	
	float P_sum=0,T_sum=0,i=0;
	char data[3];
	
	for(i=0;i<50;i++)
		{
			data[0] = data[1] = data[2] = 0;
			HAL_I2C_Mem_Write(&sen->I2C,ms5607_ADD,ms5607_CMD_ADC_4096 + ms5607_CMD_ADC_D1,1,(uint8_t *)data,1,1000);
			HAL_Delay(10);
			HAL_I2C_Mem_Read(&sen->I2C,ms5607_ADD,ms5607_CMD_ADC_READ,1,(uint8_t *)data,3,1000);
			sen->D1 = data[0]*256*256 + data[1]*256+data[2];
			
			HAL_I2C_Mem_Write(&sen->I2C,ms5607_ADD,ms5607_CMD_ADC_4096 + ms5607_CMD_ADC_D2,1,(uint8_t *)data,1,1000);
			HAL_Delay(10);
			HAL_I2C_Mem_Read(&sen->I2C,ms5607_ADD,ms5607_CMD_ADC_READ,1,(uint8_t *)data,3,1000);
			sen->D2 = data[0]*256*256 + data[1]*256+data[2];
			
			MS5611_calcute(sen);
			//print2pc("%d\r",(int)sen->P);
			
			P_sum =P_sum + (sen->P);
			T_sum =T_sum + (sen->TEMP);
		}
		
		sen->P0 = (float)P_sum / 50;
		sen->T0 = (float)T_sum / (50*100)  + 273;
		
		sen->last_P = sen->P0;
		
		sen->last_D1 =sen->D1;
		sen->last_D2 =sen->D2;
	
}


void MS5611_Read(MS5611 *sen)
{
	char data[3];
	switch(sen->station)
	{
		case 0:			
			HAL_I2C_Mem_Write(&sen->I2C,ms5607_ADD,ms5607_CMD_ADC_4096 + ms5607_CMD_ADC_D1,1,(uint8_t *)data,1,1000);
		  sen->station+=1;
			sen->ready_f = 0;
			break;
		
		case 1:
			sen->station+=1;
			sen->ready_f = 0;
			break;
		case 2:
			sen->station+=1;
			break;
		case 3:
			HAL_I2C_Mem_Read(&sen->I2C,ms5607_ADD,ms5607_CMD_ADC_READ,1,(uint8_t *)data,3,1000);
			sen->D1 = data[0]*256*256 + data[1]*256+data[2];
			HAL_I2C_Mem_Write(&sen->I2C,ms5607_ADD,ms5607_CMD_ADC_4096 + ms5607_CMD_ADC_D2,1,(uint8_t *)data,1,1000);
			sen->station+=1;
			break;
		
		case 4:
		  sen->station+=1;
			break;
		
		case 5:
			sen->station+=1;
			break;
		
		case 6:
			HAL_I2C_Mem_Read(&sen->I2C,ms5607_ADD,ms5607_CMD_ADC_READ,1,(uint8_t *)data,3,1000);
			sen->D2 = data[0]*256*256 + data[1]*256+data[2];
			MS5611_calcute(sen);
		   sen->real_p = sen->P;
		   P_sum =P_sum + sen->P;
		   i++;
		   if(i>=1000){
                 sen->P_average=P_sum/1000;
                 i=0;
				         P_sum=0;
		          	 }				 
		  sen->P=sen->last_P+ ((6*DT) / ( FILTER_P + (6*DT))) * (sen->P- sen->last_P);
  		sen->last_P=sen->P;

			HAL_I2C_Mem_Write(&sen->I2C,ms5607_ADD,ms5607_CMD_ADC_4096 + ms5607_CMD_ADC_D1,1,(uint8_t *)data,1,1000);
			MS5611_2_Height(sen);
			sen->station = 1;
			sen->ready_f = 1;
			break;
			
		default:
			sen->station = 0;
			break;
	}
		  pk=pk + q;
			Kk=pk/(pk+r) ;
			sen->xk = sen->xk +(Kk * (sen->P - sen->xk));
			pk = (1-Kk)*pk;
			sen->P_kalman=sen->xk;
	
}


void MS5611_2_Height(MS5611 *sen)
{
	//sen->Height = (sen->T0 *(pow(((double)sen->P/(double)sen->P0),0.193)-1))/0.0065;
	//sen->Height = (sen->T0 *(powf((sen->P/sen->P0)*1000000.0f,0.1903f)*1000000/138.6117f - 100000))/650.0f;
	sen->last_Height = sen->Height;
	sen->Height = (float)(sen->T0*((pow(sen->P*10000.0f,0.1903)*10000/pow(sen->P0*10000.0f,0.1903))-10000))/65.0f;
	//print2pc("%f\r",sen->Height);
}




