#ifndef H_MS5611_H
#define H_MS5611_H

#include "stm32f4xx_hal.h"
#include "math.h"
#include "Telemetri.h"



//*************************** define ********************************
#define ms5607_ADD          0xee
#define ms5607_CMD_RESET    0x1E // ADC reset command
#define ms5607_CMD_ADC_READ 0x00 // ADC read command
#define ms5607_CMD_ADC_CONV 0x40 // ADC conversion command
#define ms5607_CMD_ADC_D1   0x00 // ADC D1 conversion
#define ms5607_CMD_ADC_D2   0x10 // ADC D2 conversion
#define ms5607_CMD_ADC_256  0x40 // ADC OSR=256
#define ms5607_CMD_ADC_512  0x42 // ADC OSR=512
#define ms5607_CMD_ADC_1024 0x44 // ADC OSR=1024
#define ms5607_CMD_ADC_2048 0x46 // ADC OSR=2048
#define ms5607_CMD_ADC_4096 0x48 // ADC OSR=4096
#define ms5607_CMD_PROM_RD  0xA0 // Prom read command



//                   filter
#define	F_CUT_P			   			2.5
#define FILTER_P          1/(2*PI*F_CUT_P		      	)

#define	F_CUT_D		   			  0.5 											//**//
#define FILTER_D          1/(2*PI*F_CUT_D		      	) //**//

typedef struct {
	
	
	int coefficient[7];
	int D1,D2;
	int last_D1,last_D2;
	uint32_t dt;
	char station;
	long int TEMP;
	long int OFF;
	long int SENS;
	long int P,real_p,P_average,last_P_average;
	long int P_kalman,xk;
	float P0;
	float T0;
	long int last_P;
	float Height,last_Height;
	char ready_f;
	
	I2C_HandleTypeDef I2C;	
}MS5611;



extern MS5611 MS;
//************************************************************************88


void MS5611_Read_Prom(MS5611 *sen);
void MS5611_init(MS5611 *sen,I2C_HandleTypeDef *hi2cx);
void MS5611_Read(MS5611 *sen);
void MS5611_calcute(MS5611 *sen);
void MS5611_calib(MS5611 *sen);
void MS5611_2_Height(MS5611 *sen);






#endif 


