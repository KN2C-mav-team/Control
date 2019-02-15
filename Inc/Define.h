#ifndef H_Define_H
#define H_Define_H

#include "stm32f4xx_hal.h"



//#define Resque

#define	FALSE				 0
#define	TRUE				 1

#define LEDG_TGL     HAL_GPIO_TogglePin(GPIOE,GPIO_PIN_2)
#define LEDG_ON      HAL_GPIO_WritePin(GPIOE,GPIO_PIN_2,1)
#define LEDG_OFF     HAL_GPIO_WritePin(GPIOE,GPIO_PIN_2,0)



#define LEDB_TGL  	 HAL_GPIO_TogglePin(GPIOE,GPIO_PIN_15)
#define LEDB_ON 		 HAL_GPIO_WritePin(GPIOB,GPIO_PIN_15,1)
#define LEDB_OFF     HAL_GPIO_WritePin(GPIOB,GPIO_PIN_15,0)


#define LEDY_TGL 		 HAL_GPIO_TogglePin(GPIOA,GPIO_PIN_4)
#define LEDY_ON  		 HAL_GPIO_WritePin(GPIOA,GPIO_PIN_4,1)
#define LEDY_OFF     HAL_GPIO_WritePin(GPIOA,GPIO_PIN_4,0)

#define LEDW_TGL     HAL_GPIO_TogglePin(GPIOA,GPIO_PIN_11)
#define LEDW_ON      HAL_GPIO_WritePin(GPIOA,GPIO_PIN_11,1)
#define LEDW_OFF     HAL_GPIO_WritePin(GPIOA,GPIO_PIN_11,0)




#define DT_PULSE      5000
#define DT            0.005f
#define sampleFreq    250.0f
#define DT_PULSE_1ms  1000
#define GROUND_MODE   0
#define READY_2_FLY   1
#define FLY_MODE      2

#endif 

