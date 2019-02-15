#ifndef H_Main_H
#define H_Main_H

#include "stm32f4xx_hal.h"
#include "Define.h"
#include "stdio.h"
#include "mpu6050.h"
#include "Telemetri.h"


extern UART_HandleTypeDef huart1;
extern I2C_HandleTypeDef hi2c1;
extern TIM_HandleTypeDef htim2;
extern uint32_t Bat_;
extern UART_HandleTypeDef huart4;
extern int time;
void Led_Beat(char state,int count);
void Check_battery(int System_counter,int System_state);


#endif 
