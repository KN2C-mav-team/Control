#ifndef H_MPC_H
#define H_MPC_H

#define MPC_BUFF_AMOUNT  11


#include "stm32f4xx_hal.h"
#include "mpu6050.h"
#include "stdarg.h"
#include "main.h"

typedef struct {
	
	int data[32];
	char pack_started;
	char sum;
	char Len;
	char ready;
	ch2int conv;
	int j;
	int data_num;
	uint8_t data_send[32];// age bekhad beshkane do bodish kon :p
	uint8_t State;
	uint8_t Num;
	uint8_t Check_Sum;
	uint8_t Len_send;
	int UART_IRQ_FLAG;
	uint8_t MPC_UART_BUFF[MPC_BUFF_AMOUNT];
	
}_MPC;

extern _MPC MPC;

void Mpc_decode(_MPC* Mpc,uint8_t data);
void Mpc_Empty_Data(_MPC* Mpc);
void Mpc_Fill_Data(_MPC* Mpc , uint8_t num , ... );
void Mpc_Send_Data(_MPC* Mpc);
void Check_MPC_IRQ(_MPC* Mpc);




#endif 


