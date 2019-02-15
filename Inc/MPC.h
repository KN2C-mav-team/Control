#ifndef H_MPC_H
#define H_MPC_H

#define MPC_BUFF_AMOUNT  25//2n+5
#define MPC_BUFF_AMOUNT_2 5//11 //15

// 0 :optx,1:opty,2:start ,3:windowx 4:win y ; 5:status , 6:scale , 7:turn back , 8: x_marker , 9: y:marker , 10:staus , 11:x orb point  , 12 : z orb point  , 13 : y  point , 14: yaw point ,15:  x orb setpoint ,16 :  z , orb setoint , 17 :  y setpoint ,18 : yaw setpoint ,19 : quad state  


#include "main.h"
#include "stm32f4xx_hal.h"
#include "mpu6050.h"
#include "stdarg.h"


///uint16_t MPC_BUFF_AMOUNT[];
typedef struct {
	
	int data[32];
	int correct_data[32];
	char pack_started;
	char new_pack_started;
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
	int UART_IRQ_FLAG_2;
	uint8_t MPC_UART_BUFF[MPC_BUFF_AMOUNT];
  uint8_t MPC_UART_BUFF_2[MPC_BUFF_AMOUNT];
	
	int flag_error;
	
}_MPC;

extern _MPC MPC;
extern _MPC MPC_2;

void Mpc_decode(_MPC* Mpc,uint8_t data);
void Mpc_Empty_Data(_MPC* Mpc);
void Mpc_Fill_Data(_MPC* Mpc , uint8_t num , ... );
void Mpc_Send_Data(_MPC* Mpc);
void Check_MPC_IRQ(_MPC* Mpc,_MPC* Mpc_2);




#endif 


