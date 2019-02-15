#ifndef H_Tele_H
#define H_Tele_H


#define print2pcs(T)         UART_send_buffer(sprintf(UART_bufferf, T));
#define print2pc(T,...)      UART_send_buffer(sprintf(UART_bufferf, T, __VA_ARGS__));
	
#define		_PRINTSIZE				50

 
#include "mpu6050.h"
#include "Altitude.h"



extern char UART_bufferf[100];
extern uint64_t  counter;
extern int lock_time ;
extern char data_r;
extern uint8_t station_data;



void UART_send_buffer(int length);
void Station_Data_R(void);


#endif 
