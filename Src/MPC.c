
#include "main.h"
#include "MPC.h"

int moj_interupt_usart_rc=0;

_MPC MPC;
_MPC MPC_2;

void Mpc_decode(_MPC* Mpc,uint8_t data)
{
	if( Mpc->new_pack_started >5)  {Mpc->new_pack_started =0;}
	if(Mpc->MPC_UART_BUFF[0]!=255 || Mpc->MPC_UART_BUFF[1]!=255 || Mpc->MPC_UART_BUFF[3]!=0){
		HAL_UART_DMAStop(&huart2);
		
		HAL_UART_Receive_DMA(&huart2,MPC.MPC_UART_BUFF,MPC_BUFF_AMOUNT);//optical
		}
	switch(Mpc->new_pack_started)
				{
						
					case 0:
						if(data == 0xff) {Mpc->new_pack_started = 1;
								
						}
							                
						else             Mpc->new_pack_started = 0;
						break;
						
					case 1:
						if(data == 0xff) Mpc->new_pack_started = 2;
						else             Mpc->new_pack_started = 0;
						break;
						
					case 2:
						Mpc->Len =data - 3;
						Mpc->new_pack_started = 3;
						break;
						
					case 3:
						if(data == 0)
							Mpc->new_pack_started = 4;
						else
							Mpc->new_pack_started = 0;
						break;
					case 4:
						
						if(Mpc->Len == 0)
						{
							
							Mpc->sum = Mpc->sum + (uint8_t)data;
							if(Mpc->sum == 0)
							{
								
				    		Mpc->flag_error++;
								Mpc->sum=0;
								Mpc->new_pack_started =0;
								Mpc->ready =1;		
								Mpc->data_num = Mpc->j;
								Mpc->j=0;
							}
							else
							{
								Mpc->j=0;
								Mpc->sum=0;
								Mpc->new_pack_started =0;
							}
							
						}
						else
						{
							Mpc->sum = Mpc->sum + (uint8_t)data;
							Mpc->conv.byte[0] = data;
							Mpc->new_pack_started =5;
						}
						break;
					case 5:
							Mpc->sum = Mpc->sum + (uint8_t)data;
							Mpc->conv.byte[1] = data;
							Mpc->new_pack_started =4;
							Mpc->data[Mpc->j] = Mpc->conv.real;
							Mpc->Len = Mpc->Len - 2;
							Mpc->j++;				
						break;
					
						
				}
}

void Mpc_Empty_Data(_MPC* Mpc)
{
	for(int i=0;i<32;i++)
		Mpc->data_send[i]=0;
	Mpc->Num = 0;
	Mpc->Check_Sum = 0;
	Mpc->Len_send = 2;
	Mpc->data_send[0] = 0xff;
	Mpc->data_send[1] = 0xff;
	Mpc->data_send[2] = Mpc->Len_send;
	Mpc->data_send[3] = 0x00;//Mpc->Num;
}

void Mpc_Fill_Data(_MPC* Mpc , uint8_t num , ... )
{
	va_list arguments; 	
	ch2int conv;
	va_start( arguments, num ); 
	
	for(int i=0;i < num;i++)
		{
			conv.real=(uint16_t)va_arg( arguments, int );
			Mpc->Check_Sum = Mpc->Check_Sum + (uint8_t)conv.byte[0] + (uint8_t)conv.byte[1]; 
			Mpc->data_send[Mpc->Len_send + 2] = conv.byte[0];
			Mpc->data_send[Mpc->Len_send + 2 + 1] = conv.byte[1];
			Mpc->Len_send = Mpc->Len_send + 2;
		}
		
	va_end ( arguments );
}

void Mpc_Send_Data(_MPC* Mpc)
{
	Mpc->data_send[2] = Mpc->Len_send + 1;
	Mpc->data_send[Mpc->Len_send + 2] = ~(Mpc->Check_Sum) + 1;
	HAL_UART_Transmit(&huart1,(uint8_t*)Mpc->data_send, Mpc->Len_send + 3 , 1);
	Mpc_Empty_Data(Mpc);	
}




void Check_MPC_IRQ(_MPC* Mpc,_MPC* Mpc_2)
{
	if(Mpc->UART_IRQ_FLAG)
		{
						if(!Mpc->ready)
			{
				HAL_NVIC_DisableIRQ(DMA1_Stream5_IRQn);
				int co = 0;
				for(co=0;co<MPC_BUFF_AMOUNT;co++)
				{
					moj_interupt_usart_rc++;
					// decode_mavlink_packet(Mpc->MPC_UART_BUFF[co],Mpc);	
           Mpc_decode(Mpc,Mpc->MPC_UART_BUFF[co]);					
				}
				
				 HAL_NVIC_EnableIRQ(DMA1_Stream5_IRQn);
				
			}
			  Mpc->UART_IRQ_FLAG = 0;
			
		}
	 if(Mpc_2->UART_IRQ_FLAG)
		{		
					if(!Mpc_2->ready)
			{
				int co = 0;
				HAL_NVIC_DisableIRQ(DMA1_Stream2_IRQn);
				for(co=0;co<MPC_BUFF_AMOUNT_2;co++)
				{
							
					 //decode_mavlink_packet_2(Mpc->MPC_UART_BUFF_2[co],Mpc);		
            Mpc_decode(Mpc_2,Mpc_2->MPC_UART_BUFF_2[co]);					
			}	
				HAL_NVIC_EnableIRQ(DMA1_Stream2_IRQn);
		}				
			  Mpc_2->UART_IRQ_FLAG = 0;
		}
}

