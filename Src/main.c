/*
  ******************************************************************************
  * File Name          : main.c
  * Date               : 30/03/2015 15:47:55
  * Description        : Main program body
  ******************************************************************************
  *
  * COPYRIGHT(c) 2015 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

/* USER CODE BEGIN Includes */

#include "main.h"
#include "IMU.h"
#include "MS5611.h"
#include "EEPROM.h"
#include "RC.h"
#include "NRF.h"
#include "Control.h"
#include "Altitude.h"
#include "MPC.h"
#include "Odometery.h"
//#include "Optical_Flow.h"
#include "par_optical.h"
#include <mavlink/common/mavlink.h>
#include "stm32f4xx_it.h"


/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/

ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

I2C_HandleTypeDef hi2c1;
I2C_HandleTypeDef hi2c3;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim14;
TIM_HandleTypeDef htim13;

UART_HandleTypeDef huart4;
UART_HandleTypeDef huart3;
UART_HandleTypeDef huart2;
UART_HandleTypeDef huart1;

DMA_HandleTypeDef hdma_uart4_rx;
DMA_HandleTypeDef hdma_usart3_rx;
//DMA_HandleTypeDef hdma_usart2_rx;
DMA_HandleTypeDef hdma_usart1_rx;

/* USER CODE BEGIN PV */

#define BATT_Coef   350.0f 
#define ADDRESS (0xE0) 
#define REGISTER_ADDRESS (0x00) 
#define H_BYTE (0x02) 
#define L_BYTE (0x03) 
#define SET_CM (0x51) //cm


int Led_Level=0;
uint8_t data1 = 0; 
uint8_t data2 = 0; 
void Ping_RC_IMU(MPU_SENSOR *sen,_RC *Rc);
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM3_Init(void);
static void MX_UART4_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_USART1_UART_Init(void);

/* USER CODE BEGIN PFP */
void M_I2C_init(I2C_HandleTypeDef *hi2c,I2C_TypeDef* I2Cx, uint32_t clockSpeed);
HAL_StatusTypeDef M_I2C_Init(I2C_HandleTypeDef *hi2c);
void M_I2C_init_(I2C_HandleTypeDef *hi2c,I2C_TypeDef* I2Cx, uint32_t clockSpeed);
HAL_StatusTypeDef M_I2C_Init_(I2C_HandleTypeDef *hi2c);
static void MX_TIM14_Init(void);
void MX_TIM13_Init(void);
/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

uint32_t Bat_=0;	
int time=0,time_err=0,err_time=0;
int omid;
int counter_mpc_usart;
int moj_counter_usart_intrupt=0;
double clock_time=0;




int decode_flag_mavlink_usart=0;



int counter_srf02=0;
// begin : sbus variables
unsigned int SBUS_Channel_Data[18];
unsigned int ULTRA_SRF02;
unsigned char	 SBUS_Failsafe_Active = 0;
unsigned char	 SBUS_Lost_Frame = 0;
unsigned char	 SBUS_Current_Channel = 0;
unsigned char	 SBUS_Current_Channel_Bit = 0;
unsigned char	 SBUS_Current_Packet_Bit = 0;
uint8_t	 SBUS_Packet_Data[25];
unsigned char	 SBUS_Packet_Position = 0;
ch2int con;
int16_t mpc_data[2];
int16_t mpc_data_2[2];
uint8_t quality;
uint8_t buff_usart[34];
   uint8_t chert[34];
	 
// end : sbus variables

/* USER CODE END 0 */

int main(void)
{

  /* USER CODE BEGIN 1 */
	
	char Run_State=GROUND_MODE;
	char Run_Control =0;
	uint32_t temp;

//	uint8_t data_[20];
	int off_delay_timer=0;
	
  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* Configure the system clock */
  SystemClock_Config();

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
	
  MX_ADC1_Init();
  //MX_TIM1_Init();
  MX_TIM3_Init();
	M_I2C_init_(&hi2c1,I2C1,400000);
	M_I2C_init(&hi2c3,I2C3,400000);   
  MX_UART4_Init();
	MX_USART3_UART_Init();
	MX_USART2_UART_Init();
  MX_USART1_UART_Init();

  /* USER CODE BEGIN 2 */

	MX_TIM14_Init();
	MX_TIM13_Init();

	HAL_TIM_Base_Start_IT(&htim3);
	HAL_TIM_OC_Start_IT(&htim3,TIM_CHANNEL_1);
	//HAL_UART_Receive_DMA(&huart4,&station_data,1);
	//				__HAL_UART_FLUSH_DRREGISTER(&huart2);
	
	HAL_UART_Receive_DMA(&huart3,SBUS.buffer,25);
	HAL_UART_Receive_DMA(&huart1,MPC_2.MPC_UART_BUFF_2,MPC_BUFF_AMOUNT_2); //window 

	HAL_UART_Receive_DMA(&huart4,MPC.MPC_UART_BUFF,MPC_BUFF_AMOUNT);//optical
	HAL_ADC_Start_DMA(&hadc1,&Bat_,1);
	

	
  init_mpu(&Mpu,&hi2c3,0xD0,0);  //
//	MS5611_init(&MS,&hi2c3); // Alireza: barometer dont use
	RC_Init(&RC,0);
//	Pwm_frq(&htim1,50,500);

	HAL_TIM_Base_Start(&htim14);
	HAL_TIM_PWM_Start(&htim14,TIM_CHANNEL_1);

	
	HAL_TIM_Base_Start(&htim13);
	
	Mahony.q0=1.0f;
	z_vel.state=0;
	z_vel.C=1;
	z_vel.A=1;
	z_vel.B=DT;
	z_vel.P=1;
	z_vel.need_2_res =0;
	
  orb_pos.state=0;
	orb_pos.C=1;
	orb_pos.A=1;
	orb_pos.B=DT;
	orb_pos.P=1;
	orb_pos.need_2_res =0;
	
	MPC.ready = 0;
	MPC_2.ready=0;
	MPU_Hist.counter = 0;
	SBUS.state=0;
	SBUS.m=0;
	Cam_Kalman_init();
	
	Cam_Position.Scale_state = 0;
	Cam_Position.Modified_POS_X = 0;
	Cam_Position.Modified_POS_Y = 0;
	Cam_Position.Modified_POS_Z = 0;

 

  /* USER CODE END 2 */
  /* USER CODE BEGIN 3 */
  /* Infinite loop */
	
  while (1)
  {
		
		  clock_time++;
			__HAL_TIM_SetCounter(&htim13,0);
		
			temp=htim3.Instance->CNT;
			counter++;					
			lock_time  = 1;
					 
//      Check_battery((counter%100),Run_State);						
			  Update_IMU_Data_Mahony(&Mahony,&Mpu);							
				read_srf02();	
        Ultra.point=ULTRA_SRF02;	
        Vel_z(&z_vel,&Ultra,0);				
//    	Led_Beat(Run_State,counter);

        recieve_sbus_radio(&SBUS,&RC);		   
        ORB_get_data(&MPC,&MPC_2,&ORB_Position,&Mpu,&optical_par);                         
			  do_optical_par(&MPC,&MPC_2,&Mpu,&optical_par);							
				Rc2Controller(RC);			
        Point2Controller(Mahony,Mpu);		
		
		//	if(counter%50 == 0 ){ 
		//	encode_mavlink_packet();
	//	}		
//			   Mpc_Empty_Data(&MPC);						
//    		 Mpc_Fill_Data(&MPC,3,10,10,10);
//  			 Mpc_Send_Data(&MPC);	
         Check_MPC_IRQ(&MPC,&MPC_2)	;		
			

		//	if(counter%40 == 0 ) Station_Data_R();   
			
					switch(Run_State)
	
					{
						case GROUND_MODE:
							// SBUS_Packet_ground_mode();
						  SBUS_Packet_fly_mode();		
							if(Quad_On(RC) == 1) {
								Run_State=FLY_MODE;							
								control_init_(); 
								Set_zero_system_state();
								Run_Control = 0;
							}

							break;
						//*************************************************************************************
						case FLY_MODE:
				     if(Quad_Off(RC) == 1) Run_State=GROUND_MODE; 
							Velocity_Control(&Velocity,&Roll,&Pitch);																													
						//	Control_Altitude_Velocity(RC.RC_SW);
						 ORB_Position_control(&ORB_position,&MPC,&Roll,&Pitch); 	
            		
             SBUS_Packet_fly_mode();					
							break;
						//*************************************************************************************
						default:
							Run_State = GROUND_MODE;
							break;
					}
				
					
					
					if(htim3.Instance->CNT > temp)
						time = htim3.Instance->CNT-temp;
					else
						time = ( htim3.Instance->CNT - temp) +0xffff;
					
					if(time>=4500)	
					{
						time_err++;		
						err_time=time;
					}
					
//					while(__HAL_TIM_GetCounter(&htim13) < DT_PULSE)
//					{
//						omid = __HAL_TIM_GetCounter(&htim13);
//					}
					
            counter_mpc_usart= htim3.Instance->CNT;
  }
  /* USER CODE END 3 */

}
/** System Clock Configuration
*/
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;

  __PWR_CLK_ENABLE();

  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = 6;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  HAL_RCC_OscConfig(&RCC_OscInitStruct);

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_SYSCLK|RCC_CLOCKTYPE_PCLK1
                              |RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV4;
  HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5);

}

/* ADC1 init function */
void MX_ADC1_Init(void)
{

  ADC_ChannelConfTypeDef sConfig;

    /**Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion) 
    */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCKPRESCALER_PCLK_DIV2;
  hadc1.Init.Resolution = ADC_RESOLUTION12b;
  hadc1.Init.ScanConvMode = DISABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = ENABLE;
  hadc1.Init.EOCSelection = EOC_SINGLE_CONV;
  HAL_ADC_Init(&hadc1);

    /**Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time. 
    */
  sConfig.Channel = ADC_CHANNEL_9;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  HAL_ADC_ConfigChannel(&hadc1, &sConfig);

}

/* I2C1 init function */
void MX_I2C3_Init(void)
{

  hi2c3.Instance = I2C3;
  hi2c3.Init.ClockSpeed = 400000;
  hi2c3.Init.DutyCycle = I2C_DUTYCYCLE_16_9;
  hi2c3.Init.OwnAddress1 = 0;
  hi2c3.Init.AddressingMode = I2C_ADDRESSINGMODE_10BIT;
  hi2c3.Init.DualAddressMode = I2C_DUALADDRESS_DISABLED;
  hi2c3.Init.OwnAddress2 = 0;
  hi2c3.Init.GeneralCallMode = I2C_GENERALCALL_DISABLED;
  hi2c3.Init.NoStretchMode = I2C_NOSTRETCH_DISABLED;
  HAL_I2C_Init(&hi2c3);

}

/* TIM2 init function */
void MX_TIM1_Init(void)
{

  TIM_ClockConfigTypeDef sClockSourceConfig;
  TIM_MasterConfigTypeDef sMasterConfig;
  TIM_OC_InitTypeDef sConfigOC;

  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 0;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  HAL_TIM_Base_Init(&htim1);

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig);

  HAL_TIM_PWM_Init(&htim1);

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_OC1;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  HAL_TIMEx_MasterConfigSynchronization(&htim1 , &sMasterConfig);

  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;

  HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2);

  HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_3);

  HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_4);

}

/* TIM3 init function */
void MX_TIM3_Init(void)
{

  TIM_ClockConfigTypeDef sClockSourceConfig;
  TIM_MasterConfigTypeDef sMasterConfig;
  TIM_OC_InitTypeDef sConfigOC;

  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 83;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 0xffff;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  HAL_TIM_Base_Init(&htim3);

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig);

  HAL_TIM_OC_Init(&htim3);

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig);

  sConfigOC.OCMode = TIM_OCMODE_TIMING;
  sConfigOC.Pulse = 4000;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  HAL_TIM_OC_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1);

}

/* UART4 init function */
void MX_UART4_Init(void)
{

  huart4.Instance = UART4;
  huart4.Init.BaudRate = 115200;
  huart4.Init.WordLength = UART_WORDLENGTH_8B;
  huart4.Init.StopBits = UART_STOPBITS_1;
  huart4.Init.Parity = UART_PARITY_NONE;
  huart4.Init.Mode = UART_MODE_TX_RX;
  huart4.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart4.Init.OverSampling = UART_OVERSAMPLING_16;
  HAL_UART_Init(&huart4);

}
void MX_USART3_UART_Init(void)
{
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 100000;//115200;
  huart3.Init.WordLength = UART_WORDLENGTH_9B;
  huart3.Init.StopBits = UART_STOPBITS_2;
  huart3.Init.Parity = UART_PARITY_EVEN;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  HAL_UART_Init(&huart3);
}
void MX_USART2_UART_Init(void)
{
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 38400;//115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  HAL_UART_Init(&huart2);
}


/* USART1 init function */
void MX_USART1_UART_Init(void)
{

  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  HAL_UART_Init(&huart1);

}

/** 
  * Enable DMA controller clock
  */
void MX_DMA_Init(void) 
{
   __DMA2_CLK_ENABLE();
  __DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  HAL_NVIC_SetPriority(DMA1_Stream1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream1_IRQn);
  HAL_NVIC_SetPriority(DMA2_Stream2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream2_IRQn);
  HAL_NVIC_SetPriority(DMA1_Stream2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream2_IRQn);
//  HAL_NVIC_SetPriority(DMA1_Stream5_IRQn, 0, 0);
//  HAL_NVIC_EnableIRQ(DMA1_Stream5_IRQn);
	

}

/** Configure pins as 
        * Analog 
        * Input 
        * Output
        * EVENT_OUT
        * EXTI
*/
void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __GPIOE_CLK_ENABLE();
  __GPIOA_CLK_ENABLE();
  __GPIOB_CLK_ENABLE();
  __GPIOC_CLK_ENABLE();
  __GPIOD_CLK_ENABLE();


  /*Configure GPIO pins : PE2 PE15 */
  GPIO_InitStruct.Pin = GPIO_PIN_2|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : PA4 PA11 */
  GPIO_InitStruct.Pin = GPIO_PIN_4|GPIO_PIN_11;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
	
	  GPIO_InitStruct.Pin = GPIO_PIN_11;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);
	
	  GPIO_InitStruct.Pin = GPIO_PIN_14;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);
	
		  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pin : PD0 */
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pin : PD1 */
  GPIO_InitStruct.Pin = GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI1_IRQn);

}

/* USER CODE BEGIN 4 */
///
void M_I2C_init(I2C_HandleTypeDef *hi2c,I2C_TypeDef* I2Cx, uint32_t clockSpeed)
{
	I2C_InitTypeDef I2C_InitStruct;
	GPIO_InitTypeDef GPIO_InitDef;
	
	I2C_InitStruct.ClockSpeed = clockSpeed;
	if (I2Cx == I2C3) 
	{
		__GPIOA_FORCE_RESET();
		__GPIOA_RELEASE_RESET();
		__GPIOC_FORCE_RESET();
		__GPIOC_RELEASE_RESET();
		hi2c->Instance = I2C3;
		__HAL_RCC_I2C3_CLK_ENABLE();
		__HAL_RCC_GPIOB_CLK_ENABLE();
		GPIO_InitDef.Mode = GPIO_MODE_AF_OD;
		GPIO_InitDef.Pull = GPIO_PULLUP;
		GPIO_InitDef.Speed = GPIO_SPEED_MEDIUM;
		GPIO_InitDef.Pin = GPIO_PIN_8;
		GPIO_InitDef.Alternate = GPIO_AF4_I2C3;
		HAL_GPIO_Init(GPIOA,&GPIO_InitDef);
		GPIO_InitDef.Pin = GPIO_PIN_9;
		HAL_GPIO_Init(GPIOC,&GPIO_InitDef);
		
  I2C_InitStruct.DutyCycle = I2C_DUTYCYCLE_2;
  I2C_InitStruct.OwnAddress1 = 0;
  I2C_InitStruct.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  I2C_InitStruct.DualAddressMode = I2C_DUALADDRESS_DISABLED;
  I2C_InitStruct.OwnAddress2 = 0;
  I2C_InitStruct.GeneralCallMode = I2C_GENERALCALL_DISABLED;
  I2C_InitStruct.NoStretchMode = I2C_NOSTRETCH_DISABLED;
	hi2c->Init = I2C_InitStruct;
	hi2c->State =  HAL_I2C_STATE_BUSY;
	M_I2C_Init(&hi2c3);
	}
	else if (I2Cx == I2C1) 
	{
		__GPIOB_FORCE_RESET();
		__GPIOB_RELEASE_RESET();
		hi2c->Instance = I2C1;
		__HAL_RCC_I2C1_CLK_ENABLE();
		__HAL_RCC_GPIOB_CLK_ENABLE();
		GPIO_InitDef.Mode = GPIO_MODE_AF_OD;
		GPIO_InitDef.Pull = GPIO_PULLUP;
		GPIO_InitDef.Speed = GPIO_SPEED_MEDIUM;
		GPIO_InitDef.Pin = GPIO_PIN_6 | GPIO_PIN_7;
		GPIO_InitDef.Alternate = GPIO_AF4_I2C1;
		HAL_GPIO_Init(GPIOB,&GPIO_InitDef);
		
  I2C_InitStruct.DutyCycle = I2C_DUTYCYCLE_16_9;
  I2C_InitStruct.OwnAddress1 = 0;
  I2C_InitStruct.AddressingMode = I2C_ADDRESSINGMODE_10BIT;
  I2C_InitStruct.OwnAddress2 = 0;
  I2C_InitStruct.GeneralCallMode = I2C_GENERALCALL_DISABLED;
  I2C_InitStruct.NoStretchMode = I2C_NOSTRETCH_DISABLED;
	hi2c->Init = I2C_InitStruct;
	hi2c->State =  HAL_I2C_STATE_BUSY;
M_I2C_Init(&hi2c1);


	}

//	else if (I2Cx == I2C2) 
//	{
//		__GPIOF_FORCE_RESET();
//		__GPIOF_RELEASE_RESET();
//		hi2c->Instance = I2C2;
//		__HAL_RCC_I2C2_CLK_ENABLE();
//		__HAL_RCC_GPIOF_CLK_ENABLE();
//		GPIO_InitDef.Mode = GPIO_MODE_AF_OD;
//		GPIO_InitDef.Pull = GPIO_PULLUP;
//		GPIO_InitDef.Speed = GPIO_SPEED_MEDIUM;
//		GPIO_InitDef.Pin = GPIO_PIN_0 | GPIO_PIN_1;
//		GPIO_InitDef.Alternate = GPIO_AF4_I2C2;
//		HAL_GPIO_Init(GPIOF,&GPIO_InitDef);
//		
//  I2C_InitStruct.DutyCycle = I2C_DUTYCYCLE_2;
//  I2C_InitStruct.OwnAddress1 = 0;
//  I2C_InitStruct.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
//  I2C_InitStruct.DualAddressMode = I2C_DUALADDRESS_DISABLED;
//  I2C_InitStruct.OwnAddress2 = 0;
//  I2C_InitStruct.GeneralCallMode = I2C_GENERALCALL_DISABLED;
//  I2C_InitStruct.NoStretchMode = I2C_NOSTRETCH_DISABLED;
//	hi2c->Init = I2C_InitStruct;
//	hi2c->State =  HAL_I2C_STATE_BUSY;
//	}
//	else 
//		if (I2Cx == I2C1) 
//	{
////		__GPIOA_CLK_ENABLE();
////		__GPIOC_CLK_ENABLE();
////		__GPIOC_FORCE_RESET();
////		__GPIOA_FORCE_RESET();
////		__GPIOC_RELEASE_RESET();
////		__GPIOA_RELEASE_RESET();
//		hi2c->Instance = I2C1;
//		__HAL_RCC_I2C3_CLK_ENABLE();
//		__HAL_RCC_GPIOB_CLK_ENABLE();
//		__HAL_RCC_GPIOB_CLK_ENABLE();
//		GPIO_InitDef.Mode = GPIO_MODE_AF_OD;
//		GPIO_InitDef.Pull = GPIO_PULLUP;
//		GPIO_InitDef.Speed = GPIO_SPEED_MEDIUM;
//		GPIO_InitDef.Pin = GPIO_PIN_8 | GPIO_PIN_9 ;
//		GPIO_InitDef.Alternate = GPIO_AF4_I2C1;
////		HAL_GPIO_Init(GPIOB,&GPIO_InitDef);  //a8 scl ===> b8
////		GPIO_InitDef.Pin = GPIO_PIN_9;
//		HAL_GPIO_Init(GPIOB,&GPIO_InitDef);   //c9 sda ===> b9
//  I2C_InitStruct.DutyCycle = I2C_DUTYCYCLE_16_9;
//  I2C_InitStruct.OwnAddress1 = 0;
//  I2C_InitStruct.AddressingMode = I2C_ADDRESSINGMODE_10BIT;
//  I2C_InitStruct.DualAddressMode = I2C_DUALADDRESS_DISABLED;
//  I2C_InitStruct.OwnAddress2 = 0;
//  I2C_InitStruct.GeneralCallMode = I2C_GENERALCALL_DISABLED;
//  I2C_InitStruct.NoStretchMode = I2C_NOSTRETCH_DISABLED;
//	hi2c->Init = I2C_InitStruct;
//	hi2c->State =  HAL_I2C_STATE_BUSY;
//	}
	M_I2C_Init(hi2c);
}
void M_I2C_init_(I2C_HandleTypeDef *hi2c,I2C_TypeDef* I2Cx, uint32_t clockSpeed)
{
	I2C_InitTypeDef I2C_InitStruct;
	GPIO_InitTypeDef GPIO_InitDef;
	
	I2C_InitStruct.ClockSpeed = clockSpeed;
	if (I2Cx == I2C3) 
	{
		__GPIOA_FORCE_RESET();
		__GPIOA_RELEASE_RESET();
		__GPIOC_FORCE_RESET();
		__GPIOC_RELEASE_RESET();
		hi2c->Instance = I2C3;
		__HAL_RCC_I2C3_CLK_ENABLE();
		__HAL_RCC_GPIOB_CLK_ENABLE();
		GPIO_InitDef.Mode = GPIO_MODE_AF_OD;
		GPIO_InitDef.Pull = GPIO_PULLUP;
		GPIO_InitDef.Speed = GPIO_SPEED_MEDIUM;
		GPIO_InitDef.Pin = GPIO_PIN_8;
		GPIO_InitDef.Alternate = GPIO_AF4_I2C3;
		HAL_GPIO_Init(GPIOA,&GPIO_InitDef);
		GPIO_InitDef.Pin = GPIO_PIN_9;
		HAL_GPIO_Init(GPIOC,&GPIO_InitDef);
		
  I2C_InitStruct.DutyCycle = I2C_DUTYCYCLE_2;
  I2C_InitStruct.OwnAddress1 = 0;
  I2C_InitStruct.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  I2C_InitStruct.DualAddressMode = I2C_DUALADDRESS_DISABLED;
  I2C_InitStruct.OwnAddress2 = 0;
  I2C_InitStruct.GeneralCallMode = I2C_GENERALCALL_DISABLED;
  I2C_InitStruct.NoStretchMode = I2C_NOSTRETCH_DISABLED;
	hi2c->Init = I2C_InitStruct;
	hi2c->State =  HAL_I2C_STATE_BUSY;
	M_I2C_Init(&hi2c3);
	}
	else if (I2Cx == I2C1) 
	{
		__GPIOB_FORCE_RESET();
		__GPIOB_RELEASE_RESET();
		hi2c->Instance = I2C1;
		__HAL_RCC_I2C1_CLK_ENABLE();
		__HAL_RCC_GPIOB_CLK_ENABLE();
		GPIO_InitDef.Mode = GPIO_MODE_AF_OD;
		GPIO_InitDef.Pull = GPIO_PULLUP;
		GPIO_InitDef.Speed = GPIO_SPEED_MEDIUM;
		GPIO_InitDef.Pin = GPIO_PIN_6 | GPIO_PIN_7;
		GPIO_InitDef.Alternate = GPIO_AF4_I2C1;
		HAL_GPIO_Init(GPIOB,&GPIO_InitDef);
		
  I2C_InitStruct.DutyCycle = I2C_DUTYCYCLE_16_9;
  I2C_InitStruct.OwnAddress1 = 0;
  I2C_InitStruct.AddressingMode = I2C_ADDRESSINGMODE_10BIT;
  I2C_InitStruct.OwnAddress2 = 0;
  I2C_InitStruct.GeneralCallMode = I2C_GENERALCALL_DISABLED;
  I2C_InitStruct.NoStretchMode = I2C_NOSTRETCH_DISABLED;
	hi2c->Init = I2C_InitStruct;
	hi2c->State =  HAL_I2C_STATE_BUSY;
M_I2C_Init(&hi2c1);	


	}
	

//	else if (I2Cx == I2C2) 
//	{
//		__GPIOF_FORCE_RESET();
//		__GPIOF_RELEASE_RESET();
//		hi2c->Instance = I2C2;
//		__HAL_RCC_I2C2_CLK_ENABLE();
//		__HAL_RCC_GPIOF_CLK_ENABLE();
//		GPIO_InitDef.Mode = GPIO_MODE_AF_OD;
//		GPIO_InitDef.Pull = GPIO_PULLUP;
//		GPIO_InitDef.Speed = GPIO_SPEED_MEDIUM;
//		GPIO_InitDef.Pin = GPIO_PIN_0 | GPIO_PIN_1;
//		GPIO_InitDef.Alternate = GPIO_AF4_I2C2;
//		HAL_GPIO_Init(GPIOF,&GPIO_InitDef);
//		
//  I2C_InitStruct.DutyCycle = I2C_DUTYCYCLE_2;
//  I2C_InitStruct.OwnAddress1 = 0;
//  I2C_InitStruct.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
//  I2C_InitStruct.DualAddressMode = I2C_DUALADDRESS_DISABLED;
//  I2C_InitStruct.OwnAddress2 = 0;
//  I2C_InitStruct.GeneralCallMode = I2C_GENERALCALL_DISABLED;
//  I2C_InitStruct.NoStretchMode = I2C_NOSTRETCH_DISABLED;
//	hi2c->Init = I2C_InitStruct;
//	hi2c->State =  HAL_I2C_STATE_BUSY;
//	}
//	else 
//		if (I2Cx == I2C1) 
//	{
////		__GPIOA_CLK_ENABLE();
////		__GPIOC_CLK_ENABLE();
////		__GPIOC_FORCE_RESET();
////		__GPIOA_FORCE_RESET();
////		__GPIOC_RELEASE_RESET();
////		__GPIOA_RELEASE_RESET();
//		hi2c->Instance = I2C1;
//		__HAL_RCC_I2C3_CLK_ENABLE();
//		__HAL_RCC_GPIOB_CLK_ENABLE();
//		__HAL_RCC_GPIOB_CLK_ENABLE();
//		GPIO_InitDef.Mode = GPIO_MODE_AF_OD;
//		GPIO_InitDef.Pull = GPIO_PULLUP;
//		GPIO_InitDef.Speed = GPIO_SPEED_MEDIUM;
//		GPIO_InitDef.Pin = GPIO_PIN_8 | GPIO_PIN_9 ;
//		GPIO_InitDef.Alternate = GPIO_AF4_I2C1;
////		HAL_GPIO_Init(GPIOB,&GPIO_InitDef);  //a8 scl ===> b8
////		GPIO_InitDef.Pin = GPIO_PIN_9;
//		HAL_GPIO_Init(GPIOB,&GPIO_InitDef);   //c9 sda ===> b9
//  I2C_InitStruct.DutyCycle = I2C_DUTYCYCLE_16_9;
//  I2C_InitStruct.OwnAddress1 = 0;
//  I2C_InitStruct.AddressingMode = I2C_ADDRESSINGMODE_10BIT;
//  I2C_InitStruct.DualAddressMode = I2C_DUALADDRESS_DISABLED;
//  I2C_InitStruct.OwnAddress2 = 0;
//  I2C_InitStruct.GeneralCallMode = I2C_GENERALCALL_DISABLED;
//  I2C_InitStruct.NoStretchMode = I2C_NOSTRETCH_DISABLED;
//	hi2c->Init = I2C_InitStruct;
//	hi2c->State =  HAL_I2C_STATE_BUSY;
//	}

}
HAL_StatusTypeDef M_I2C_Init(I2C_HandleTypeDef *hi2c)
{
  uint32_t freqrange = 0;
  uint32_t pclk1 = 0;
	uint16_t tmpreg = 0;
	uint16_t result = 0x04;
  /* Check the I2C handle allocation */
  if(hi2c == NULL)
  {
    return HAL_ERROR;
  }

  /* Check the parameters */
  assert_param(IS_I2C_ALL_INSTANCE(hi2c->Instance));
  assert_param(IS_I2C_CLOCK_SPEED(hi2c->Init.ClockSpeed));
  assert_param(IS_I2C_DUTY_CYCLE(hi2c->Init.DutyCycle));
  assert_param(IS_I2C_OWN_ADDRESS1(hi2c->Init.OwnAddress1));
  assert_param(IS_I2C_ADDRESSING_MODE(hi2c->Init.AddressingMode));
  assert_param(IS_I2C_DUAL_ADDRESS(hi2c->Init.DualAddressMode));
  assert_param(IS_I2C_OWN_ADDRESS2(hi2c->Init.OwnAddress2));
  assert_param(IS_I2C_GENERAL_CALL(hi2c->Init.GeneralCallMode));
  assert_param(IS_I2C_NO_STRETCH(hi2c->Init.NoStretchMode));


  hi2c->State = HAL_I2C_STATE_BUSY;

  /* Disable the selected I2C peripheral */
  __HAL_I2C_DISABLE(hi2c);

  /* Get PCLK1 frequency */
  pclk1 = HAL_RCC_GetPCLK1Freq();

  /* Calculate frequency range */
  freqrange = I2C_FREQRANGE(pclk1);

	
  /*---------------------------- I2Cx CR2 Configuration ----------------------*/
  /* Get the I2Cx CR2 value */
  tmpreg = hi2c->Instance->CR2;
	/* Clear frequency FREQ[5:0] bits */
  tmpreg &= (uint16_t)~((uint16_t)I2C_CR2_FREQ);
	/* Set frequency bits depending on pclk1 value */
  freqrange = (uint16_t)(pclk1 / 1000000);
	tmpreg |= freqrange;
  /* Write to I2Cx CR2 */
  hi2c->Instance->CR2 = tmpreg;

	/*---------------------------- I2Cx CCR Configuration ----------------------*/
  /* Disable the selected I2C peripheral to configure TRISE */
  hi2c->Instance->CR1 &= (uint16_t)~((uint16_t)I2C_CR1_PE);
  /* Reset tmpreg value */
  /* Clear F/S, DUTY and CCR[11:0] bits */
  tmpreg = 0;

  /* Configure speed in standard mode */
  if (hi2c->Init.ClockSpeed <= 100000)
  {
    /* Standard mode speed calculate */
    result = (uint16_t)(pclk1 / (hi2c->Init.ClockSpeed << 1));
    /* Test if CCR value is under 0x4*/
    if (result < 0x04)
    {
      /* Set minimum allowed value */
      result = 0x04;  
    }
    /* Set speed value for standard mode */
    tmpreg |= result;	  
    /* Set Maximum Rise Time for standard mode */
    hi2c->Instance->TRISE = freqrange + 1; 
  }
  /* Configure speed in fast mode */
  /* To use the I2C at 400 KHz (in fast mode), the PCLK1 frequency (I2C peripheral
     input clock) must be a multiple of 10 MHz */
  else /*(I2C_InitStruct->I2C_ClockSpeed <= 400000)*/
  {
    if (hi2c->Init.DutyCycle == I2C_DUTYCYCLE_2)
    {
      /* Fast mode speed calculate: Tlow/Thigh = 2 */
      result = (uint16_t)(pclk1 / (hi2c->Init.ClockSpeed * 3));
    }
    else /*I2C_InitStruct->I2C_DutyCycle == I2C_DutyCycle_16_9*/
    {
      /* Fast mode speed calculate: Tlow/Thigh = 16/9 */
      result = (uint16_t)(pclk1 / (hi2c->Init.ClockSpeed * 25));
      /* Set DUTY bit */
      result |= I2C_DUTYCYCLE_16_9;
    }

    /* Test if CCR value is under 0x1*/
    if ((result & I2C_CCR_CCR) == 0)
    {
      /* Set minimum allowed value */
      result |= (uint16_t)0x0001;  
    }
    /* Set speed value and set F/S bit for fast mode */
    tmpreg |= (uint16_t)(result | I2C_CCR_FS);
    /* Set Maximum Rise Time for fast mode */
    hi2c->Instance->TRISE = (uint16_t)(((freqrange * (uint16_t)300) / (uint16_t)1000) + (uint16_t)1);  
  }

  /* Write to I2Cx CCR */
  hi2c->Instance->CCR = tmpreg;
  /* Enable the selected I2C peripheral */
  hi2c->Instance->CR1 |= I2C_CR1_PE;
  
  /*---------------------------- I2Cx CR1 Configuration ----------------------*/
	/* Get the I2Cx CR1 value */
  tmpreg = hi2c->Instance->CR1;
  /* Clear ACK, SMBTYPE and  SMBUS bits */
  tmpreg &= (uint16_t)0xFBF5;
  /* Configure I2Cx: mode and acknowledgement */
  /* Set SMBTYPE and SMBUS bits according to I2C_Mode value */
  /* Set ACK bit according to I2C_Ack value */
	
  tmpreg |= (uint16_t)(hi2c->Init.GeneralCallMode | hi2c->Init.NoStretchMode);
  /* Write to I2Cx CR1 */
  hi2c->Instance->CR1 = tmpreg;
	
  /*---------------------------- I2Cx OAR1 Configuration ---------------------*/
  /* Configure I2Cx: Own Address1 and addressing mode */
  hi2c->Instance->OAR1 = (hi2c->Init.AddressingMode | hi2c->Init.OwnAddress1);

  /* Enable the selected I2C peripheral */
  __HAL_I2C_ENABLE(hi2c);

  hi2c->ErrorCode = HAL_I2C_ERROR_NONE;
  hi2c->State = HAL_I2C_STATE_READY;

  return HAL_OK;
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{	

	
	if(huart->Instance == UART4)
	{	
			
			MPC.UART_IRQ_FLAG	= 1;
	}
	
		else if(huart->Instance == USART3)
	{	
			SBUS.flag= 1;
			
		
	}
//	if(CAM_IRQ_Flag)
//	{
			else if(huart->Instance == USART1 )
		{
moj_counter_usart_intrupt++;
		//	 __HAL_UART_FLUSH_DRREGISTER(&huart1);
//		
				
			MPC_2.UART_IRQ_FLAG	= 1;
//			//conuter_mpc_usart = __HAL_TIM_GetCounter(&htim13);
		}
////		else if(huart->Instance == UART4)
////		{
////			moj_interupt_usart_rc++;
////			data_r = 1;
////			if(station_data == 's') RC.init = 0;
////			//HAL_UART_Transmit(&huart1,&rxbuff,1,100);
////			//HAL_UART_Receive_IT(&huart1,(uint8_t *)aRxBuffer,1);
////		}
////	}
	
}



void HAL_I2C_MasterRxCpltCallback(I2C_HandleTypeDef *hi2c)
{

//		LEDY_TGL;
	
}

void HAL_TIM_OC_DelayElapsedCallback(TIM_HandleTypeDef *htim)
{
	if(htim->Instance == TIM3)
	{
		if(htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1)
		{
		  __HAL_TIM_SetCompare(htim,TIM_CHANNEL_1,__HAL_TIM_GetCompare(htim,TIM_CHANNEL_1)+DT_PULSE);
			lock_time = 0;
		}

	}		
	
}
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	
	if(GPIO_Pin == GPIO_PIN_1)
	{
		if(Ultra.State == 1 && HAL_GPIO_ReadPin(GPIOD,GPIO_Pin) ==1)
		{
			Ultra.Begine = (uint16_t)htim3.Instance->CNT;
			Ultra.State = 2;
			
		}
		else if(Ultra.State == 2 && HAL_GPIO_ReadPin(GPIOD,GPIO_Pin) ==0)
		{
			Ultra.End = (uint16_t)htim3.Instance->CNT;
			Ultra.State = 3;
			Ultra.ready = 1;

		}
  }
}

void Led_Beat(char state,int count)
{
	Led_Level = Led_Level + 1;
	switch(state)
	{
		case GROUND_MODE:
				if(Led_Level % 45 ==0 && Led_Level >121)
					LEDB_TGL;
				if(Led_Level > 392)
				{
					LEDB_OFF;		
					Led_Level =0;
				}
			break;
		case READY_2_FLY:
				if(Led_Level % 30 ==0 && Led_Level >61)
					LEDB_TGL;
				if(Led_Level > 209)
				{
					LEDB_OFF;		
					Led_Level =0;
				}
			break;
		case FLY_MODE:
			if(Led_Level % 30 ==0)
					LEDB_TGL;
			
			break;
	}
	if(NRF.fail==1)
	{
			if( ((int)(count/10)) % 4 ==0)
					LEDY_TGL;		
	}
	else LEDY_OFF;
}

void Check_battery(int System_counter,int System_State)
{
	unsigned int LED_BAT_on_time=0;
	
	if(System_State == FLY_MODE)
	{
		if((12-(Bat_/BATT_Coef))>0)
			LED_BAT_on_time=(unsigned int)((100*(12.6-(Bat_/BATT_Coef))));
		else LED_BAT_on_time=0;
		
		if(System_counter < LED_BAT_on_time)	
			LEDW_ON;
		else LEDW_OFF;		
	}
	else 
	{
		if((11.5f-(Bat_/BATT_Coef))>0)
			LED_BAT_on_time=(unsigned int)((200*(11.5f-(Bat_/BATT_Coef))));  
		else LED_BAT_on_time=0;
		
		if(System_counter < LED_BAT_on_time)	
			LEDW_ON;
		else LEDW_OFF;
	}
	
}



//void setup()
//{

//}
mavlink_system_t                   mavlink_system;
mavlink_optical_flow_t             OPTICAL_FLOW;
mavlink_battery_status_t           BATT_STATUS;
mavlink_manual_setpoint_t          RC_SETPOINT;
mavlink_sim_state_t                SIM_STATE ;
mavlink_attitude_t                 ATTITUDE ;
mavlink_altitude_t                 ALTITUDE ;
mavlink_rc_channels_t              RC_RAW_PPM_CHANNELS; // The standard PPM modulation is as follows: 1000 microseconds: 0%, 2000 microseconds: 100%
mavlink_rc_channels_raw_t          RC_CHANNELS_RAW ;
mavlink_rc_channels_scaled_t       RC_CHANNEL_SCALED ;
mavlink_mission_item_t             MISSION_ITEM ; //Message encoding a mission item. This message is emitted to announce the presence of a mission item and to set a mission item on the system. The mission item can be either in x, y, z meters (type: LOCAL) or x:lat, y:lon, z:altitude. Local frame is Z-down, right handed (NED), global frame is Z-up, right handed (ENU). See also http://qgroundcontrol.org/mavlink/waypoint_protocol.
mavlink_mission_request_t          MISSION_REQUEST ;//Request the information of the mission item with the sequence number seq. The response of the system to this message should be a MISSION_ITEM message. http://qgroundcontrol.org/mavlink/waypoint_protocol
mavlink_mission_set_current_t      MISSION_SET_CURRENT ;//Set the mission item with sequence number seq as current item. This means that the MAV will continue to this mission item on the shortest path (not following the mission items in-between).
mavlink_mission_current_t          MISSION_CURRENT; //Message that announces the sequence number of the current active mission item. The MAV will fly towards this mission item.
mavlink_mission_request_list_t     MISSION_REQUEST_LIST; //Request the overall list of mission items from the system/component.
mavlink_mission_count_t            MISSION_COUNT ; //This message is emitted as response to MISSION_REQUEST_LIST by the MAV and to initiate a write transaction. The GCS can then request the individual mission item based on the knowledge of the total number of MISSIONs.
mavlink_local_position_ned_cov_t   LOCAL_POSITION_NED_COV ; //The filtered local position (e.g. fused computer vision and accelerometers). Coordinate frame is right-handed, Z-axis down (aeronautical frame, NED / north-east-down convention)
mavlink_mission_item_int_t         MISSION_ITEM_INT  ;//
mavlink_scaled_imu_t               SCALED_IMU ; //The RAW IMU readings for the usual 9DOF sensor setup. This message should contain the scaled values to the described units
mavlink_attitude_quaternion_t      ATTITUDE_QUATERNION;
mavlink_highres_imu_t              HIGHRES_IMU ; //The IMU readings in SI units in NED body frame
mavlink_control_system_state_t     CONTROL_SYSTEM_STATE ;//The smoothed, monotonic system state used to feed the control loops of the system


 mavlink_message_t msg;
 mavlink_status_t status;

void encode_mavlink_packet(void)
{
  uint8_t buf[MAVLINK_MAX_PACKET_LEN];

	uint16_t len;
	mavlink_system.sysid = 20;                   ///< ID 20 for this airplane
	mavlink_system.compid = MAV_COMP_ID_ALL;     ///< The component sending the message is the IMU, it could be also a Linux process

	
  //shrue meghdar dehi moteghayer haye mavlink*********************************************	
//  RC_SETPOINT.pitch      = RC.Pitch*(PI/180);
//	RC_SETPOINT.roll       = RC.Roll*(PI/180);
//  RC_SETPOINT.thrust     = RC.Throttle;
//  RC_SETPOINT.yaw        = RC.Yaw*(PI/180);	
//	RC_SETPOINT.mode_switch= Run_State;
//	RC_SETPOINT.manual_override_switch =
//	RC_SETPOINT.time_boot_ms =
	//***************************************************************************************
//	CONTROL_SYSTEM_STATE.x_acc =  //X acceleration in body frame
//	CONTROL_SYSTEM_STATE.y_acc = 
//	CONTROL_SYSTEM_STATE.z_acc =
//	CONTROL_SYSTEM_STATE.x_pos =  //X position in local frame
//	CONTROL_SYSTEM_STATE.y_pos =
//	CONTROL_SYSTEM_STATE.z_pos = 
//	CONTROL_SYSTEM_STATE.pitch_rate = //Angular rate in pitch axis
//	CONTROL_SYSTEM_STATE.roll_rate  = //Angular rate in roll axis
//	CONTROL_SYSTEM_STATE.yaw_rate   = //Angular rate in yaw axis
//	CONTROL_SYSTEM_STATE.x_vel      = //X velocity in body frame
//	CONTROL_SYSTEM_STATE.y_vel      =
//	CONTROL_SYSTEM_STATE.z_vel      =
//	CONTROL_SYSTEM_STATE.vel_variance = //Variance of body velocity estimate
//	CONTROL_SYSTEM_STATE.airspeed = -1 //	Airspeed, set to -1 if unknown
//	CONTROL_SYSTEM_STATE.pos_variance = //Variance in local position
//	CONTROL_SYSTEM_STATE.q = //The attitude, represented as Quaternion
//	CONTROL_SYSTEM_STATE.time_usec = //Timestamp (micros since boot or Unix epoch)
	//***************************************************************************************
  HIGHRES_IMU.xacc=  Mpu.acc_x;
	HIGHRES_IMU.yacc=  Mpu.acc_y;
	HIGHRES_IMU.zacc=  Mpu.acc_z;
	HIGHRES_IMU.xgyro= Mpu.gyro_x;
	HIGHRES_IMU.ygyro= Mpu.gyro_y;
	HIGHRES_IMU.zgyro= Mpu.gyro_z;
	//***************************************************************************************

	ATTITUDE_QUATERNION.q1=Mahony.q0;
	ATTITUDE_QUATERNION.q2=Mahony.q1;
	ATTITUDE_QUATERNION.q3=Mahony.q2;
	ATTITUDE_QUATERNION.q4=Mahony.q3;
	//***************************************************************************************
//	SCALED_IMU.xacc=Mpu.acc_x;
//	SCALED_IMU.yacc=Mpu.acc_y;
//	SCALED_IMU.ymag=Mpu.acc_z;
//	
//	SCALED_IMU.xgyro=Mpu.gyro_x;
//	SCALED_IMU.ygyro=Mpu.gyro_y;
//	SCALED_IMU.zgyro=Mpu.gyro_z;
	
	//****************************************************************************************
//	LOCAL_POSITION_NED_COV.ax= //X Acceleration (m/s^2)
//	LOCAL_POSITION_NED_COV.ay=
//	LOCAL_POSITION_NED_COV.az=
//	LOCAL_POSITION_NED_COV.covariance=//Covariance matrix upper right triangular (first nine entries are the first ROW, next eight entries are the second row, etc.)
//	LOCAL_POSITION_NED_COV.estimator_type= //	Timestamp (microseconds since UNIX epoch) in UTC. 0 for unknown. Commonly filled by the precision time source of a GPS receiver.
//	LOCAL_POSITION_NED_COV.time_boot_ms= //Timestamp (milliseconds since system boot). 0 for system without monotonic timestamp
//	LOCAL_POSITION_NED_COV.time_utc= //	Timestamp (microseconds since UNIX epoch) in UTC. 0 for unknown. Commonly filled by the precision time source of a GPS receiver.
//	LOCAL_POSITION_NED_COV.vx= //X Speed (m/s)
//	LOCAL_POSITION_NED_COV.vy=
//	LOCAL_POSITION_NED_COV.vz=
//	LOCAL_POSITION_NED_COV.x= //	X Position
//	LOCAL_POSITION_NED_COV.y=
//	LOCAL_POSITION_NED_COV.z= 
	//****************************************************************************************
//	MISSION_COUNT.count =1;//Number of mission items in the sequence
//	MISSION_COUNT.target_component =0;
//	MISSION_COUNT.target_system=20;
	//****************************************************************************************
//	MISSION_REQUEST_LIST.target_component=0;
//	MISSION_REQUEST_LIST.target_system=20;
	//****************************************************************************************
//	MISSION_CURRENT.seq=1;
	//**********************************************************************************
//	MISSION_SET_CURRENT.seq= 1;
//	MISSION_SET_CURRENT.target_component=0;
//	MISSION_SET_CURRENT.target_system=20;
	//**************************************************************************************
//	MISSION_REQUEST.seq = 1 ;
//	MISSION_REQUEST.target_component =0 ; //Component which should execute the command, 0 for all components
//	MISSION_REQUEST.target_system = 20 ;
	//*****************************************************************************************
//	Message encoding a mission item. 
//	This message is emitted to announce the presence of a mission item and to set a mission item on the system.
//	The mission item can be either in x, y, z meters (type: LOCAL) or x:lat, y:lon, z:altitude.
//	Local frame is Z-down, right handed (NED), global frame is Z-up, right handed (ENU).
//	See also http://qgroundcontrol.org/mavlink/waypoint_protocol
//	MISSION_ITEM.autocontinue= //	autocontinue to next wp
	MISSION_ITEM.command = MAV_CMD_NAV_TAKEOFF_LOCAL; //The scheduled action for the MISSION. see MAV_CMD in common.xml MAVLink specs 
	MISSION_ITEM.current = 1 ;//false:0, true:1
	MISSION_ITEM.frame =MAV_FRAME_LOCAL_NED;//The coordinate system of the MISSION. see MAV_FRAME in mavlink_types.h
//	MISSION_ITEM.param1 = MAV_CMD_NAV_TAKEOFF;//PARAM1, see MAV_CMD enum
//	MISSION_ITEM.param2 =MAV_CMD_NAV_TAKEOFF;//PARAM2, see MAV_CMD enum
//	MISSION_ITEM.param3 =MAV_CMD_NAV_TAKEOFF;//PARAM3, see MAV_CMD enum
//	MISSION_ITEM.param4 =MAV_CMD_NAV_TAKEOFF;//PARAM4, see MAV_CMD enum
//Mission Param #1	Minimum pitch (if airspeed sensor present), desired pitch without sensor [rad]
//Mission Param #2	Empty
//Mission Param #3	Takeoff ascend rate [ms^-1]
//Mission Param #4	Yaw angle [rad] (if magnetometer or another yaw estimation source present), ignored without one of these
//Mission Param #5	Y-axis position [m]
//Mission Param #6	X-axis position [m]
//Mission Param #7	Z-axis position [m]
//	MISSION_ITEM.seq = //Sequence
	//MISSION_ITEM.target_component = //Component ID
	MISSION_ITEM.target_system = 20 ;//System ID
	MISSION_ITEM.x = 77;//PARAM5 / local: x position, global: latitude
	MISSION_ITEM.y =77;//PARAM6 / y position: global: longitude
	MISSION_ITEM.z =77; //PARAM7 / z position: global: altitude (relative or absolute, depending on frame.
  //*****************************************************************************************	
//	SIM_STATE.roll  = Roll.point*(PI/180);
//	SIM_STATE.pitch = Pitch.point*(PI/180);
//	SIM_STATE.yaw   = Yaw.point*(PI/180);
//	SIM_STATE.xacc  = Mpu.acc_x;
//	SIM_STATE.yacc  = Mpu.acc_y;
//  SIM_STATE.zacc  = Mpu.acc_z;
//	SIM_STATE.xgyro = Mpu.gyro_x;
//	SIM_STATE.zgyro = Mpu.gyro_z;
//	SIM_STATE.ygyro = Mpu.gyro_y;
//  SIM_STATE.q1    = Mahony.q0;
//	SIM_STATE.q2    = Mahony.q1;
//	SIM_STATE.q3    = Mahony.q2;
//	SIM_STATE.q4    = Mahony.q3;
//	SIM_STATE.alt   = 
//	SIM_STATE.lat   =
//	SIM_STATE.lon   =
//	SIM_STATE.std_dev_horz =
//	SIM_STATE.std_dev_vert= =
//	SIM_STATE.vd    =
//	SIM_STATE.ve    =
//	SIM_STATE.vn    =
	//********************************************************************************************
	ATTITUDE.pitch  = Pitch.point*(PI/180);
	ATTITUDE.roll   = Roll.point*(PI/180);
	ATTITUDE.yaw    = Yaw.point*(PI/180);
//	ATTITUDE.pitchspeed   =
//	ATTITUDE.rollspeed    =
//	ATTITUDE.yawspeed     = 
//	ATTITUDE.time_boot_ms =
  //********************************************************************************************
  OPTICAL_FLOW.flow_x       =100;// optical_par.delta_X; //Flow in pixels * 10 in x-sensor direction (dezi-pixels)
	OPTICAL_FLOW.flow_y       = 522;//optical_par.delta_Y; //	Flow in pixels * 10 in y-sensor direction (dezi-pixels)
//	OPTICAL_FLOW.flow_comp_m_x= Velocity.X.point; //Flow in meters in x-sensor direction, angular-speed compensated
//	OPTICAL_FLOW.flow_comp_m_y= Velocity.Y.point; //Flow in meters in y-sensor direction, angular-speed compensated
//	OPTICAL_FLOW.ground_distance= //	Ground distance in meters. Positive value: distance known. Negative value: Unknown distance
	OPTICAL_FLOW.quality=11;
//	OPTICAL_FLOW.time_usec=
  //*********************************************************************************************
//	BATT_STATUS.battery_function=MAV_BATTERY_FUNCTION_ALL;
//	BATT_STATUS.type       = MAV_BATTERY_TYPE_LIPO;
//	BATT_STATUS.voltages[1]= 10*(Bat_/BATT_Coef)/3;
//  BATT_STATUS.voltages[2]= 10*(Bat_/BATT_Coef)/3;
//  BATT_STATUS.voltages[3]= 10*(Bat_/BATT_Coef)/3;
//	BATT_STATUS.battery_remaining=100*((Bat_/BATT_Coef)/12.6);
//	BATT_STATUS.current_battery = 
//	BATT_STATUS.temperature=
//	BATT_STATUS.current_consumed=
//BATT_STATUS.energy_consumed =
  //***********************************************************************************************
//	ALTITUDE.altitude_local = Ultra.point ; 
//	ALTITUDE.altitude_monotonic = ALTITUDE_OFFSET;
//		ALTITUDE.altitude_amsl =Ultra.point;
//		ALTITUDE.altitude_relative=Ultra.point;
//		ALTITUDE.altitude_terrain = Ultra.point;
//		ALTITUDE.bottom_clearance = Ultra.point;
  //*********************************************************************************************
//    RC_RAW_PPM_CHANNELS.chan1_raw = RC.RC_channel[0];
//		RC_RAW_PPM_CHANNELS.chan2_raw = RC.RC_channel[1];
//		RC_RAW_PPM_CHANNELS.chan3_raw = RC.RC_channel[2];
//		RC_RAW_PPM_CHANNELS.chan4_raw = RC.RC_channel[3];
//		RC_RAW_PPM_CHANNELS.chan5_raw = RC.RC_channel[4];
//		RC_RAW_PPM_CHANNELS.chan6_raw = RC.RC_channel[5];
//		RC_RAW_PPM_CHANNELS.chan7_raw = RC.RC_channel[6];	 
//		RC_RAW_PPM_CHANNELS.chan8_raw = RC.RC_channel[7];	
//		RC_RAW_PPM_CHANNELS.chancount = 8;
//	RC_RAW_CHANNELS.rssi=
//	RC_RAW_CHANNELS.time_boot_ms = 
  //*********************************************************************************************
//		RC_CHANNELS_RAW.chan1_raw = RC.RC_channel[0];
//		RC_CHANNELS_RAW.chan2_raw = RC.RC_channel[1];
//		RC_CHANNELS_RAW.chan3_raw = RC.RC_channel[2];
//		RC_CHANNELS_RAW.chan4_raw = RC.RC_channel[3];
//		RC_CHANNELS_RAW.chan5_raw = RC.RC_channel[4];
//		RC_CHANNELS_RAW.chan6_raw = RC.RC_channel[5];
//		RC_CHANNELS_RAW.chan7_raw = RC.RC_channel[6];
//		RC_CHANNELS_RAW.chan8_raw = RC.RC_channel[7];
//  RC_CHANNELS_RAW.port      =
//	RC_CHANNELS_RAW.rssi      = 
//	RC_CHANNELS_RAW.time_boot_ms =
//***********************************************************************************************
//    RC_CHANNEL_SCALED.chan1_scaled = RC.RC_TRIM ;
//    RC_CHANNEL_SCALED.chan2_scaled = RC.HOV_THR ;
//	  RC_CHANNEL_SCALED.chan3_scaled = RC.RC_SW   ;
//		RC_CHANNEL_SCALED.chan4_scaled = RC.HOV_PIT ;
//		RC_CHANNEL_SCALED.chan5_scaled = RC.Yaw     ;
//		RC_CHANNEL_SCALED.chan6_scaled = RC.Roll    ;
//		RC_CHANNEL_SCALED.chan7_scaled = RC.Throttle;
//		RC_CHANNEL_SCALED.chan8_scaled = RC.Pitch   ;
//	RC_CHANNEL_SCALED.port         =
//	RC_CHANNEL_SCALED.rssi         =
//	RC_CHANNEL_SCALED.time_boot_ms =
  //**********************************************************************************************
		
	//payane meghdar dehi moteghayer haye mavlink*************************************************
	
	// Define the system type, in this case an airplane
	uint8_t system_type = MAV_TYPE_QUADROTOR;
	uint8_t autopilot_type = MAV_AUTOPILOT_GENERIC;
	 
	uint8_t system_mode = MAV_MODE_PREFLIGHT; ///< Booting up
	uint32_t custom_mode = 0;                 ///< Custom mode, can be defined by user/adopter
	uint8_t system_state = MAV_STATE_STANDBY; ///< System ready for flight
	 
	// Initialize the required buffers
	mavlink_message_t msg;
	
	mavlink_msg_heartbeat_pack(mavlink_system.sysid, mavlink_system.compid, &msg, system_type, autopilot_type, system_mode, custom_mode, system_state);
	// Copy the message to the send buffer
	len = mavlink_msg_to_send_buffer(buf, &msg);
	HAL_UART_Transmit(&huart1,(uint8_t*)buf,len,10);	
	
	// Pack the message
	mavlink_msg_heartbeat_pack(mavlink_system.sysid, mavlink_system.compid, &msg, system_type, autopilot_type, system_mode, custom_mode, system_state);
	// Copy the message to the send buffer
	len = mavlink_msg_to_send_buffer(buf, &msg);
	HAL_UART_Transmit(&huart4,(uint8_t*)buf,len,10);	
	//NrF_Fill_Data(&NRF,1,buf_2);	 CONTROL_SYSTEM_STATE
	//**************************************************************************************** 
//	mavlink_msg_highres_imu_encode(mavlink_system.sysid,mavlink_system.compid,&msg,&HIGHRES_IMU);
//	len = mavlink_msg_to_send_buffer(buf, &msg); 
//	HAL_UART_Transmit(&huart1,(uint8_t*)buf,len,10);
//	//**************************************************************************************** 
//	mavlink_msg_attitude_quaternion_encode(mavlink_system.sysid,mavlink_system.compid,&msg,&ATTITUDE_QUATERNION);
//	len = mavlink_msg_to_send_buffer(buf, &msg); 
//	HAL_UART_Transmit(&huart1,(uint8_t*)buf,len,10);
			//**************************************************************************************** 
//	mavlink_msg_scaled_imu_encode(mavlink_system.sysid,mavlink_system.compid,&msg,&SCALED_IMU);
//	len = mavlink_msg_to_send_buffer(buf, &msg); 
//	HAL_UART_Transmit(&huart1,(uint8_t*)buf,len,1);
//		//**************************************************************************************** 
//	mavlink_msg_local_position_ned_cov_encode(mavlink_system.sysid,mavlink_system.compid,&msg,&LOCAL_POSITION_NED_COV);
//	len = mavlink_msg_to_send_buffer(buf, &msg); 
//	HAL_UART_Transmit(&huart1,(uint8_t*)buf,len,1);
//	//**************************************************************************************** 
//	mavlink_msg_mission_count_encode(mavlink_system.sysid,mavlink_system.compid,&msg,&MISSION_COUNT);
//	len = mavlink_msg_to_send_buffer(buf, &msg); 
//	HAL_UART_Transmit(&huart1,(uint8_t*)buf,len,1);
//	//**************************************************************************************** 
//	mavlink_msg_mission_request_list_encode(mavlink_system.sysid,mavlink_system.compid,&msg,&MISSION_REQUEST_LIST);
//	len = mavlink_msg_to_send_buffer(buf, &msg); 
//	HAL_UART_Transmit(&huart1,(uint8_t*)buf,len,1);
//	//**************************************************************************************** 
//	mavlink_msg_mission_current_encode(mavlink_system.sysid,mavlink_system.compid,&msg,&MISSION_CURRENT);
//	len = mavlink_msg_to_send_buffer(buf, &msg); 
//	HAL_UART_Transmit(&huart1,(uint8_t*)buf,len,1);
//	//**************************************************************************************** 
//		 mavlink_msg_mission_set_current_encode(mavlink_system.sysid,mavlink_system.compid,&msg,&MISSION_SET_CURRENT);
//	 len = mavlink_msg_to_send_buffer(buf, &msg); 
//	 HAL_UART_Transmit(&huart1,(uint8_t*)buf,len,1);
//	//****************************************************************************************
//	 mavlink_msg_mission_request_encode(mavlink_system.sysid,mavlink_system.compid,&msg,&MISSION_REQUEST);
//	 len = mavlink_msg_to_send_buffer(buf, &msg); 
//	 HAL_UART_Transmit(&huart1,(uint8_t*)buf,len,1);	
//	 //****************************************************************************************
//	 mavlink_msg_mission_item_encode(mavlink_system.sysid,mavlink_system.compid,&msg,&MISSION_ITEM);
//	 len = mavlink_msg_to_send_buffer(buf, &msg); 
//	 HAL_UART_Transmit(&huart1,(uint8_t*)buf,len,1);	
//	 //****************************************************************************************
//	 mavlink_msg_manual_setpoint_encode(mavlink_system.sysid,mavlink_system.compid,&msg,&RC_SETPOINT);
//	 len = mavlink_msg_to_send_buffer(buf, &msg); 
//	 HAL_UART_Transmit(&huart1,(uint8_t*)buf,len,1);	
//	//********************************************************************************************
//	 mavlink_msg_sim_state_encode(mavlink_system.sysid,mavlink_system.compid,&msg,&SIM_STATE);
//	 len = mavlink_msg_to_send_buffer(buf, &msg); 
//	 HAL_UART_Transmit(&huart1,(uint8_t*)buf,len,1);	
	//********************************************************************************************
//	 mavlink_msg_attitude_encode(mavlink_system.sysid,mavlink_system.compid,&msg,&ATTITUDE);
//	 len = mavlink_msg_to_send_buffer(buf, &msg); 
//	 HAL_UART_Transmit(&huart1,(uint8_t*)buf,len,1);	
//	 //*******************************************************************************************
	 mavlink_msg_optical_flow_encode(mavlink_system.sysid,mavlink_system.compid,&msg,&OPTICAL_FLOW);
	 len = mavlink_msg_to_send_buffer(buf, &msg); 
	 HAL_UART_Transmit(&huart4,(uint8_t*)buf,len,1);	
//   //********************************************************************************************	 
//	 mavlink_msg_battery_status_encode(mavlink_system.sysid,mavlink_system.compid,&msg,&BATT_STATUS);
//	 len = mavlink_msg_to_send_buffer(buf, &msg); 
//	 HAL_UART_Transmit(&huart1,(uint8_t*)buf,len,1);	
//   //********************************************************************************************	 	 
//	 mavlink_msg_altitude_encode(mavlink_system.sysid,mavlink_system.compid,&msg,&ALTITUDE);
//	 len = mavlink_msg_to_send_buffer(buf, &msg); 
//	 HAL_UART_Transmit(&huart1,(uint8_t*)buf,len,1);	
//   //********************************************************************************************	 
//	 mavlink_msg_rc_channels_encode(mavlink_system.sysid,mavlink_system.compid,&msg,&RC_RAW_PPM_CHANNELS);
//	 len = mavlink_msg_to_send_buffer(buf, &msg); 
//	 HAL_UART_Transmit(&huart1,(uint8_t*)buf,len,1);	
//   //********************************************************************************************	 
//	 mavlink_msg_rc_channels_raw_encode(mavlink_system.sysid,mavlink_system.compid,&msg,&RC_CHANNELS_RAW);
//	 len = mavlink_msg_to_send_buffer(buf, &msg); 
//	 HAL_UART_Transmit(&huart1,(uint8_t*)buf,len,1);	
//   //********************************************************************************************	 
//	 mavlink_msg_rc_channels_scaled_encode(mavlink_system.sysid,mavlink_system.compid,&msg,&RC_CHANNEL_SCALED);
//	 len = mavlink_msg_to_send_buffer(buf, &msg); 
//	 HAL_UART_Transmit(&huart1,(uint8_t*)buf,len,1);	
//   //********************************************************************************************		 
//	 NrF_Fill_Data(&NRF,1,buf_2);
//	 
	// Send the message with the standard UART send function
	// uart0_send might be named differently depending on
	// the individual microcontroller / library in use.
	
}

	
void decode_mavlink_packet(uint8_t c,_MPC* Mpc)
{	

	// Try to get a new message
	
	
	if(mavlink_parse_char(MAVLINK_COMM_0, c, &msg, &status)) {
		//Mpc->ready=1;

			// Handle me,ssage
			switch(msg.msgid)
			{
				case MAVLINK_MSG_ID_HEARTBEAT:
				{	
							
					mavlink_heartbeat_t data;
						mavlink_msg_heartbeat_decode(&msg,&data);			
									
				}	
				break;
//				case MAV_CMD_DO_CHANGE_ALTITUDE :
					
			case MAVLINK_MSG_ID_COMMAND_LONG:					
				break;
			case MAVLINK_MSG_ID_OPTICAL_FLOW:
			{
				decode_flag_mavlink_usart++;	

				mavlink_msg_optical_flow_decode(&msg,&OPTICAL_FLOW);
			  mpc_data[0]= OPTICAL_FLOW.flow_x;  
		   	mpc_data[1]= OPTICAL_FLOW.flow_y;  
		  	quality= OPTICAL_FLOW.quality;
			//		Mpc->ready=0;
	} 
			
	   	break;
			
			
			
			default:				
					break;
			}
	}
}

void decode_mavlink_packet_2(uint8_t c,_MPC* Mpc)
{	

	// Try to get a new message
	
	
	if(mavlink_parse_char(MAVLINK_COMM_0, c, &msg, &status)) {
		//Mpc->ready=1;

			// Handle me,ssage
			switch(msg.msgid)
			{
				case MAVLINK_MSG_ID_HEARTBEAT:
				{	
							decode_flag_mavlink_usart++;	

					mavlink_heartbeat_t data;
						mavlink_msg_heartbeat_decode(&msg,&data);			
									
				}	
				break;
//				case MAV_CMD_DO_CHANGE_ALTITUDE :
					
			case MAVLINK_MSG_ID_COMMAND_LONG:					
				break;
			case MAVLINK_MSG_ID_OPTICAL_FLOW:
			{
				mavlink_msg_optical_flow_decode(&msg,&OPTICAL_FLOW);
			  mpc_data_2[0]= OPTICAL_FLOW.flow_x;  
		   	mpc_data_2[1]= OPTICAL_FLOW.flow_y;  
		  	quality= OPTICAL_FLOW.quality;
			//		Mpc->ready=0;
	} 
			
	   	break;
			
			
			
			default:				
					break;
			}
	}
}
void 	read_srf02(){		
									if(counter%22 == 0  ){
										uint8_t settings = SET_CM; 
										HAL_I2C_Mem_Write(&hi2c1, ADDRESS, REGISTER_ADDRESS, 1, &settings, 1, 100);	
										HAL_I2C_Mem_Read(&hi2c1, ADDRESS, H_BYTE, 1, &data1, 1, 100);				
										HAL_I2C_Mem_Read(&hi2c1, ADDRESS, L_BYTE, 1, &data2, 1, 100); 
										con.byte[0] = data2;
										con.byte[1] = data1;
										ULTRA_SRF02= con.real;
                                                            }
}
void SBUS_Packet_ground_mode(){
	           			SBUS_Channel_Data[0] =  994;//roll
									SBUS_Channel_Data[1] =  994; //pitch
									SBUS_Channel_Data[2] =  309; //throtle
									SBUS_Channel_Data[3] =  309; //yaw
									SBUS_Channel_Data[4] =  306;
									SBUS_Channel_Data[5] =  306; //sw
									SBUS_Channel_Data[6] =  306;
									SBUS_Channel_Data[7] =  306;  //thr

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

void SBUS_Packet_fly_mode(){
	           			SBUS_Channel_Data[0] =  RC.RC_channel[5] - Velocity.Y.Out_float;//roll
									SBUS_Channel_Data[1] =  RC.RC_channel[7] + Velocity.X.Out_float; //pitch
									SBUS_Channel_Data[2] =  RC.RC_channel[6]-9;// +  Altitude_Velocity.Out  ; //throtle
									SBUS_Channel_Data[3] =  RC.RC_channel[4]; //yaw
									SBUS_Channel_Data[4] =  RC.RC_channel[1];
									SBUS_Channel_Data[5] =  RC.RC_channel[2]; //sw
									SBUS_Channel_Data[6] =  RC.RC_channel[3];
									SBUS_Channel_Data[7] =  RC.RC_channel[0];  //thr
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


void MX_TIM14_Init(void)
{

  TIM_OC_InitTypeDef sConfigOC;

  htim14.Instance = TIM14;
  htim14.Init.Prescaler = (SystemCoreClock/(2*1000000)) - 1;
  htim14.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim14.Init.Period = 1000000/100 - 1;
  htim14.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  HAL_TIM_Base_Init(&htim14);

  HAL_TIM_PWM_Init(&htim14);

  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  HAL_TIM_PWM_ConfigChannel(&htim14, &sConfigOC, TIM_CHANNEL_1);

}

void MX_TIM13_Init(void)
{

  TIM_OC_InitTypeDef sConfigOC;

  htim13.Instance = TIM13;
  htim13.Init.Prescaler = SystemCoreClock/(2*1000000);
  htim13.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim13.Init.Period = 0xffff;
  htim13.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  HAL_TIM_Base_Init(&htim13);

  HAL_TIM_OC_Init(&htim13);

  sConfigOC.OCMode = TIM_OCMODE_TIMING;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  HAL_TIM_OC_ConfigChannel(&htim13, &sConfigOC, TIM_CHANNEL_1);

}

/* USER CODE END 4 */

#ifdef USE_FULL_ASSERT

/**
   * @brief Reports the name of the source file and the source line number
   * where the assert_param error has occurred.
   * @param file: pointer to the source file name
   * @param line: assert_param error line source number
   * @retval None
   */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
    ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */

}

#endif

/**
  * @}
  */ 

/**
  * @}
*/ 

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
