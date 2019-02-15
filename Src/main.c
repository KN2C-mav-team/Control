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
#include "Optical_Flow.h"


/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim14;
TIM_HandleTypeDef htim13;

UART_HandleTypeDef huart4;
UART_HandleTypeDef huart1;
DMA_HandleTypeDef hdma_uart4_rx;
DMA_HandleTypeDef hdma_usart1_rx;

/* USER CODE BEGIN PV */
I2C_HandleTypeDef hi2c3;
int Led_Level=0;
void Ping_RC_IMU(MPU_SENSOR *sen,_RC *Rc);
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC1_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_UART4_Init(void);
static void MX_USART1_UART_Init(void);

/* USER CODE BEGIN PFP */
void M_I2C_init(I2C_HandleTypeDef *hi2c,I2C_TypeDef* I2Cx, uint32_t clockSpeed);
HAL_StatusTypeDef M_I2C_Init(I2C_HandleTypeDef *hi2c);
static void MX_TIM14_Init(void);
void MX_TIM13_Init(void);
/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

uint32_t Bat_=0;	
int time=0,time_err=0,err_time=0;
int omid;
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
  MX_I2C1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_UART4_Init();
  MX_USART1_UART_Init();

  /* USER CODE BEGIN 2 */
	MX_TIM14_Init();
	MX_TIM13_Init();
	
	M_I2C_init(&hi2c3,I2C3,400000);
	HAL_TIM_Base_Start_IT(&htim3);
	HAL_TIM_OC_Start_IT(&htim3,TIM_CHANNEL_1);
	HAL_UART_Receive_DMA(&huart1,&station_data,1);
	HAL_UART_Receive_DMA(&huart4,MPC.MPC_UART_BUFF,MPC_BUFF_AMOUNT);
	HAL_ADC_Start_DMA(&hadc1,&Bat_,1);
	

	
	init_mpu(&Mpu,&hi2c1,0xD0,0);	
	MS5611_init(&MS,&hi2c1); //*
	Nrf_Init(&NRF,hi2c3,15);
	RC_Init(&RC,hi2c3,0);
	Pwm_frq(&htim2,600,2048);
	//	Pwm_frq(&htim14,1200,2048);
	Servo_init(&htim14,100);
		
	//Servo_Set_Angle(&htim14,35);
	//HAL_Delay(2000)
	Servo_Set_Angle(&htim14,0);	//Zero Degree
	//HAL_Delay(2000);
	//Servo_Set_Angle(&htim14,-65); //80 Degree 
	
	
	HAL_TIM_Base_Start(&htim14);
	HAL_TIM_PWM_Start(&htim14,TIM_CHANNEL_1);
	Ping_RC_IMU(&Mpu,&RC);
	
	HAL_TIM_Base_Start(&htim13);
	
	
	
	Mahony.q0=1.0f;
	z_vel.state=0;
	z_vel.C=1;
	z_vel.A=1;
	z_vel.B=DT;
	z_vel.P=1;
	z_vel.need_2_res =0;
	
	MPC.ready = 0;
	MPU_Hist.counter = 0;
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
					__HAL_TIM_SetCounter(&htim13,0);
		
					Check_MPC_IRQ(&MPC);
//      lock time			
//			if(lock_time == 0)
//			{
					
				  temp=htim3.Instance->CNT;
					counter++;					
					lock_time  = 1;
					
				  Check_battery((counter%100),Run_State);
				
					//      Sen
					MS5611_Read(&MS);
					RC_Read(&RC,counter);
				
					//      IMU
					Update_IMU_Data_Mahony(&Mahony,&Mpu);

	
					//PTAM_get_data(&MPC,&Cam_Position,&Mpu);
					//Correct_Cam_POS(&Cam_Position);
					
					do_optical(&MPC,&optical,&Mpu);
					
					Nrf_(&NRF,counter);
				
					Rc2Controller(RC);
					Point2Controller(Mahony,Mpu);
					//       print 
					Read_Srf(htim3,&Ultra);

					
				
					Led_Beat(Run_State,counter);
					if(counter%10 == 0 )
					{	
						Mpc_Empty_Data(&MPC);
						//Mpc_Fill_Data(&MPC,3,(int)(Ultra.point*100),(int)(100*Cam_Position.Modified_POS_Z),(int)(10*(Bat_/21.6)));
						Mpc_Fill_Data(&MPC,2,(int)(Ultra.point),(int)(10*(Bat_/21.6)));

						Mpc_Send_Data(&MPC);						
						
						//Station_Data_R();
						//HAL_Delay(3);												
					}		
				
				
					switch(Run_State)
					{
						case GROUND_MODE:
							
							if(Quad_On(RC) == 1)
                 Run_State=READY_2_FLY;
							
							Scaling_Cam_POS(&Cam_Position,Ultra.point);							
							//control_init_();
							
						 Motor(MIN_Motor_Speed,0,0,0,0);
				//		Motor(MIN_Motor_Speed +  RC.Throttle,0,0,0,0);  
							break;
						//*************************************************************************************
						case READY_2_FLY:
							
							if((float)RC.Throttle > (0.1f*(float)Throttle_range))
							{
                     Run_State=FLY_MODE;
										 control_init_(); // test zaraieb
										 Set_zero_system_state();//*
										 Run_Control = 0;
							}
			
							if(Quad_Off(RC) == 1)
                     Run_State=GROUND_MODE;
							
						
							Motor(RDY_Motor_Speed,0,0,0,0);
					//	Motor( MIN_Motor_Speed,0,0,0,0);  //**//   
							
							break;
						//*************************************************************************************
						case FLY_MODE:
							
							if(Quad_Off(RC) == 1)
                     Run_State=GROUND_MODE;
			 
							if( ((float)RC.Throttle < (0.1f*(float)Throttle_range))  &&  (RC.RC_SW==0) )
								{
										 off_delay_timer++;
										 if( off_delay_timer> 100 )
											 {
													Run_State=GROUND_MODE; 
													off_delay_timer=0; 
											 }
								}
							else
										 off_delay_timer=0;
							
							
							if( (fabs(Roll.point-Roll.offset)>1.5 || fabs(Pitch.point-Pitch.offset)>1.5)  &&  Run_Control==0)
							  {
									control_init_();
									Run_Control = 1;
			          } 
																														
							First_Person_control(&Roll,&Pitch,&Yaw, -90.0f);	
							//Third_Person_control(&Roll,&Pitch,&Yaw);	
								
							// ** Attention **// This programm Change Altitude_Velocity Set point if THR_CUT Enable
							//**Position_Control(&Position,&Roll,&Pitch,&Yaw);
							//Velocity_Control(&Velocity,&Roll,&Pitch);
		
							
							Control(&Roll);
							Control(&Pitch);
							Control(&Yaw);							
							
							// ** Attention **// One off the Control_Altitude Or Control_Altitude_Velocity must be enable!!!
							//Control_Altitude(RC.RC_SW);																					
							Control_Altitude_Velocity(RC.RC_SW);									
								
				

               
						 Motor( Motor_force + RDY_Motor_Speed + Initial_Mass_Force ,Roll.Out,Pitch.Out,Yaw.Out,0);  //**// 
	          // Motor( MIN_Motor_Speed,0,0,0,0);					 
					 //  Motor( 2000,0,0,0,0);  //**// 	
								//Servo_Set_Angle(&htim14,(int)(90.0f*(RC.HOV_PIT)));
							
							break;
						//*************************************************************************************
						default:
							Run_State = GROUND_MODE;
							break;
					}
					Check_MPC_IRQ(&MPC);//
					
					
					if(htim3.Instance->CNT > temp)
						time = htim3.Instance->CNT-temp;
					else
						time = ( htim3.Instance->CNT - temp) +0xffff;
					
					if(time>=4000)	
					{
						time_err++;		
						err_time=time;
					}
					
					while(__HAL_TIM_GetCounter(&htim13) < DT_PULSE)
					{
						omid = __HAL_TIM_GetCounter(&htim13);
					}

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
void MX_I2C1_Init(void)
{

  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 400000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_16_9;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_10BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLED;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLED;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLED;
  HAL_I2C_Init(&hi2c1);

}

/* TIM2 init function */
void MX_TIM2_Init(void)
{

  TIM_ClockConfigTypeDef sClockSourceConfig;
  TIM_MasterConfigTypeDef sMasterConfig;
  TIM_OC_InitTypeDef sConfigOC;

  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 0;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  HAL_TIM_Base_Init(&htim2);

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig);

  HAL_TIM_PWM_Init(&htim2);

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_OC1;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig);

  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1);

  HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_2);

  HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_3);

  HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_4);

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
  huart4.Init.BaudRate = 57600;
  huart4.Init.WordLength = UART_WORDLENGTH_8B;
  huart4.Init.StopBits = UART_STOPBITS_1;
  huart4.Init.Parity = UART_PARITY_NONE;
  huart4.Init.Mode = UART_MODE_TX_RX;
  huart4.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart4.Init.OverSampling = UART_OVERSAMPLING_16;
  HAL_UART_Init(&huart4);

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
  /* DMA controller clock enable */
  __DMA2_CLK_ENABLE();
  __DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  HAL_NVIC_SetPriority(DMA2_Stream2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream2_IRQn);
  HAL_NVIC_SetPriority(DMA1_Stream2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream2_IRQn);

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
	if (I2Cx == I2C1) 
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
		
  I2C_InitStruct.DutyCycle = I2C_DUTYCYCLE_2;
  I2C_InitStruct.OwnAddress1 = 0;
  I2C_InitStruct.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  I2C_InitStruct.DualAddressMode = I2C_DUALADDRESS_DISABLED;
  I2C_InitStruct.OwnAddress2 = 0;
  I2C_InitStruct.GeneralCallMode = I2C_GENERALCALL_DISABLED;
  I2C_InitStruct.NoStretchMode = I2C_NOSTRETCH_DISABLED;
	hi2c->Init = I2C_InitStruct;
	hi2c->State =  HAL_I2C_STATE_BUSY;
	}
	else if (I2Cx == I2C2) 
	{
		__GPIOF_FORCE_RESET();
		__GPIOF_RELEASE_RESET();
		hi2c->Instance = I2C2;
		__HAL_RCC_I2C2_CLK_ENABLE();
		__HAL_RCC_GPIOF_CLK_ENABLE();
		GPIO_InitDef.Mode = GPIO_MODE_AF_OD;
		GPIO_InitDef.Pull = GPIO_PULLUP;
		GPIO_InitDef.Speed = GPIO_SPEED_MEDIUM;
		GPIO_InitDef.Pin = GPIO_PIN_0 | GPIO_PIN_1;
		GPIO_InitDef.Alternate = GPIO_AF4_I2C2;
		HAL_GPIO_Init(GPIOF,&GPIO_InitDef);
		
  I2C_InitStruct.DutyCycle = I2C_DUTYCYCLE_2;
  I2C_InitStruct.OwnAddress1 = 0;
  I2C_InitStruct.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  I2C_InitStruct.DualAddressMode = I2C_DUALADDRESS_DISABLED;
  I2C_InitStruct.OwnAddress2 = 0;
  I2C_InitStruct.GeneralCallMode = I2C_GENERALCALL_DISABLED;
  I2C_InitStruct.NoStretchMode = I2C_NOSTRETCH_DISABLED;
	hi2c->Init = I2C_InitStruct;
	hi2c->State =  HAL_I2C_STATE_BUSY;
	}
	else if (I2Cx == I2C3) 
	{
//		__GPIOA_CLK_ENABLE();
//		__GPIOC_CLK_ENABLE();
//		__GPIOC_FORCE_RESET();
//		__GPIOA_FORCE_RESET();
//		__GPIOC_RELEASE_RESET();
//		__GPIOA_RELEASE_RESET();
		hi2c->Instance = I2C3;
		__HAL_RCC_I2C3_CLK_ENABLE();
		__HAL_RCC_GPIOA_CLK_ENABLE();
		__HAL_RCC_GPIOC_CLK_ENABLE();
		GPIO_InitDef.Mode = GPIO_MODE_AF_OD;
		GPIO_InitDef.Pull = GPIO_PULLUP;
		GPIO_InitDef.Speed = GPIO_SPEED_MEDIUM;
		GPIO_InitDef.Pin = GPIO_PIN_8;
		GPIO_InitDef.Alternate = GPIO_AF4_I2C3;
		HAL_GPIO_Init(GPIOA,&GPIO_InitDef);
		GPIO_InitDef.Pin = GPIO_PIN_9;
		HAL_GPIO_Init(GPIOC,&GPIO_InitDef);
		
  I2C_InitStruct.DutyCycle = I2C_DUTYCYCLE_16_9;
  I2C_InitStruct.OwnAddress1 = 0;
  I2C_InitStruct.AddressingMode = I2C_ADDRESSINGMODE_10BIT;
  I2C_InitStruct.DualAddressMode = I2C_DUALADDRESS_DISABLED;
  I2C_InitStruct.OwnAddress2 = 0;
  I2C_InitStruct.GeneralCallMode = I2C_GENERALCALL_DISABLED;
  I2C_InitStruct.NoStretchMode = I2C_NOSTRETCH_DISABLED;
	hi2c->Init = I2C_InitStruct;
	hi2c->State =  HAL_I2C_STATE_BUSY;
	}
	M_I2C_Init(hi2c);
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
	
//	if(huart->Instance == UART4)
//	{	
	
//	if(CAM_IRQ_Flag)
//	{
			if(huart->Instance == UART4 )
		{
			MPC.UART_IRQ_FLAG	= 1;
		}else if(huart->Instance == USART1)
		{
			
			data_r = 1;
			if(station_data == 's')
				RC.init = 0;
			//HAL_UART_Transmit(&huart1,&rxbuff,1,100);
			//HAL_UART_Receive_IT(&huart1,(uint8_t *)aRxBuffer,1);
		}
//	}
	
}


void HAL_I2C_MasterRxCpltCallback(I2C_HandleTypeDef *hi2c)
{

		LEDY_TGL;
	
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
		if((12-(Bat_/216.0f))>0)
			LED_BAT_on_time=(unsigned int)((100*(12-(Bat_/216.0f))));
		else LED_BAT_on_time=0;
		
		if(System_counter < LED_BAT_on_time)	
			LEDW_ON;
		else LEDW_OFF;		
	}
	else 
	{
		if((11.5f-(Bat_/216.0f))>0)
			LED_BAT_on_time=(unsigned int)((200*(11.5f-(Bat_/216.0f))));
		else LED_BAT_on_time=0;
		
		if(System_counter < LED_BAT_on_time)	
			LEDW_ON;
		else LEDW_OFF;
	}
	
}

void Ping_RC_IMU(MPU_SENSOR* sen,_RC* Rc)
{
	uint8_t state =0,data[20];
	while(state < 3)
	{
		
		HAL_Delay(300);
		
		state = 0;
		
		if(ping_mpu(sen) == 104)
			state++;
		else
		{
			LEDG_TGL;
			LEDW_TGL;
			
			Pwm_set( &htim2, 0 , _MRU );
			Pwm_set( &htim2, 0 , _MLU );
			Pwm_set( &htim2, 0 , _MRD );
			Pwm_set( &htim2, 0 , _MLD );
		}
		data[19]=1;
		if( HAL_I2C_Master_Transmit(&Rc->I2C,30,(uint8_t*)&data[19],1,1) == HAL_OK)
			{
					HAL_I2C_Master_Receive(&Rc->I2C,30, (uint8_t *)&data[0],16,1);
  				state++;		
			}	
		else
		{
			LEDB_TGL;
			LEDY_TGL;
			
			Pwm_set( &htim2, 0 , _MRU );
			Pwm_set( &htim2, 0 , _MLU );
			Pwm_set( &htim2, 0 , _MRD );
			Pwm_set( &htim2, 0 , _MLD );
		}
		if( (Roll.error_coeficient==1) || (Pitch.error_coeficient==1) || (Yaw.error_coeficient==1) || (Altitude.error_coeficient==1) )
		{
			LEDW_TGL;
			LEDY_TGL;
			
			Pwm_set( &htim2, 0 , _MRU );
			Pwm_set( &htim2, 0 , _MLU );
			Pwm_set( &htim2, 0 , _MRD );
			Pwm_set( &htim2, 0 , _MLD );
		}
		else
			state++;	
		
	}
	LEDY_OFF;
	LEDG_OFF;
	LEDB_OFF;
	LEDW_OFF;
	init_mpu(&Mpu,&hi2c1,0xD0,0);
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
