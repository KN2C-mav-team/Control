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
#include "mpu6050.h" 
#include "EEPROM.h"
#include "RC.h"
#include "Control.h"
#include "Altitude.h"
#include "MPC.h"
#include "Odometery.h"
#include "par_optical.h"
//#include <mavlink/common/mavlink.h>
#include "stm32f4xx_it.h"

/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/

I2C_HandleTypeDef hi2c1;
I2C_HandleTypeDef hi2c3;

TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim13;

UART_HandleTypeDef huart4;
UART_HandleTypeDef huart3;
UART_HandleTypeDef huart2;
UART_HandleTypeDef huart1;

DMA_HandleTypeDef hdma_uart4_rx;
DMA_HandleTypeDef hdma_usart3_rx;
DMA_HandleTypeDef hdma_usart2_rx;
DMA_HandleTypeDef hdma_usart1_rx;

/* USER CODE BEGIN PV */
uint8_t data1 = 0; 
uint8_t data2 = 0; 
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
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
void MX_TIM13_Init(void);
/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

uint32_t Bat_= 0;	
int time=0,time_err=0,err_time=0;
int omid;
int counter_mpc_usart;
int moj_counter_usart_intrupt=0;
int clock_time=0;
int test;
uint64_t counter =0;					
int lock_time ;
char Run_State=GROUND_MODE;


/* USER CODE END 0 */

int main(void)
{

  /* USER CODE BEGIN 1 */
	
	
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
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_TIM3_Init();

  /* USER CODE BEGIN 2 */

	M_I2C_init(&hi2c1,I2C1,400000); 
	init_mpu(&Mpu,&hi2c1,0xD0,0); 

	//M_I2C_init(&hi2c1,I2C1,400000);   
	 
  MX_USART1_UART_Init(); 
 	MX_UART4_Init();
	MX_USART3_UART_Init();
	MX_USART2_UART_Init();
	MX_TIM13_Init();

	HAL_TIM_Base_Start_IT(&htim3);
	HAL_TIM_OC_Start_IT(&htim3,TIM_CHANNEL_1);
	
	
	//HAL_UART_Receive_DMA(&huart4,&station_data,1);
	HAL_UART_Receive_DMA(&huart3,SBUS.buffer,25);
	//HAL_UART_Receive_DMA(&huart2,+MPC_2.MPC_UART_BUFF_2,MPC_BUFF_AMOUNT_2); //window 
	HAL_UART_Receive_DMA(&huart4,MPC.MPC_UART_BUFF,MPC_BUFF_AMOUNT);//optical
	

	RC_Init(&RC,0);
	HAL_TIM_Base_Start(&htim13);
	

 

  /* USER CODE END 2 */
  /* USER CODE BEGIN 3 */
  /* Infinite loop */
	
  while (1)
  {
		
			__HAL_TIM_SetCounter(&htim13 , 0);
			clock_time++;
			temp=htim3.Instance->CNT;
			counter++;					
			lock_time  = 1;
		
	  	Update_IMU_Data_Mahony(&Mahony,&Mpu);
			
				
	    Read_Srf(htim3,&Ultra);
      
   	 	recieve_sbus_radio(&SBUS,&RC);		   
      		    
					
			if(counter%10 == 0 ){ 	
//			   Mpc_Empty_Data(&MPC);						
//    		 Mpc_Fill_Data(&MPC,1,(int)Ultra.point);
//  			 Mpc_Send_Data(&MPC);
				data1=10;
				HAL_UART_Transmit(&huart4,&data1,1, 100);
			 }
			 
       Check_MPC_IRQ(&MPC,&MPC_2)	;
			 do_optical_par(&MPC,&MPC_2,&Mpu,&optical_par,&Ultra);
				
		switch(Run_State)

		{
			case GROUND_MODE:
										
				SBUS_Packet_fly_mode();	
				if(clock_time%100 == 0 ){			
					HAL_GPIO_TogglePin(GPIOE,GPIO_PIN_11);
					HAL_GPIO_WritePin(GPIOE,GPIO_PIN_14,0);
				}
			
				if(Quad_On(RC) == 1) {
					Run_State=FLY_MODE;							
					control_init_(); 
					Set_zero_system_state();
					Run_Control = 0;
					HAL_GPIO_WritePin(GPIOE,GPIO_PIN_11,0);
					HAL_GPIO_WritePin(GPIOE,GPIO_PIN_14,1);
					
				}

				break;
			//*************************************************************************************
			case FLY_MODE:
			 if(Quad_Off(RC) == 1) Run_State=GROUND_MODE; 
			
			 if(clock_time%25 == 0 ){
				 HAL_GPIO_TogglePin(GPIOE,GPIO_PIN_14);
				 HAL_GPIO_TogglePin(GPIOE,GPIO_PIN_11);
			 }
			 

			Altitude_control(RC.THR_CUT);
			Velocity_Control(&Velocity);
			//Window_detection(&window_detection,&MPC);
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
		
		if(time>=DT_PULSE)	
		{
			time_err++;		
			err_time=time;
		}
		
		while(__HAL_TIM_GetCounter(&htim13) < DT_PULSE)
		{
			omid = __HAL_TIM_GetCounter(&htim13);
		}
		
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
  huart4.Init.BaudRate = 9600;
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
  huart2.Init.BaudRate = 57600;
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
  HAL_NVIC_SetPriority(DMA1_Stream5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream5_IRQn);
	

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


  GPIO_InitStruct.Pin = GPIO_PIN_6;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);


  /*Configure GPIO pin : PD1 */
  GPIO_InitStruct.Pin = GPIO_PIN_8;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
	
	  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);
	
	  /*Configure GPIO pin : PD0 */
  GPIO_InitStruct.Pin = GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);




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
					int c=0;
//		while(c<200){
//			c++;
//		HAL_GPIO_TogglePin(GPIOB,GPIO_PIN_6);
//		HAL_GPIO_TogglePin(GPIOB,GPIO_PIN_7);
//		HAL_Delay(2);
//		     }
//		__GPIOB_FORCE_RESET();
//		__GPIOB_RELEASE_RESET();

		
    hi2c->Instance = I2C1;
		__HAL_RCC_I2C1_CLK_ENABLE();		
		__HAL_RCC_GPIOB_CLK_ENABLE();
		GPIO_InitDef.Mode = GPIO_MODE_AF_OD;
		GPIO_InitDef.Pull = GPIO_NOPULL;
		GPIO_InitDef.Speed = GPIO_SPEED_HIGH;
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
		
		
		
		
		

  
    /**I2C1 GPIO Configuration    
    PB6     ------> I2C1_SCL
    PB7     ------> I2C1_SDA 
    */




	  }
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

			else if(huart->Instance == USART2 )
		{

				moj_counter_usart_intrupt++;
				MPC_2.UART_IRQ_FLAG	= 1;	
			

		}

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
	
	//if(GPIO_Pin == GPIO_PIN_8)
	
		if(Ultra.State == 1 && HAL_GPIO_ReadPin(GPIOB,GPIO_Pin) ==1)
		{
			Ultra.Begine = (uint16_t)htim3.Instance->CNT;
			Ultra.State = 2;
			
		}
		else if(Ultra.State == 2 && HAL_GPIO_ReadPin(GPIOB,GPIO_Pin) ==0)
		{
			Ultra.End = (uint16_t)htim3.Instance->CNT;
			Ultra.State = 0;
			Ultra.ready = 1;

		}
  
}


void MX_TIM13_Init(void)
{

  TIM_OC_InitTypeDef sConfigOC;

  htim13.Instance = TIM13;
  htim13.Init.Prescaler = 83;
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
