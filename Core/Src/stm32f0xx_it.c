/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    stm32f0xx_it.c
  * @brief   Interrupt Service Routines.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f0xx_it.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN TD */

/* USER CODE END TD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

volatile uint32_t pins;

/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/

/* USER CODE BEGIN EV */

/* USER CODE END EV */

/******************************************************************************/
/*           Cortex-M0 Processor Interruption and Exception Handlers          */
/******************************************************************************/
/**
  * @brief This function handles Non maskable interrupt.
  */
void NMI_Handler(void)
{
  /* USER CODE BEGIN NonMaskableInt_IRQn 0 */

  /* USER CODE END NonMaskableInt_IRQn 0 */
  /* USER CODE BEGIN NonMaskableInt_IRQn 1 */
   while (1)
  {
  }
  /* USER CODE END NonMaskableInt_IRQn 1 */
}

/**
  * @brief This function handles Hard fault interrupt.
  */
void HardFault_Handler(void)
{
  /* USER CODE BEGIN HardFault_IRQn 0 */

  /* USER CODE END HardFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_HardFault_IRQn 0 */
    /* USER CODE END W1_HardFault_IRQn 0 */
  }
}

/**
  * @brief This function handles System service call via SWI instruction.
  */
void SVC_Handler(void)
{
  /* USER CODE BEGIN SVC_IRQn 0 */

  /* USER CODE END SVC_IRQn 0 */
  /* USER CODE BEGIN SVC_IRQn 1 */

  /* USER CODE END SVC_IRQn 1 */
}

/**
  * @brief This function handles Pendable request for system service.
  */
void PendSV_Handler(void)
{
  /* USER CODE BEGIN PendSV_IRQn 0 */

  /* USER CODE END PendSV_IRQn 0 */
  /* USER CODE BEGIN PendSV_IRQn 1 */

  /* USER CODE END PendSV_IRQn 1 */
}

/**
  * @brief This function handles System tick timer.
  */
void SysTick_Handler(void)
{
  /* USER CODE BEGIN SysTick_IRQn 0 */

  /* USER CODE END SysTick_IRQn 0 */

  /* USER CODE BEGIN SysTick_IRQn 1 */

  /* USER CODE END SysTick_IRQn 1 */
}

/******************************************************************************/
/* STM32F0xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32f0xx.s).                    */
/******************************************************************************/

/**
  * @brief This function handles TIM1 break, update, trigger and commutation interrupts.
  */
void TIM1_BRK_UP_TRG_COM_IRQHandler(void)
{
  /* USER CODE BEGIN TIM1_BRK_UP_TRG_COM_IRQn 0 */

	if (LL_TIM_IsActiveFlag_UPDATE(TIM1))
	{
		LL_TIM_ClearFlag_UPDATE(TIM1);

		static int i = 0;

		// enable rows
		pins = DIODES_MASK;
		setDiodesState(pins);

		switch(i){
			case 0:
				selectBJT(BJT_0); // Maps to GPIO_ODR_10
				current_bjt = 0;
				LL_TIM_OC_SetCompareCH1(TIM1, brightness[0][0]);
				LL_TIM_OC_SetCompareCH2(TIM1, brightness[1][0]);
				LL_TIM_OC_SetCompareCH3(TIM1, brightness[2][0]);
				LL_TIM_OC_SetCompareCH4(TIM1, brightness[3][0]);
				break;
			case 1:
				selectBJT(BJT_1); // Maps to GPIO_ODR_11
				current_bjt = 1;
				LL_TIM_OC_SetCompareCH1(TIM1, brightness[0][1]);
				LL_TIM_OC_SetCompareCH2(TIM1, brightness[1][1]);
				LL_TIM_OC_SetCompareCH3(TIM1, brightness[2][1]);
				LL_TIM_OC_SetCompareCH4(TIM1, brightness[3][1]);
				break;
			case 2:
				selectBJT(BJT_2); // Maps to GPIO_ODR_12
				current_bjt = 2;
				LL_TIM_OC_SetCompareCH1(TIM1, brightness[0][2]);
				LL_TIM_OC_SetCompareCH2(TIM1, brightness[1][2]);
				LL_TIM_OC_SetCompareCH3(TIM1, brightness[2][2]);
				LL_TIM_OC_SetCompareCH4(TIM1, brightness[3][2]);
				break;
			case 3:
				selectBJT(BJT_3); // Maps to GPIO_ODR_13
				current_bjt = 3;
				LL_TIM_OC_SetCompareCH1(TIM1, brightness[0][3]);
				LL_TIM_OC_SetCompareCH2(TIM1, brightness[1][3]);
				LL_TIM_OC_SetCompareCH3(TIM1, brightness[2][3]);
				LL_TIM_OC_SetCompareCH4(TIM1, brightness[3][3]);
				break;
		}
		i++;
		i = i % 4;
	}


  /* USER CODE END TIM1_BRK_UP_TRG_COM_IRQn 0 */
  /* USER CODE BEGIN TIM1_BRK_UP_TRG_COM_IRQn 1 */

  /* USER CODE END TIM1_BRK_UP_TRG_COM_IRQn 1 */
}

/**
  * @brief This function handles TIM1 capture compare interrupt.
  */
void TIM1_CC_IRQHandler(void)
{
  /* USER CODE BEGIN TIM1_CC_IRQn 0 */

	if(LL_TIM_IsActiveFlag_CC1(TIM1)){
		LL_TIM_ClearFlag_CC1(TIM1);

		pins &= ~DIODE_0;
	}
	else if(LL_TIM_IsActiveFlag_CC2(TIM1)){
		LL_TIM_ClearFlag_CC2(TIM1);

		pins &= ~DIODE_1;
	}
	else if(LL_TIM_IsActiveFlag_CC3(TIM1)){
		LL_TIM_ClearFlag_CC3(TIM1);

		pins &= ~DIODE_2;
	}
	else if(LL_TIM_IsActiveFlag_CC4(TIM1)){
		LL_TIM_ClearFlag_CC4(TIM1);

		pins &= ~DIODE_3;
	}


	setDiodesState(pins);
  /* USER CODE END TIM1_CC_IRQn 0 */
  /* USER CODE BEGIN TIM1_CC_IRQn 1 */

  /* USER CODE END TIM1_CC_IRQn 1 */
}

/**
  * @brief This function handles TIM3 global interrupt.
  */
void TIM3_IRQHandler(void)
{
  /* USER CODE BEGIN TIM3_IRQn 0 */

	if (LL_TIM_IsActiveFlag_UPDATE(TIM3))
	{
		LL_TIM_ClearFlag_UPDATE(TIM3);
	}

  /* USER CODE END TIM3_IRQn 0 */
  /* USER CODE BEGIN TIM3_IRQn 1 */

  /* USER CODE END TIM3_IRQn 1 */
}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */
