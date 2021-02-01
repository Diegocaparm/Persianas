#include "main.h"
#include "stm32f4xx_it.h"
extern TIM_HandleTypeDef htim2;

void NMI_Handler(void)
{
  while (1)
  {
  }
}

void HardFault_Handler(void)
{
  while (1)
  {
  }
}


void MemManage_Handler(void)
{
  while (1)
  {
  }
}

void BusFault_Handler(void)
{
  while (1)
  {
  }
}

void UsageFault_Handler(void)
{
  while (1)
  {
  }
}

void SVC_Handler(void)
{
}

void DebugMon_Handler(void)
{
}

void PendSV_Handler(void)
{
}


void SysTick_Handler(void)
{
  HAL_IncTick();
}

void EXTI0_IRQHandler(void)
{
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_0); //Asignamos el pin PA0 al creador de interrupciones
}										//bandera que llevarán al cambio de modo

void TIM2_IRQHandler(void)
{
  HAL_TIM_IRQHandler(&htim2); 	//Asignamos el temporizador TIM2 al controlador de
}								//interrupciones periódicas
