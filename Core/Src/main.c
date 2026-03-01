/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
#include "cmsis_os.h"
#include <string.h>
#include "FreeRTOS.h"
#include "cmsis_os2.h"
#include "semphr.h"
#include "stm32f4xx_hal_gpio.h"
#include "task.h"
#include <stdbool.h>

typedef enum{
  short_press = 1,
  double_press = 2,
  long_press = 3
}Button_state;

typedef enum {
  IDLE,
  BLINK_SLOW,
  BLINK_FAST,
  BREATHE,
  PANIC,
  ANY_STATE
}Led_state;

typedef void (*Led_output)(void);

typedef struct{
  Led_state Current_state;
  Button_state input;
  Led_state Next_state;
  Led_output action;
}Transition;

void Led_idle(void);
void Led_blink_slow(void);
void Led_blink_fast(void);
void Led_breathe(void);
void Led_panic(void);

Transition transition_table[] = { //таблиця переходів FSM
  {IDLE, short_press, BLINK_SLOW, Led_blink_slow},
  {BLINK_SLOW, short_press, BLINK_FAST, Led_blink_fast},
  {BLINK_FAST, short_press, BREATHE, Led_breathe},
  {BREATHE, short_press, IDLE, Led_idle},
  {ANY_STATE, double_press, IDLE, Led_idle},
  {ANY_STATE, long_press, PANIC, Led_panic}
};

#define Led_Pin GPIO_PIN_1 // LED
#define Button_Pin GPIO_PIN_2 // Кнопка
#define TIMER_BASE_FREQ 10000

UART_HandleTypeDef huart1;
TIM_HandleTypeDef htim2;
osSemaphoreId_t Button_semaphore;
osMessageQueueId_t Queue_FSM;
osMessageQueueId_t Queue_LED;
osThreadId_t ButtonTaskHandle;
osThreadId_t DispatchTaskHandle;
osThreadId_t LedTaskHandle;
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_TIM2_Init(void);

void ButtonTask(){
   Button_state state;
   while(1){
      if(osSemaphoreAcquire(Button_semaphore ,osWaitForever) == osOK) { // Очікує семафор від кнопки
        uint32_t start_time = HAL_GetTick();
        
        while(HAL_GPIO_ReadPin(GPIOA, Button_Pin) == GPIO_PIN_RESET){ // чекаємо поки користувач відіжме кнопку
          osDelay(10);
        }

        uint32_t duration = HAL_GetTick() - start_time; // Вимірюємо скільки часу кнопка була зажата

        if(duration > 2000){
          state = long_press;
          osMessageQueuePut(Queue_FSM, &state, 0, 0); // Надсилаємо в чергу 3(Long press)
        }
        else{
          bool second_ress = false;
          uint32_t wait_time = HAL_GetTick();

          while((uint32_t)(HAL_GetTick() - wait_time) < 400){
            if(HAL_GPIO_ReadPin(GPIOA,Button_Pin) == GPIO_PIN_RESET){ // перевірка на дабл клік
              second_ress = true;
              break;
            }
            osDelay(10);
          }

          if(second_ress){
            osSemaphoreAcquire(Button_semaphore, 0); // Ловимо семафор від подвійного натискання
            state = double_press;
            osMessageQueuePut(Queue_FSM, &state, 0, 0); // Надсилаємо в чергу 2(double press)
          }
          else{

            state = short_press;
            osMessageQueuePut(Queue_FSM, &state, 0, 0); // Надсилаємо в чергу 1(short press)
          }
        }
        osDelay(100);
      }
    }
}

void DispatchTask(void *argument) { 
  int transition_table_size = sizeof(transition_table) / sizeof(Transition);
  static Led_state real_current_state = IDLE;
  Button_state receivedEvent;

  for (;;) {
    if (osMessageQueueGet(Queue_FSM, &receivedEvent, NULL, osWaitForever) == osOK) {

      for (int i = 0; i < transition_table_size; i++) {
        
        if ((transition_table[i].Current_state == real_current_state || //Реалізація логіки FSM
             transition_table[i].Current_state == ANY_STATE) &&
            transition_table[i].input == receivedEvent) 
        {

          if (transition_table[i].action != NULL) {
             osMessageQueuePut(Queue_LED, &transition_table[i].action, 0, 0); // Відправляємо вказівник на функцію
          }

          real_current_state = transition_table[i].Next_state; // Оновлюємо стан

          break; 
        }
      }
    }
  }
}

void Blink_frequency(int frequency){
  uint32_t arr_value = TIMER_BASE_FREQ / frequency;
  __HAL_TIM_SET_AUTORELOAD(&htim2, arr_value);
  __HAL_TIM_SET_COUNTER(&htim2, 0);
  HAL_TIM_Base_Start_IT(&htim2);
    
}

void Led_idle(){
  HAL_TIM_Base_Stop_IT(&htim2); // Зупиняємо таймер
  HAL_GPIO_WritePin(GPIOA, Led_Pin, GPIO_PIN_RESET);
}

void Led_blink_slow(){
  Blink_frequency(1);
}

void Led_blink_fast(){
  Blink_frequency(10);
}

void Led_breathe(){
  HAL_TIM_Base_Stop_IT(&htim2);
}

void Led_panic(){
  Blink_frequency(30);
}

void LedTask(){
  Led_output action = NULL;
  for(;;){
    if(osMessageQueueGet(Queue_LED, &action, NULL, osWaitForever) == osOK){
      if(action != NULL){
        action();
      }
      osDelay(10);
    }
  }
}

void UartTask(){
  while(1){

    }
}

void StatsTask(){
  while(1){
          
    }
}

int main(void)
{
  /* 1. Насамперед — базовий HAL та тактування */
  HAL_Init();
  SystemClock_Config();

  /* 2. Тепер периферія */
  MX_GPIO_Init();
  MX_USART1_UART_Init();
  MX_TIM2_Init();
  /* 4. Ініціалізація RTOS */
  osKernelInitialize();
  
  Button_semaphore = osSemaphoreNew(1, 0, NULL);
  Queue_FSM = osMessageQueueNew(10, sizeof(Button_state), NULL);
  Queue_LED = osMessageQueueNew(10, sizeof(Led_output), NULL);

  const osThreadAttr_t ButtonTask_attributes = {
    .name = "ButtonTask",
    .stack_size = 2048,
    .priority = (osPriority_t) osPriorityAboveNormal,
  };
  const osThreadAttr_t DispatchTask_attributes = {
    .name = "Dispatch", 
    .stack_size = 2048, 
    .priority = osPriorityNormal
  };
  const osThreadAttr_t LedTask_attributes = {
    .name = "LedTask", 
    .stack_size = 2048, 
    .priority = osPriorityNormal
  };
  
  ButtonTaskHandle = osThreadNew(ButtonTask, NULL, &ButtonTask_attributes);
  DispatchTaskHandle = osThreadNew(DispatchTask, NULL, &DispatchTask_attributes);
  LedTaskHandle = osThreadNew(LedTask, NULL, &LedTask_attributes);
  osKernelStart();
  while(1){}
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 1599;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 9999;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  HAL_NVIC_SetPriority(TIM2_IRQn, 5, 0); // Пріоритет нижчий за системний тик
  HAL_NVIC_EnableIRQ(TIM2_IRQn);
  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_RESET);

  /*Configure GPIO pin : PA1 */
  GPIO_InitStruct.Pin = GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PA2 */
  GPIO_InitStruct.Pin = GPIO_PIN_2;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI2_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI2_IRQn);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin){
  if(GPIO_Pin == Button_Pin){
    static uint32_t last_interrupt_time = 0;
    uint32_t current_time = HAL_GetTick();
    // Якщо переривання прийшло швидше ніж через 60 мс — ігноруємо його
    if ((current_time - last_interrupt_time) > 60) {
      osSemaphoreRelease(Button_semaphore);
    }
    last_interrupt_time = current_time;
  }
}
/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* USER CODE BEGIN 5 */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END 5 */
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM1 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */
  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM1) {
    HAL_IncTick(); // Потрібно для FreeRTOS
  }
  // Додай цей блок:
  if (htim->Instance == TIM2) {
    HAL_GPIO_TogglePin(GPIOA, Led_Pin); 
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  Blink_frequency(20);
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
