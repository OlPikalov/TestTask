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
//#include "FreeRTOS.h"
//#include "cmsis_os2.h"
#include "semphr.h"
//#include "stm32_hal_legacy.h"
//#include "stm32f4xx_hal.h"
//#include "stm32f4xx_hal_gpio.h"
//#include "stm32f4xx_hal_tim.h"
//#include "stm32f4xx_hal_uart.h"
//#include "task.h"
#include <stdbool.h>
#include <stdio.h>
//#include <sys/_types.h>

void StartTimerForRunTimeStats(void);
uint32_t GetTimerForRunTimeStats(void);


uint8_t rx_data;          // Тимчасовий байт
char rx_buffer[20];       // Буфер для команди
int rx_index = 0;         //індекс буфера

typedef enum{
  short_press = 1,
  double_press = 2,
  long_press = 3
}Button_state; //Стани кнопок

typedef enum {
  IDLE,
  BLINK_SLOW,
  BLINK_FAST,
  BREATHE,
  PANIC,
  ANY_STATE
}Led_state; // Стани леду

typedef struct {
    Button_state state;
    TickType_t timestamp;
} Queue_FSM_data; // Структура данних для передачі в DispatchTask

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

static Led_state real_current_state = IDLE; // Поточний стан автомата

typedef struct {
  const char* command_name;
  void (*action)();
} Input_commands;

void status();
void tasks();
void drop_to_idle();
void panic();
void reset_stats();

Input_commands command_table[] = {
  {"status", status},
  {"tasks",tasks},
  {"drop_to_idle",drop_to_idle},
  {"panic",panic},
  {"reset_stats", reset_stats}
}; //Таблиця для опрацювання команд

// Гамма-коригована крива:
const uint32_t gamma_table[] = {
   0, 1, 2, 4, 7, 10, 14, 19, 25, 31, 38, 46, 55, 65, 75, 87, 99, 112, 126, 141,
    156, 172, 190, 207, 226, 245, 265, 286, 307, 329, 351, 374, 397, 421, 445, 470, 495, 520, 545, 570,
    596, 621, 646, 672, 696, 721, 745, 769, 792, 815, 837, 858, 879, 899, 918, 936, 953, 969, 983, 995,
    1006, 1016, 1025, 1033, 1040, 1045, 1050, 1054, 1057, 1059, 1060, 1060, 1059, 1057, 1054, 1050, 1045, 1040, 1033, 1025,
    1016, 1006, 998, 988, 977, 965, 952, 939, 924, 909, 893, 876, 858, 840, 821, 802, 782, 762, 741, 720,
    698, 677, 655, 633, 611, 589, 567, 545, 523, 501, 479, 458, 437, 416, 395, 375, 355, 335, 316, 297,
    279, 261, 244, 227, 211, 196, 181, 167, 154, 141, 129, 118, 108, 98, 89, 80, 72, 65, 58, 52,
    46, 41, 36, 31, 27, 23, 19, 16, 13, 11, 9, 7, 5, 4, 3, 2, 1, 1, 0, 0
};

#define GAMMA_SIZE (sizeof(gamma_table)/sizeof(gamma_table[0]))

#define Led_Pin GPIO_PIN_1 // LED
#define Button_Pin GPIO_PIN_2 // Кнопка
#define TIMER_BASE_FREQ 10000

UART_HandleTypeDef huart1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
DMA_HandleTypeDef hdma_tim2_ch2_ch4;
SemaphoreHandle_t Button_semaphore;
SemaphoreHandle_t  UART_INPUT_semaphore;
osMessageQueueId_t Queue_FSM;
osMessageQueueId_t Queue_LED;
osMessageQueueId_t Queue_UART;
osThreadId_t ButtonTaskHandle;
osThreadId_t DispatchTaskHandle;
osThreadId_t LedTaskHandle;
osThreadId_t UartTaskHandle;
osThreadId_t StatsTaskHandle;
GPIO_InitTypeDef GPIO_InitStruct = {0};
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_DMA_Init(void);
void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);
void Led_panic();

volatile uint32_t ulHighFrequencyTimerTicks = 0;

void StartTimerForRunTimeStats(void) {
    HAL_TIM_Base_Start_IT(&htim3); // Запуск таймера для ран тайм статистики
}

uint32_t GetTimerForRunTimeStats(void) {
    return ulHighFrequencyTimerTicks;
}



void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
    if (huart->Instance == USART1) {
      BaseType_t xHigherPriorityTaskWoken = pdFALSE;

        if(rx_data == '\n'){ //Перевірка на кінець повідомлення

          xSemaphoreGiveFromISR(UART_INPUT_semaphore, &xHigherPriorityTaskWoken); //Надаємо семафор FromISR
          if (xHigherPriorityTaskWoken == pdTRUE) {
            portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
          }
        }

        if (rx_index < sizeof(rx_buffer) - 1) { // залишаємо місце для \0
            rx_buffer[rx_index++] = rx_data;
            rx_buffer[rx_index] = '\0'; // Завжди тримаємо рядок валідним
        } else {
            rx_index = 0; // Захист від переповнення
        }
        HAL_UART_Receive_IT(&huart1, &rx_data, 1); //очікуємо новий байт
    }
}

void vApplicationStackOverflowHook(TaskHandle_t xTask, char *pcTaskName) {
    __disable_irq(); // Зупиняємо все, RTOS більше не працює
    
    char *failed_task_name = pcTaskName;
    while(1) {
        HAL_GPIO_TogglePin(GPIOA, Led_Pin);
        for(volatile int i = 0; i < 200000; i++); // Груба затримка
    }
}

void GPIO_LED_OUTPUT(){ //Конфігурація ЛЕД на OUTPUT_PP
  HAL_NVIC_DisableIRQ(EXTI2_IRQn);
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  GPIO_InitStruct.Pin = Led_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
  osDelay(10);
  HAL_NVIC_EnableIRQ(EXTI2_IRQn);
}

void ButtonTask(){
   Queue_FSM_data FSM_data;
   for(;;){
      if(osSemaphoreAcquire(Button_semaphore ,osWaitForever) == osOK) { // Очікує семафор від кнопки
        uint32_t start_time = HAL_GetTick();
        
        while(HAL_GPIO_ReadPin(GPIOA, Button_Pin) == GPIO_PIN_RESET){ // чекаємо поки користувач відіжме кнопку
          osDelay(10);
        }

        uint32_t duration = HAL_GetTick() - start_time; // Вимірюємо скільки часу кнопка була зажата

        if(duration > 2000){
          FSM_data.state = long_press;
          FSM_data.timestamp = xTaskGetTickCount();
          osMessageQueuePut(Queue_FSM, &FSM_data, 0, 0); // Надсилаємо в чергу 3(Long press)
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
            FSM_data.state = double_press;
            FSM_data.timestamp = xTaskGetTickCount();
            osMessageQueuePut(Queue_FSM, &FSM_data, 0, 0); // Надсилаємо в чергу 2(double press)
          }
          else{

            FSM_data.state = short_press;
            FSM_data.timestamp = xTaskGetTickCount();
            osMessageQueuePut(Queue_FSM, &FSM_data, 0, 0); // Надсилаємо в чергу 1(short press)
          }
        }
        while(osSemaphoreAcquire(Button_semaphore, 0) == osOK); // Ловимо фантомні натискання
        osDelay(100);
      }
    }
}

static int32_t max_latency = 0; // max latency

void DispatchTask(void *argument) { 
  int transition_table_size = sizeof(transition_table) / sizeof(Transition);
  Queue_FSM_data receivedEvent; // Вхідний сигнал від кнопки
  for (;;) {
    if (osMessageQueueGet(Queue_FSM, &receivedEvent, NULL, osWaitForever) == osOK) {
      for (int i = 0; i < transition_table_size; i++) {
        
        TickType_t current_tick = xTaskGetTickCount();
        int32_t latency = current_tick - receivedEvent.timestamp;

        if (latency > max_latency) {
            max_latency = latency; // Оновлюємо max latency
        }

        if ((transition_table[i].Current_state == real_current_state || //Реалізація логіки FSM
             transition_table[i].Current_state == ANY_STATE) &&
            transition_table[i].input == receivedEvent.state) 
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
  uint32_t arr_value = TIMER_BASE_FREQ / frequency; // рахуємо нове значення для Регістру автоперезавантаження
  GPIO_LED_OUTPUT();
  __HAL_TIM_SET_PRESCALER(&htim2, 1599);
  __HAL_TIM_SET_AUTORELOAD(&htim2, arr_value);
  __HAL_TIM_SET_COUNTER(&htim2, 0); // Обнуляємо таймер
  HAL_TIM_Base_Start_IT(&htim2); // Запускаємо таймер
  __HAL_TIM_ENABLE_IT(&htim2, TIM_IT_UPDATE);
    
}

void PWM_initialization(){
  __HAL_TIM_DISABLE_IT(&htim2, TIM_IT_UPDATE);
  __HAL_TIM_SET_PRESCALER(&htim2, 159);
  // 2. Перемикаємо пін
  HAL_TIM_MspPostInit(&htim2); //ініціалізуємо ЛЕД як AF_PP

  __HAL_TIM_SET_COUNTER(&htim2, 0);

  HAL_TIM_PWM_Start_DMA(&htim2, TIM_CHANNEL_2, gamma_table, GAMMA_SIZE); //Запускаємо PWM з DMA
}

void Led_idle() {
  HAL_TIM_PWM_Stop_DMA(&htim2, TIM_CHANNEL_2); //Коректна зупинка DMA
  HAL_TIM_Base_Stop_IT(&htim2);
  GPIO_LED_OUTPUT();
  HAL_GPIO_WritePin(GPIOA, Led_Pin, GPIO_PIN_RESET);
}

void Led_blink_slow(){
  Blink_frequency(1); // Задаємо частоту 1Гц
}

void Led_blink_fast(){
  Blink_frequency(10);// Задаємо частоту 10Гц
}

void Led_breathe(){
  PWM_initialization();
}

void Led_panic(){
  HAL_TIM_PWM_Stop_DMA(&htim2, TIM_CHANNEL_2); //Коректна зупинка DMA
  HAL_TIM_Base_Stop_IT(&htim2);
  GPIO_LED_OUTPUT();
  static char *msg = "\n\n\033[1;31m[ ! ] Program in panic mode, reset is imminent in 3, 2, 1 ...\033[0m\n\n";
  osMessageQueuePut(Queue_UART,&msg, 0, 0);
  Blink_frequency(20);// Задаємо частоту 20Гц
}

void LedTask(){
  Led_output action = NULL;
  for(;;){
    if(osMessageQueueGet(Queue_LED, &action, NULL, osWaitForever) == osOK){ // Отримуємо вказівнки на функцію через чергу
      if(action != NULL){
        action(); // Задаємо частоту для LED за допомогою вказівника на функцію
      }
      osDelay(10);
    }
  }
}

void drop_to_idle(){
  real_current_state = IDLE;
  Led_output func_ptr = Led_idle;
  osMessageQueuePut(Queue_LED, &func_ptr, 0, 0); // Відправляємо вказівник на функцію

  static char *msg = "\n[FSM] Forced transition to IDLE\n";
  osMessageQueuePut(Queue_UART,&msg, 0, 0);
  
}

void get_uptime_string(char *out_buffer) {
    TickType_t ticks = xTaskGetTickCount();
    uint32_t total_seconds = ticks / configTICK_RATE_HZ;
    
    uint32_t hours = total_seconds / 3600;
    uint32_t minutes = (total_seconds % 3600) / 60;
    uint32_t seconds = total_seconds % 60;
    
    sprintf(out_buffer, "%02lu:%02u:%02u", (unsigned long)hours, (unsigned int)minutes, (unsigned int)seconds);
}

const char* state_names[] = {
    "IDLE", "BLINK_SLOW", "BLINK_FAST", "BREATHE", "PANIC"
};

void status() {
    static char status_buffer[256];
    char uptime_str[16];
    
    // Отримуємо Uptime
    get_uptime_string(uptime_str);
    
    // Отримуємо вільну купу (Heap)
    size_t free_heap = xPortGetFreeHeapSize();
    
    snprintf(status_buffer, sizeof(status_buffer),
             "\r\n[ SYSTEM STATUS ]\r\n"
             "\r\nUptime: %s / State: %s / Heap free: %u B / Max latency: %u ms\r\n",
             uptime_str, 
             state_names[real_current_state], 
             (unsigned int)free_heap,
             (unsigned int)max_latency); //Формуємо оновне повідомлення
    
    char *ptr = status_buffer;
    osMessageQueuePut(Queue_UART, &ptr, 0, 10); //Відправляємо в чергу
}

#define MAX_TASKS 10
#define ROW_SIZE  64

void tasks() {
    static TaskStatus_t xTaskStatusArray[MAX_TASKS];
    static char table_rows[MAX_TASKS + 10][ROW_SIZE];
    UBaseType_t uxArraySize;
    uint32_t ulTotalRunTime;
    uint32_t row_idx = 0;

    uxArraySize = uxTaskGetSystemState(xTaskStatusArray, MAX_TASKS, &ulTotalRunTime);

    if (uxArraySize > 0) {

        snprintf(table_rows[row_idx], ROW_SIZE, "\r\n               Tasks Table\r\n");
        char *ptr = table_rows[row_idx++];
        osMessageQueuePut(Queue_UART, &ptr, 0, 10);

        snprintf(table_rows[row_idx], ROW_SIZE, "------------------------------------------\r\n");
        ptr = table_rows[row_idx++];
        osMessageQueuePut(Queue_UART, &ptr, 0, 10);

        // Заголовок колонок
        snprintf(table_rows[row_idx], ROW_SIZE, "Name         | St | Prio | HWM_Byte | ID\r\n");
        ptr = table_rows[row_idx++];
        osMessageQueuePut(Queue_UART, &ptr, 0, 10);

        // Дані тасків
        for (UBaseType_t x = 0; x < uxArraySize; x++) {
            char stateChar = ' ';
            switch (xTaskStatusArray[x].eCurrentState) {
                case eRunning:   stateChar = 'X'; break;
                case eReady:     stateChar = 'R'; break;
                case eBlocked:   stateChar = 'B'; break;
                case eSuspended: stateChar = 'S'; break;
                default:         stateChar = '?'; break;
            }

            snprintf(table_rows[row_idx], ROW_SIZE, "%-12s | %-2c | %-4lu | %-8lu | %-2lu\r\n",
                    xTaskStatusArray[x].pcTaskName, stateChar,
                    xTaskStatusArray[x].uxCurrentPriority,
                    (unsigned long)xTaskStatusArray[x].usStackHighWaterMark * 4,
                    xTaskStatusArray[x].xTaskNumber);

            ptr = table_rows[row_idx++];
            osMessageQueuePut(Queue_UART, &ptr, 0, 10);
            if (row_idx >= (MAX_TASKS + 8)) break; // Захист буфера
        }

        // HWM
        static char manual_hwm[128];
        snprintf(manual_hwm, sizeof(manual_hwm), 
                "------------------------------------------\r\n"
                "[HWM] Btn:%u Disp:%u Led:%u Uart:%u Stats:%u\r\n",
                (unsigned int)uxTaskGetStackHighWaterMark(ButtonTaskHandle),
                (unsigned int)uxTaskGetStackHighWaterMark(DispatchTaskHandle),
                (unsigned int)uxTaskGetStackHighWaterMark(LedTaskHandle),
                (unsigned int)uxTaskGetStackHighWaterMark(UartTaskHandle),
                (unsigned int)uxTaskGetStackHighWaterMark(StatsTaskHandle));
        char *m_ptr = manual_hwm;
        osMessageQueuePut(Queue_UART, &m_ptr, 0, 10);
    }
}

void panic(){
  real_current_state = PANIC;
  Led_output func_ptr = Led_panic;
  osMessageQueuePut(Queue_LED, &func_ptr, 0, 0); // Відправляємо вказівник на функцію
}

void reset_stats(){
  max_latency = 0; //Скидаємо max_latency
  static char *msg = "\nStats cleared\n";
  osMessageQueuePut(Queue_UART,&msg, 0, 0);
}

void UartTask() {
  char *msg_ptr;
  int command_table_size = sizeof(command_table) / sizeof(command_table[0]);

  for(;;) {
    if (osMessageQueueGet(Queue_UART, &msg_ptr, NULL, 100) == osOK) { //Вивід в юарт через чергу
      if (msg_ptr != NULL) {
        HAL_UART_Transmit(&huart1, (uint8_t*)msg_ptr, strlen(msg_ptr), 200);
      }
    }

    if (xSemaphoreTake(UART_INPUT_semaphore, 0) == pdTRUE) { //Обробка команд
      bool command_found = false;
      for (int i = 0; i < command_table_size; i++) {
        if (strstr(rx_buffer, command_table[i].command_name) != NULL) {
          if (command_table[i].action != NULL) {
            command_table[i].action();
          }
          command_found = true;
          break;
        }
      }
      
      if(!command_found) { // Невідома команда
        char *err = "\nUnknown command\r\n";
        HAL_UART_Transmit(&huart1, (uint8_t*)err, strlen(err), 100);
      }

      // Очищуємо буфер після обробки
      memset(rx_buffer, 0, sizeof(rx_buffer));
      rx_index = 0;
    }
  }
}
    

void StatsTask() {
    static char run_time_report[512]; // Буфер для рантайм статистики
    char *r_ptr = run_time_report;

    for (;;) {
        osDelay(10000); // Кожні 10 секунд

        // Очищуємо буфер
        memset(run_time_report, 0, sizeof(run_time_report));

        // Генеруємо звіт про використання процесора
        vTaskGetRunTimeStats(run_time_report);
      
        static char *h_ptr = "\r\n------- CPU Usage (Run Time Stats) -------\r\nTask\t\tAbs Time\tTime %\r\n";
        osMessageQueuePut(Queue_UART, &h_ptr, 0, osWaitForever);

        // Відправка рантайм статистики
        osMessageQueuePut(Queue_UART, &r_ptr, 0, osWaitForever);
        
        static char *f_ptr = "------------------------------------------\r\n";
        osMessageQueuePut(Queue_UART, &f_ptr, 0, osWaitForever);
    }
}
int main(void)
{
  // Ініціалізація HAL та тактування
  HAL_Init();
  SystemClock_Config();

  //Ініціалізація переферії
  MX_GPIO_Init();
  MX_USART1_UART_Init();
  MX_DMA_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();

  osKernelInitialize(); // Ініціалізуємо РТОС

  //Ініціалізація засобів IPC
  Button_semaphore = xSemaphoreCreateBinary();
  UART_INPUT_semaphore = xSemaphoreCreateBinary();
  Queue_FSM = osMessageQueueNew(10, sizeof(Queue_FSM_data), NULL);
  Queue_LED = osMessageQueueNew(10, sizeof(Led_output), NULL);
  Queue_UART = osMessageQueueNew(20,  sizeof(char*), NULL);

  HAL_UART_Receive_IT(&huart1, &rx_data, 1); // Ініціалізація роботи UART переривання

  // Атрибути створених тасок
  const osThreadAttr_t ButtonTask_attributes = {
    .name = "ButtonTask",
    .stack_size = 1024,
    .priority = (osPriority_t) osPriorityAboveNormal,
  };
  const osThreadAttr_t DispatchTask_attributes = {
    .name = "DispatchTask", 
    .stack_size = 1024, 
    .priority = osPriorityNormal
  };
  const osThreadAttr_t LedTask_attributes = {
    .name = "LedTask", 
    .stack_size = 1024, 
    .priority = osPriorityNormal
  };
  const osThreadAttr_t UartTask_attributes = {
    .name = "UartTask", 
    .stack_size = 1400, 
    .priority = osPriorityNormal
  };
  const osThreadAttr_t StatsTask_attributes = {
    .name = "StatsTask", 
    .stack_size = 1300, 
    .priority = osPriorityBelowNormal
  };

  HAL_GPIO_WritePin(GPIOA,Led_Pin, GPIO_PIN_RESET); // Ресет леда на початку програми

  //Ініціалізація тасок
  ButtonTaskHandle = osThreadNew(ButtonTask, NULL, &ButtonTask_attributes);
  UartTaskHandle = osThreadNew(UartTask, NULL, &UartTask_attributes);
  DispatchTaskHandle = osThreadNew(DispatchTask, NULL, &DispatchTask_attributes);
  LedTaskHandle = osThreadNew(LedTask, NULL, &LedTask_attributes);
  StatsTaskHandle = osThreadNew(StatsTask, NULL, &StatsTask_attributes);

  osKernelStart(); // Запускаємо РТОС


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

static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream6_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream6_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream6_IRQn);

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
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 159;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 999;
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
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK) {
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

  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */
  /* USER CODE END TIM2_Init 2 */
}


static void MX_TIM3_Init(void) {
    __HAL_RCC_TIM3_CLK_ENABLE();
    htim3.Instance = TIM3;
    htim3.Init.Prescaler = 15;
    htim3.Init.Period = 100;
    htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
    HAL_TIM_Base_Init(&htim3);
    
    HAL_NVIC_SetPriority(TIM3_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(TIM3_IRQn);
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
    
    // Debounce 60 мс
    if ((current_time - last_interrupt_time) > 60) {
      
      BaseType_t xHigherPriorityTaskWoken = pdFALSE;
      if (xSemaphoreGiveFromISR(Button_semaphore, &xHigherPriorityTaskWoken) == pdTRUE) {
        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
      }
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
    HAL_IncTick(); 
  }

  if (htim->Instance == TIM2) {

    uint32_t pclk = HAL_RCC_GetPCLK1Freq();

    uint32_t current_psc = htim2.Instance->PSC;
    uint32_t current_arr = htim2.Instance->ARR;

    float current_freq = (float)pclk / ((current_psc + 1) * (current_arr + 1));

    if ((GPIOA->MODER & (GPIO_MODER_MODER1_Msk)) == (GPIO_MODER_MODER1_0)) {
      HAL_GPIO_TogglePin(GPIOA, Led_Pin);
    }
    static uint32_t First_Timer_callback = 0;
    if (current_freq > 19.5f && current_freq < 20.5f) { // Перевірка на стан PANIC

      if (First_Timer_callback == 0) {
        First_Timer_callback = HAL_GetTick(); 
      }

      if ((HAL_GetTick() - First_Timer_callback) > 3000) { //Пройшло 3 секунди виконуєм ресет
        taskDISABLE_INTERRUPTS();
        __disable_irq();
        NVIC_SystemReset();
      }
            
      
    }
    else First_Timer_callback = 0;
  }

  if (htim->Instance == TIM3) {
        ulHighFrequencyTimerTicks++;
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
