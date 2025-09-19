/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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
#include "usart.h"
#include "gpio.h"
#include <stdio.h>
#include <string.h>
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef enum {
  STATE_ACTIVE,
  STATE_STOPPED
} SystemState_t;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
volatile uint8_t g_wake_up_request = 0;
volatile uint8_t g_go_to_sleep_request = 0;

SystemState_t current_state = STATE_ACTIVE;

// Buffer para o CLI
uint8_t cli_buffer[10];
uint8_t cli_idx = 0;
uint8_t g_uart_rx_char; 
/* USER CODE END PV */

#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)

PUTCHAR_PROTOTYPE
{
  HAL_UART_Transmit(&huart2, (uint8_t *)&ch, 1, HAL_MAX_DELAY);
  return ch;
}

void process_cli(void) {
  if (__HAL_UART_GET_FLAG(&huart2, UART_FLAG_RXNE) == SET) {
    HAL_UART_Receive(&huart2, &cli_buffer[cli_idx], 1, 1);
    if (cli_buffer[cli_idx] == '\r' || cli_buffer[cli_idx] == '\n') {
      cli_buffer[cli_idx] = '\0'; // Termina a string
      if (strcmp((char*)cli_buffer, "OFF") == 0) {
        printf("Comando 'OFF' recebido. Entrando em modo Stop...\r\n");
        g_go_to_sleep_request = 1;
      }
      cli_idx = 0; // Reseta o buffer
    } else {
      if (cli_idx < sizeof(cli_buffer) - 1) {
        cli_idx++;
      }
    }
  }
}
/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */
	printf("Sistema iniciado. Display LIGADO.\r\n");
	HAL_UART_Receive_IT(&huart2, &g_uart_rx_char, 1);
  /* USER CODE END 2 */

  // 2. Desliga o circuito de detecção de toque
 
  
  current_state = STATE_ACTIVE;
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */
		if (current_state == STATE_ACTIVE) {
      // --- MODO ATIVO ---
      HAL_GPIO_TogglePin(LED_STATUS_GPIO_Port, LED_STATUS_Pin);
      HAL_Delay(500); // Pisca o LED para indicar que está ativo

      // Verifica por comandos CLI
      process_cli();

      // Verifica se o botão de dormir foi pressionado
      if (g_go_to_sleep_request) {
        g_go_to_sleep_request = 0; // Limpa a flag
        current_state = STATE_STOPPED; // Muda para o estado de parada
      }

    } else if (current_state == STATE_STOPPED) {
      // --- SEQUÊNCIA PARA ENTRAR EM MODO STOP ---
      printf("Entrando em modo Stop. Toque na tela para acordar.\r\n");
      
      // Garante que a mensagem foi enviada
      HAL_Delay(100); 

      // 1. Desliga o LED de status
      HAL_GPIO_WritePin(LED_STATUS_GPIO_Port, LED_STATUS_Pin, GPIO_PIN_SET);
      
      // 2. Desliga a alimentação do display
      HAL_GPIO_WritePin(DISPLAY_PWR_CTRL_GPIO_Port, DISPLAY_PWR_CTRL_Pin, GPIO_PIN_RESET);
      
      // 3. Habilita o circuito de detecção de toque (ligando seu GND)
      HAL_GPIO_WritePin(HAB_TOUCH_GPIO_Port, HAB_TOUCH_Pin, GPIO_PIN_RESET);
      
      // Limpa a flag de wake-up para garantir que não acordemos imediatamente
      __HAL_PWR_CLEAR_FLAG(PWR_FLAG_WUF1); // Limpa as flags de wake-up
      g_wake_up_request = 0;

      // ********** ENTRANDO EM MODO STOP **********
      HAL_PWR_EnterSTOPMode(PWR_MAINREGULATOR_ON, PWR_STOPENTRY_WFI);
      // *******************************************
      
      // --- PONTO DE RETORNO APÓS ACORDAR ---

      // ** CRÍTICO: ** Reconfigura o clock do sistema
      SystemClock_Config();
      
      // --- SEQUÊNCIA AO ACORDAR ---
      // 1. Desabilita o circuito de detecção de toque para economizar energia e evitar interrupções
      HAL_GPIO_WritePin(HAB_TOUCH_GPIO_Port, HAB_TOUCH_Pin, GPIO_PIN_SET);
      
      // 2. Liga a alimentação do display
      HAL_GPIO_WritePin(DISPLAY_PWR_CTRL_GPIO_Port, DISPLAY_PWR_CTRL_Pin, GPIO_PIN_SET);
      
      printf("\r\nSistema acordou do modo Stop!\r\n");

      // Muda para o estado ativo
      current_state = STATE_ACTIVE;
    }
    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance == USART2)
  {
    // Verifica se o caractere é um fim de linha (Enter)
    if (g_uart_rx_char == '\r' || g_uart_rx_char == '\n') 
    {
      if (cli_idx > 0) // Só processa se tivermos recebido algo
      { 
        cli_buffer[cli_idx] = '\0'; // Finaliza a string

        // Converte para maiúsculas (para aceitar 'off', 'OFF', 'Off', etc.)
        for (int i = 0; cli_buffer[i] != '\0'; i++) {
          if (cli_buffer[i] >= 'a' && cli_buffer[i] <= 'z') {
            cli_buffer[i] = cli_buffer[i] - 32;
          }
        }

        // Compara o comando
        if (strcmp((char*)cli_buffer, "OFF") == 0) 
        {
          // ATENÇÃO: Estamos em um contexto de interrupção.
          // Não podemos usar printf() ou HAL_Delay() aqui. Apenas definimos a flag.
          // O loop principal (no estado ATIVO) verá esta flag e fará o resto.
          if (current_state == STATE_ACTIVE) {
             g_go_to_sleep_request = 1;
          }
        }
      }
      cli_idx = 0; // Reseta o índice do buffer
    } 
    else 
    {
      // Adiciona caractere ao buffer se não for um fim de linha
      if (cli_idx < sizeof(cli_buffer) - 1) 
      {
        cli_buffer[cli_idx++] = g_uart_rx_char;
      }
    }
    
    // Re-arma a interrupção para receber o próximo byte
    HAL_UART_Receive_IT(&huart2, &g_uart_rx_char, 1);
  }
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
  if (GPIO_Pin == GPIO_PIN_13) { // Botão B1 (PC13) foi pressionado
    // Este botão só tem efeito se estivermos no modo ATIVO
    if (current_state == STATE_ACTIVE) {
      g_go_to_sleep_request = 1;
    }
  } 
  else if (GPIO_Pin == GPIO_PIN_7) { // Sinal de wake-up (PC7) foi ativado
    // Este sinal só nos interessa quando estamos para entrar ou já em modo STOP.
    // Apenas definimos a flag, o processamento ocorre no loop principal ao acordar.
    g_wake_up_request = 1;
  }
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  __HAL_FLASH_SET_LATENCY(FLASH_LATENCY_1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI48;
  RCC_OscInitStruct.HSI48State = RCC_HSI48_ON;

  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSIUSB48;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
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
