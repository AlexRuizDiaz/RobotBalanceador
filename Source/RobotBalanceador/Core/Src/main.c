/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2024 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */

/* REFERENCIAS
 * HAL library for GY-521 (MPU6050) with Kalman filter: https://github.com/leech001/MPU6050
 *
 *
 *
 */

/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "i2c.h"
#include "tim.h"
#include "usb_device.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include "usbd_cdc.h"
#include "mpu6050.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

typedef struct {
 float kp;
 float ki;
 float kd;
 // Lo que sea necesario
} PID_t;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

// Macro de valor absoluto que no castea float a int
#define ABS(x) ((x >= 0)? x : -x)

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

uint32_t last_mpu_time = 0;
MPU6050_t MPU6050;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

void AplicarPWM(float valor);
void AplicarPWM2(float valor);
float AccionPID(PID_t *pid, float setpoint, float nueva_lectura);
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
  MX_USB_DEVICE_Init();
  MX_I2C1_Init();
  MX_TIM4_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */

  // Inicializar variables
  last_mpu_time = HAL_GetTick();

  // Inicialización del PWM
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);
  __HAL_TIM_SetCompare(&htim2, TIM_CHANNEL_1, 0);
  __HAL_TIM_SetCompare(&htim2, TIM_CHANNEL_2, 0);
  // __HAL_TIM_SetCompare(&htim2, TIM_CHANNEL_3, 0); El DRV8870 utiliza 4 pines de PWM
  // __HAL_TIM_SetCompare(&htim2, TIM_CHANNEL_4, 0);
  // Esperar para la inicialización del MPU6050
  while (MPU6050_Init(&hi2c1) == 1);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1) {

    // Lectura del MPU6050 cada 100ms
    if (HAL_GetTick() - last_mpu_time > 100) {
      MPU6050_Read_All(&hi2c1, &MPU6050);
    }

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USB;
  PeriphClkInit.UsbClockSelection = RCC_USBCLKSOURCE_PLL_DIV1_5;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

// Aplicar PWM a los motores
void AplicarPWM(float valor) {
  // Duty va de 0 a 1000

  // Convertir valor a entero
  uint16_t entero = (uint16_t)ABS(valor);

  // Limitar de 0 a 1000
  entero = (entero > 1000)? 1000 : entero;
  __HAL_TIM_SetCompare(&htim2, TIM_CHANNEL_1, entero);
  __HAL_TIM_SetCompare(&htim2, TIM_CHANNEL_2, entero);

  if (valor >= 0.0) {
    //Motor Derecho
    HAL_GPIO_WritePin(MOT1_IN1_GPIO_Port, MOT1_IN1_Pin, 1); //Giro Horario
    HAL_GPIO_WritePin(MOT1_IN2_GPIO_Port, MOT1_IN2_Pin, 0); //Grio Antihorario
    //Motor Izquierdo
    HAL_GPIO_WritePin(MOT2_IN1_GPIO_Port, MOT2_IN1_Pin, 1); //Giro Horario
    HAL_GPIO_WritePin(MOT2_IN2_GPIO_Port, MOT2_IN2_Pin, 0); //Grio Antihorario
  } else {
    //Motor Derecho
    HAL_GPIO_WritePin(MOT1_IN1_GPIO_Port, MOT1_IN1_Pin, 0); //Giro Horario
    HAL_GPIO_WritePin(MOT1_IN2_GPIO_Port, MOT1_IN2_Pin, 1); //Grio Antihorario
    //Motor Izquierdo
    HAL_GPIO_WritePin(MOT2_IN1_GPIO_Port, MOT2_IN1_Pin, 0); //Giro Horario
    HAL_GPIO_WritePin(MOT2_IN2_GPIO_Port, MOT2_IN2_Pin, 1); //Grio Antihorario
  }
}

// Aplicar PWM a los motores utilizando el puente H DRV8870
void AplicarPWM2(float valor) {
  // Duty va de 0 a 1000

  // Convertir valor a entero
  uint16_t entero = (uint16_t)ABS(valor);

  // Limitar de 0 a 1000
  entero = (entero > 1000)? 1000 : entero;


  if (valor >= 0.0) {
    // Motor Derecho
    __HAL_TIM_SetCompare(&htim2, TIM_CHANNEL_1, entero);
    __HAL_TIM_SetCompare(&htim2, TIM_CHANNEL_2, 0);

    // Motor Izquierdo
    __HAL_TIM_SetCompare(&htim2, TIM_CHANNEL_3, entero);
    __HAL_TIM_SetCompare(&htim2, TIM_CHANNEL_4, 0);

  } else {
    // Motor Derecho
    __HAL_TIM_SetCompare(&htim2, TIM_CHANNEL_1, 0);
    __HAL_TIM_SetCompare(&htim2, TIM_CHANNEL_2, entero);

    // Motor Izquierdo
    __HAL_TIM_SetCompare(&htim2, TIM_CHANNEL_3, 0);
    __HAL_TIM_SetCompare(&htim2, TIM_CHANNEL_4, entero);
  }
}

float AccionPID(PID_t *pid, float setpoint, float nueva_lectura) {
  // Código ...





  //return accion;
  return 0.0;
}


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
  while (1) {
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
