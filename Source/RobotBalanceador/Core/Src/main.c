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
 * Digital PID Controllers - Dr. Varodom Toochinda
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

#include "string.h"
#include "usbd_cdc_if.h"
#include "mpu6050.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

typedef struct {
 float k1, k2, k3;
 float e0, e1, e2;
 float u_min, u_max, u;
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
PID_t pid;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

void PWM_Aplicar(float valor);
void PWM_Aplicar2(float valor);
void PID_Init(PID_t *pid, float kp, float ki, float kd, float u_min, float u_max);
float PID(PID_t *pid, float setpoint, float nueva_lectura);

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
  MX_TIM4_Init();
  MX_TIM3_Init();
  MX_I2C2_Init();
  /* USER CODE BEGIN 2 */

  // Inicializar variables
  char msg[50];
  last_mpu_time = HAL_GetTick();


  // Inicialización del PWM
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
  //HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);  El DRV8870 utiliza 4 pines de PWM
  //HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4);
  __HAL_TIM_SetCompare(&htim3, TIM_CHANNEL_1, 0);
  __HAL_TIM_SetCompare(&htim3, TIM_CHANNEL_2, 0);
  // __HAL_TIM_SetCompare(&htim3, TIM_CHANNEL_3, 0);
  // __HAL_TIM_SetCompare(&htim3, TIM_CHANNEL_4, 0);


  // Inicialización de timer para Encoder
  HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_ALL);


  // Inicializar el PID
  PID_Init(&pid, 40, 0, 0, -1000, 1000);

  // Esperar para la inicialización del MPU6050
  while (MPU6050_Init(&hi2c2) == 1);



  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1) {

    // Lectura del MPU6050 cada 100ms
    if (HAL_GetTick() - last_mpu_time > 100) {
      MPU6050_Read_All(&hi2c2, &MPU6050);

      float angulo = MPU6050.KalmanAngleY;
      float accion = PID(&pid, 0, angulo);
      PWM_Aplicar(accion);

      // Enviar a la PC
      snprintf(msg, 50, "accion=%f,angulo=%f\r\n", accion, angulo);
      CDC_Transmit_FS((uint8_t *)msg, strlen(msg));
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

// Aplicar PWM a los motores, si el valor es negativo va en el otro sentido
void PWM_Aplicar(float valor) {
  // Duty va de 0 a 1000

  // Convertir valor a entero
  uint16_t entero = (uint16_t)ABS(valor);

  // Limitar de 0 a 1000
  entero = (entero > 1000)? 1000 : entero;
  __HAL_TIM_SetCompare(&htim3, TIM_CHANNEL_1, entero);
  __HAL_TIM_SetCompare(&htim3, TIM_CHANNEL_2, entero);

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
void PWM_Aplicar2(float valor) {
  // Duty va de 0 a 1000

  // Convertir valor a entero
  uint16_t entero = (uint16_t)ABS(valor);

  // Limitar de 0 a 1000
  entero = (entero > 1000)? 1000 : entero;


  if (valor >= 0.0) {
    // Motor Derecho
    __HAL_TIM_SetCompare(&htim3, TIM_CHANNEL_1, entero);
    __HAL_TIM_SetCompare(&htim3, TIM_CHANNEL_2, 0);

    // Motor Izquierdo
    __HAL_TIM_SetCompare(&htim3, TIM_CHANNEL_3, entero);
    __HAL_TIM_SetCompare(&htim3, TIM_CHANNEL_4, 0);

  } else {
    // Motor Derecho
    __HAL_TIM_SetCompare(&htim3, TIM_CHANNEL_1, 0);
    __HAL_TIM_SetCompare(&htim3, TIM_CHANNEL_2, entero);

    // Motor Izquierdo
    __HAL_TIM_SetCompare(&htim3, TIM_CHANNEL_3, 0);
    __HAL_TIM_SetCompare(&htim3, TIM_CHANNEL_4, entero);
  }
}

void PID_Init(PID_t *pid, float kp, float ki, float kd, float u_min, float u_max) {
  pid->k1 = kp + ki + kd;
  pid->k2 = -kp - 2*kd;
  pid->k3 = kd;
  pid->e1 = 0.0;
  pid->e2 = 0.0;
  pid->u_min = u_min;
  pid->u_max = u_max;
  pid->u = 0.0;
}


// Cálculo de la acción del PID
float PID(PID_t *pid, float setpoint, float nueva_lectura) {
  pid->e2 = pid->e1;
  pid->e1 = pid->e0;

  pid->e0 = setpoint - nueva_lectura;
  float delta_u = pid->k1*pid->e0 + pid->k2*pid->e1 + pid->k3*pid->e2;
  pid->u = pid->u + delta_u;

  if (pid->u > pid->u_max) pid->u = pid->u_max;
  if (pid->u < pid->u_min) pid->u = pid->u_min;

  return pid->u;
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
