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
#include "adc.h"
#include "dma.h"
#include "fatfs.h"
#include "i2c.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "bmi088.h"
#include "motor_servo.h"
#include "data_logger.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define ACC_IRQ
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
bmi088_struct_t bmi_imu_s;

uint8_t str[200];
uint8_t is_500ms = 0;

extern float error;
extern uint32_t pulse;

uint32_t starting_time = 0;
uint8_t is_started = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
static void reg_3v3_on(void);
static void reg_3v3_off(void);
void bmi_callback(bmi088_struct_t *BMI);
void serial_println(char* str, UART_HandleTypeDef *huart_disp);
static uint8_t bmi_begin(void);
void reg_3v3_on();
void reg_3v3_off();
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
  MX_DMA_Init();
  MX_I2C1_Init();
  MX_I2C3_Init();
  MX_SPI3_Init();
  MX_USART3_UART_Init();
  MX_TIM1_Init();
  MX_ADC1_Init();
  MX_ADC2_Init();
  MX_TIM2_Init();
  MX_TIM4_Init();
  MX_TIM3_Init();
  MX_FATFS_Init();
  /* USER CODE BEGIN 2 */
  reg_3v3_off();
  reg_3v3_on();
  	uint8_t bmi_ret = bmi_begin();
    if(bmi_ret)
    {
  	  serial_println("bmi fail", &TTL_HNDLR);
        if((bmi_ret & 0x01) == 0x01)
        {
          sprintf((char*)str, "error in accel");
          serial_println((char*) str, &TTL_HNDLR);
        }
        if((bmi_ret & 0x02) == 0x02)
        {
          sprintf((char*)str, "error in gyro");
          serial_println((char*) str, &TTL_HNDLR);
        }
    }

    Motor_Servo_Init(&htim3);
	bmi088_config(&bmi_imu_s);
	get_offset(&bmi_imu_s);

	HAL_TIM_Base_Start_IT(&htim4);

	data_logger_init();
	HAL_GPIO_WritePin(BUZZER_GPIO_Port, BUZZER_Pin, GPIO_PIN_SET);
	HAL_Delay(500);
	HAL_GPIO_WritePin(BUZZER_GPIO_Port, BUZZER_Pin, GPIO_PIN_RESET);


	 // Hedef açı
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
/*
	  HAL_ADC_Start(&VLT_ADC_HNDLR);
	  HAL_ADC_Start(&AMP_ADC_HNDLR);

	  HAL_ADC_PollForConversion(&VLT_ADC_HNDLR, 100);
	  adc1 = HAL_ADC_GetValue(&VLT_ADC_HNDLR);
	  float adc1_f = (float)adc1 - 25.0;
	  adc1_f = (adc1_f > 0) * adc1_f;
	  adc1_f	= adc1_f * 5.31;

	  HAL_ADC_PollForConversion(&AMP_ADC_HNDLR, 100);
	  adc2 = HAL_ADC_GetValue(&AMP_ADC_HNDLR);
	  float adc2_f = (float)adc2 * 0.015;

	  sprintf((char*)str, "adc1= %lu\n\r", adc1);
	  HAL_UART_Transmit(&huart3, str, strlen((char*)str), 50);

	  sprintf((char*)str, "adc2= %.2f\n\r", adc2_f);
	  HAL_UART_Transmit(&huart3, str, strlen((char*)str), 50);

	  uint8_t chip_id = bmi088_getGyroChipId();
	  sprintf((char*)str, "chip id =  %x\n\r", chip_id);
	  HAL_UART_Transmit(&huart3, str, strlen((char*)str), 50);
*/

//	  sprintf((char*)str, "encoder =  %lu\n\r", (TIM2->CNT)>>2);
//	  HAL_UART_Transmit(&TTL_HNDLR, str, strlen((char*)str), 50);
	  if(is_started == 0 && (bmi_imu_s.datas.acc_x > 3000))
	  {
		  is_started = 1;
		  bmi_imu_s.datas.gyro_x = 0;
	  }
	  bmi088_update(&bmi_imu_s);

	  if(is_500ms)
	  {
		  log_datas(HAL_GetTick(), bmi_imu_s.datas.gyro_x_angle, pulse, error);
		  sprintf((char*)str, "angle_ x:%f pwm:%lu error:%f\n\r", bmi_imu_s.datas.gyro_x_angle, pulse, error);
		  serial_println((char*)str, &TTL_HNDLR);
		  is_500ms = 0;
	  }
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

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 180;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Activate the Over-Drive mode
  */
  if (HAL_PWREx_EnableOverDrive() != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
uint8_t bmi_begin(void)
{
	//Acc config
	bmi_imu_s.device_config.acc_bandwith = ACC_BWP_OSR4;
	bmi_imu_s.device_config.acc_outputDateRate = ACC_ODR_200;
	bmi_imu_s.device_config.acc_powerMode = ACC_PWR_SAVE_ACTIVE;
	bmi_imu_s.device_config.acc_range = ACC_RANGE_24G;

	// Gyro config
	bmi_imu_s.device_config.gyro_bandWidth = GYRO_BW_116;
	bmi_imu_s.device_config.gyro_range = GYRO_RANGE_2000;
	bmi_imu_s.device_config.gyro_powerMode = GYRO_LPM_NORMAL;

	//Device config
	bmi_imu_s.device_config.acc_IRQ = EXTI9_5_IRQn;
	bmi_imu_s.device_config.gyro_IRQ = EXTI9_5_IRQn;
	bmi_imu_s.device_config.BMI_I2c = &IMU_I2C_HNDLR;
	bmi_imu_s.device_config.offsets = NULL;	//Offset datas stored in backup sram for saving them unwanted reset.
	bmi_imu_s.IMU_callback = bmi_callback;
	return	bmi088_init(&bmi_imu_s);
}
void bmi_callback(bmi088_struct_t *BMI)
{
    if(is_started == 1)
    {
    	float current_angle = bmi_imu_s.datas.gyro_x_angle;
    	Motor_Servo_Control(current_angle, TARGET_ANGLE);
    }

}

//Pcb 3.3 v regulator on function.
void reg_3v3_on()
{
	HAL_GPIO_WritePin(SENSOR_EN_GPIO_Port, SENSOR_EN_Pin, GPIO_PIN_SET);
	HAL_Delay(50);
}

//Pcb 3.3 v regulator off function.
void reg_3v3_off()
{
	HAL_GPIO_WritePin(SENSOR_EN_GPIO_Port, SENSOR_EN_Pin, GPIO_PIN_RESET);
	HAL_Delay(50);
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	if(GPIO_Pin == INT_ACC_Pin)
	{
		bmi088_set_accel_INT(&bmi_imu_s);
	}
	if(GPIO_Pin == INT_GYRO_Pin)
	{
		bmi088_set_gyro_INT(&bmi_imu_s);
	}
}

void serial_println(char* str, UART_HandleTypeDef *huart_disp)
{
	HAL_UART_Transmit_DMA(huart_disp, (uint8_t*)str, strlen(str));
	HAL_UART_Transmit_DMA(huart_disp, (uint8_t*)"\r\n", 2);
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if(htim->Instance == TIM4)
	{
		is_500ms = 1;
	}
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
