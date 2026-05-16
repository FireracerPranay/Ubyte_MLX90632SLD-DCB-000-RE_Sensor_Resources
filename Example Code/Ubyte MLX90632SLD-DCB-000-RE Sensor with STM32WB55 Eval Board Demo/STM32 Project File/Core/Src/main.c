/* USER CODE BEGIN Header */


/*This sample code is made by Rohit Maurya (Embedded Software Developer at Ubyte Consulting).
This code demonstrates all the parameters that can be read from the Melexis Technologies NV MLX90632SLD-DCB-000-RE Sensor on the Ubyte MLX90632SLD-DCB-000-RE Evaluation Board connected to Ubyte STM32WB55CGU6 Development Board

  */


/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include "mlx90632.h"
#include "mlx90632_depends.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define MLX90632_I2C_ADDR (0x3B << 1)
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
int32_t P_R, P_G, P_T, P_O, Ea, Eb, Fa, Fb, Ga;
int16_t Ha, Hb, Gb, Ka;
float Ambient_Temperature_C, Object_Temperature_C; //Ambient Temperature is the temperature measured by onboard temperature sensor on MLX90632SLD-DCB-000-RE. Object Temperature is the temperature measured using non contact IR for the object in front of the sensor
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void PeriphCommonClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_USART1_UART_Init(void);
/* USER CODE BEGIN PFP */
int32_t read_eeprom_32(int16_t reg_address, int32_t *value);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
// --- 1. UART PRINTF REDIRECTION ---
#ifdef __GNUC__
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif

PUTCHAR_PROTOTYPE {
    HAL_UART_Transmit(&huart1, (uint8_t *)&ch, 1, HAL_MAX_DELAY);
    return ch;
}

// --- 2. MELEXIS LIBRARY WRAPPERS ---
// 1. I2C Read Wrapper
int32_t mlx90632_i2c_read(int16_t register_address, uint16_t *value) {
    uint8_t buffer[2];
    // Read from sensor address 0x3B
    if (HAL_I2C_Mem_Read(&hi2c1, (0x3B << 1), (uint16_t)register_address, I2C_MEMADD_SIZE_16BIT, buffer, 2, 100) == HAL_OK) {
        // Fix: Shift MSB and combine (MSB first)
        *value = (buffer[0] << 8) | buffer[1];
        return 0; // Success
    }
    return -1; // Fail
}

// 2. I2C Write Wrapper
int32_t mlx90632_i2c_write(int16_t register_address, uint16_t value) {
    uint8_t buffer[2];
    // Fix: Extract MSB first
    buffer[0] = (value >> 8) & 0xFF;
    buffer[1] = value & 0xFF;

    if (HAL_I2C_Mem_Write(&hi2c1, (0x3B << 1), (uint16_t)register_address, I2C_MEMADD_SIZE_16BIT, buffer, 2, 100) == HAL_OK) {
        return 0; // Success
    }
    return -1; // Fail
}

// 3. Microsecond Sleep Wrapper
void usleep(int min_range, int max_range) {
    uint32_t delay_ms = (min_range / 1000) + 1;
    HAL_Delay(delay_ms);
}

// 4. Millisecond Sleep Wrapper
void msleep(int msecs) {
    HAL_Delay((uint32_t)msecs);
}

// --- 3. HELPER FOR 32-BIT EEPROM READS ---
// The Melexis sensor stores 32-bit constants across two consecutive 16-bit addresses
int32_t read_eeprom_32(int16_t reg_address, int32_t *value) {
    uint16_t lsb, msb;
    if (mlx90632_i2c_read(reg_address, &lsb) != 0) return -1;
    if (mlx90632_i2c_read(reg_address + 1, &msb) != 0) return -1;
    *value = (int32_t)((msb << 16) | lsb);
    return 0;
}
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

  /* Configure the peripherals common clocks */
  PeriphCommonClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_I2C1_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */

  // Setting the Default state of the RGB LED to off
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, 1);
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, 1);
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, 1);

  printf("MLX90632 Initialization Starting...\r\n");

  printf("Scanning I2C bus...\r\n");
    int devices_found = 0;
    for(uint8_t i = 1; i < 128; i++) {
        // Shift the 7-bit address by 1 for HAL
        HAL_StatusTypeDef res = HAL_I2C_IsDeviceReady(&hi2c1, (uint16_t)(i << 1), 3, 100);
        if(res == HAL_OK) {
            printf("-> Device found at 7-bit address: 0x%02X\r\n", i);
            devices_found++;
        }
    }
    if(devices_found == 0) {
        printf("-> No I2C devices found.\r\n");
    }
    printf("Scan complete.\r\n");
    // -------------------------

    // 1. Initialize sensor and check EEPROM version
    if (mlx90632_init() < 0) {
        printf("Error: Failed to initialize MLX90632. Check wiring.\r\n");
        while (1) { HAL_Delay(1000); }
    }

    mlx90632_set_emissivity(0.98); // Set emissivity to 0.98

    printf("Sensor initialized successfully.\r\n");

    // 2. Read 32-bit EEPROM Calibration Constants
    read_eeprom_32(MLX90632_EE_P_R, &P_R);
    read_eeprom_32(MLX90632_EE_P_G, &P_G);
    read_eeprom_32(MLX90632_EE_P_T, &P_T);
    read_eeprom_32(MLX90632_EE_P_O, &P_O);
    read_eeprom_32(MLX90632_EE_Ea, &Ea);
    read_eeprom_32(MLX90632_EE_Eb, &Eb);
    read_eeprom_32(MLX90632_EE_Fa, &Fa);
    read_eeprom_32(MLX90632_EE_Fb, &Fb);
    read_eeprom_32(MLX90632_EE_Ga, &Ga);

    // 3. Read 16-bit EEPROM Calibration Constants
    uint16_t temp;
    mlx90632_i2c_read(MLX90632_EE_Gb, &temp); Gb = (int16_t)temp;
    mlx90632_i2c_read(MLX90632_EE_Ka, &temp); Ka = (int16_t)temp;
    mlx90632_i2c_read(MLX90632_EE_Ha, &temp); Ha = (int16_t)temp;
    mlx90632_i2c_read(MLX90632_EE_Hb, &temp); Hb = (int16_t)temp;

    printf("Calibration constants loaded from EEPROM.\r\n");

    int16_t ambient_new_raw, ambient_old_raw, object_new_raw, object_old_raw;
    double pre_ambient, pre_object, ambient_temp_c, object_temp_c;
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  // 4. Trigger and read raw data from the sensor
	      if (mlx90632_read_temp_raw(&ambient_new_raw, &ambient_old_raw, &object_new_raw, &object_old_raw) == 0) {

	          // 5. Pre-process the raw values
	          pre_ambient = mlx90632_preprocess_temp_ambient(ambient_new_raw, ambient_old_raw, Gb);
	          pre_object = mlx90632_preprocess_temp_object(object_new_raw, object_old_raw, ambient_new_raw, ambient_old_raw, Ka);

	          // 6. Calculate actual temperatures in Celsius
	          ambient_temp_c = mlx90632_calc_temp_ambient(ambient_new_raw, ambient_old_raw, P_T, P_R, P_G, P_O, Gb);
	          Ambient_Temperature_C = ambient_temp_c;

	          // Note: The calc_temp_object function requires pre-processed values cast to int32_t as per the library signature
	          object_temp_c = mlx90632_calc_temp_object((int32_t)pre_object, (int32_t)pre_ambient, Ea, Eb, Ga, Fa, Fb, Ha, Hb);
	          Object_Temperature_C = object_temp_c;

	          // 7. Output via UART
	          printf("Ambient Temp: %.2f C | Object Temp: %.2f C\r\n", ambient_temp_c, object_temp_c);
	      } else {
	          printf("Failed to read sensor data.\r\n");
	      }

	      //if temperature is greater than or equal to 50°C, red LED will light up
	      if (Object_Temperature_C >= 50) {

	    	  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, 0);
	    	  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, 1);
	    	  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, 1);
	      }

	      //if temperature is greater than or equal to 20°C and less than 50°C, yellow LED (red + green) will light up
	      if (Object_Temperature_C < 50  && Object_Temperature_C >= 20) {
	    	  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, 0);
	    	  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, 0);
	    	  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, 1);
	      }

	      //if temperature is less than 20°C, blue LED (red + green) will light up
	      if (Object_Temperature_C < 20) {
	    	  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, 1);
	    	  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, 1);
	    	  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, 0);
	      }

	      HAL_Delay(200); // Wait 200ms before next measurement
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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.MSICalibrationValue = RCC_MSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure the SYSCLKSource, HCLK, PCLK1 and PCLK2 clocks dividers
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK4|RCC_CLOCKTYPE_HCLK2
                              |RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_MSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.AHBCLK2Divider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLK4Divider = RCC_SYSCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief Peripherals Common Clock Configuration
  * @retval None
  */
void PeriphCommonClock_Config(void)
{
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

  /** Initializes the peripherals clock
  */
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_SMPS;
  PeriphClkInitStruct.SmpsClockSelection = RCC_SMPSCLKSOURCE_HSI;
  PeriphClkInitStruct.SmpsDivSelection = RCC_SMPSCLKDIV_RANGE1;

  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN Smps */

  /* USER CODE END Smps */
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.Timing = 0x00000003;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

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
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart1, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart1, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart1) != HAL_OK)
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
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(RGB_Red_GPIO_Port, RGB_Red_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, RGB_Green_Pin|RGB_Blue_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : RGB_Red_Pin */
  GPIO_InitStruct.Pin = RGB_Red_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(RGB_Red_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : RGB_Green_Pin RGB_Blue_Pin */
  GPIO_InitStruct.Pin = RGB_Green_Pin|RGB_Blue_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
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
