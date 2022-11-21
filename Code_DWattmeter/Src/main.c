/* USER CODE BEGIN Header */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "fatfs.h"
#include "i2c.h"
#include "rtc.h"
#include "spi.h"
#include "tim.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
// #include <stdio.h>
#include "lcd_i2c.h"  // LCD library
#include <math.h>
#include <stdio.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
FATFS fs; // File system object
FIL fil;  // File object
// Capacity and free space
FATFS *pfs;
DWORD fre_clust;
uint32_t free_space;

// RTC
RTC_TimeTypeDef sTime;
RTC_DateTypeDef sDate;
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
uint8_t menu = 0;
uint8_t write_sd = 0;

const uint16_t adc_cross_zero_v = 2270;
const uint16_t adc_cross_zero_i = 1541;
const uint16_t max_samples = 2814;

uint8_t frecuency = 50;
uint16_t sample_count = 0;  // It must be same size as max_samples

uint16_t adc1_value = 0;
uint16_t adc2_value = 0;
uint16_t last_adc_value[] = {0, 0}; // 0: V, 1: I

float voltage_sum = 0;
float RMS_voltage = 0;
float current_sum = 0;
float RMS_current = 0;

float P = 0;
float power_sum = 0;

float S = 0;
float Q = 0;
float PF = 0;

/* USER CODE END PV */

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
  MX_I2C1_Init();
  MX_ADC1_Init();
  MX_TIM2_Init();
  MX_ADC2_Init();
  MX_TIM3_Init();
  MX_TIM1_Init();
  MX_SPI2_Init();
  MX_FATFS_Init();
  MX_RTC_Init();
  /* USER CODE BEGIN 2 */
  // Function to calibrate ADC
  HAL_ADCEx_Calibration_Start(&hadc1);
  HAL_ADCEx_Calibration_Start(&hadc2);
  HAL_Delay(100);

  // Init LCD
  Lcd_Init();
  Lcd_Clear();
  
  // Print fixed messages
  Lcd_Set_Cursor(1,1);
  Lcd_Send_String("V=");
  Lcd_Set_Cursor(2,1);
  Lcd_Send_String("I=");
  Lcd_Set_Cursor(1,14);
  Lcd_Send_String("Hz");
  Lcd_Set_Cursor(2,16);
  Lcd_Send_String("W");

  // Init SD card
  if (f_mount(&fs, "", 1) != FR_OK) {
    Lcd_Set_Cursor(1,1);
    Lcd_Send_String("SD error");
    HAL_Delay(1500);
  }
  // Check card capacity
  f_getfree("", &fre_clust, &pfs);
  // total = (uint32_t)((pfs->n_fatent - 2) * pfs->csize * 0.5);
  free_space = (uint32_t)(fre_clust * pfs->csize * 0.5);
  if (free_space < 100) {
    Lcd_Set_Cursor(1,1);
    Lcd_Send_String("SD full");
    HAL_Delay(1500);
  }

  // HAL_GPIO_WritePin(ALERT_LED_GPIO_Port, ALERT_LED_Pin, GPIO_PIN_SET);

  // Check if SD is workink ONLY DEBUG
  // if (f_open(&fil, "debug.txt", FA_OPEN_ALWAYS | FA_WRITE | FA_READ) != FR_OK) {
  //   Lcd_Set_Cursor(1,1);
  //   Lcd_Send_String("File error");
  //   HAL_Delay(1500);
  // }
  // // Go to end of file
  // f_lseek(&fil, fil.fsize);
  // // Write header
  // f_puts("Debugging SD Card\n", &fil);
  // f_close(&fil);

  // HAL_GPIO_WritePin(ALERT_LED_GPIO_Port,ALERT_LED_Pin, GPIO_PIN_RESET);

  // Start TMR2: refresh LCD
  HAL_TIM_Base_Start_IT(&htim2);
  // Start TMR3: ADC sampling
  HAL_TIM_Base_Start_IT(&htim3);
  
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

    // Start ADC1: Current
    HAL_ADC_Start(&hadc1);
    // Start ADC2: Voltage
    HAL_ADC_Start(&hadc2);

    // Read ADC1: Current
    adc1_value = HAL_ADC_GetValue(&hadc1);
    // Read ADC2: Voltage
    adc2_value = HAL_ADC_GetValue(&hadc2);

    // Calculate frecuency
    if (adc2_value == adc_cross_zero_v && last_adc_value[0] < adc_cross_zero_v) {
      // Start timer 1 if timer has been stopped
      if ( (htim1.Instance->CR1) & (TIM_CR1_CEN == 0) ) {
        HAL_TIM_Base_Start_IT(&htim1);
      }
      else {
        // Stop timer 1 if timer has been started
        HAL_TIM_Base_Stop_IT(&htim1);
        uint16_t timer1_value = __HAL_TIM_GET_COUNTER(&htim1);
        // Calculate frequency
        frecuency = (uint8_t)( (72000000 / 16) / timer1_value );
        __HAL_TIM_SET_COUNTER(&htim1, 0);
      }
    }
    if (write_sd == 1)
    {
      // Stop timer 2 and 3
      HAL_TIM_Base_Stop_IT(&htim2);
      HAL_TIM_Base_Stop_IT(&htim3);
      // Disable GPIO exti interrupt 9:5
      HAL_NVIC_DisableIRQ(EXTI9_5_IRQn);
      
      HAL_GPIO_WritePin(ALERT_LED_GPIO_Port, ALERT_LED_Pin, GPIO_PIN_SET);
      // Write SD
      if (f_open(&fil, "data.csv", FA_OPEN_ALWAYS | FA_WRITE | FA_READ) != FR_OK) {
        Lcd_Set_Cursor(1,1);
        Lcd_Send_String("File error");
        HAL_Delay(1500);
      }
      // Go to end of file
      f_lseek(&fil, fil.fsize);
      // header
      // f_puts("FECHA;Vrms;Irms;S;P;Q;FP;frec;\n", &fil);

      // Get date and time
      HAL_RTC_GetTime(&hrtc, &sTime, RTC_FORMAT_BIN);
      HAL_RTC_GetDate(&hrtc, &sDate, RTC_FORMAT_BIN);

      char buffer[100];
      sprintf(buffer, "%d/%d/%d-%d:%d;%d.%d%d;%d.%d%d;%d.%d%d;%d.%d%d;%d.%d%d;%d.%d%d;%d;\n",
        sDate.Date, sDate.Month, sDate.Year, sTime.Hours, sTime.Minutes,
        (uint8_t) RMS_voltage, (uint8_t) (RMS_voltage * 10) % 10, (uint8_t) (RMS_voltage * 100) % 10,
        (uint8_t) RMS_current, (uint8_t) (RMS_current * 10) % 10, (uint8_t) (RMS_current * 100) % 10,
        (uint8_t) S, (uint8_t) (S * 10) % 10, (uint8_t) (S * 100) % 10,
        (uint8_t) P, (uint8_t) (P * 10) % 10, (uint8_t) (P * 100) % 10,
        (uint8_t) Q, (uint8_t) (Q * 10) % 10, (uint8_t) (Q * 100) % 10,
        (uint8_t) PF, (uint8_t) (PF * 10) % 10, (uint8_t) (PF * 100) % 10,
        frecuency
        );
      f_puts(buffer, &fil);

      f_close(&fil);
      
      write_sd = 0;

      // Enable GPIO exti interrupt 9:5
      HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);
      // Continue timer 2 and 3
      HAL_TIM_Base_Start_IT(&htim2);
      HAL_TIM_Base_Start_IT(&htim3);
      HAL_GPIO_WritePin(ALERT_LED_GPIO_Port, ALERT_LED_Pin, GPIO_PIN_RESET);
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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSI|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
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
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_RTC|RCC_PERIPHCLK_ADC;
  PeriphClkInit.RTCClockSelection = RCC_RTCCLKSOURCE_LSI;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV8;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM2)
    {
    HAL_GPIO_TogglePin(BUILDIN_LED_GPIO_Port, BUILDIN_LED_Pin);

      
      if (menu == 0)
      {
        // Clear voltage LCD
        Lcd_Set_Cursor(1,3);
        Lcd_Send_String("      ");
        Lcd_Set_Cursor(2,3);
        Lcd_Send_String("      ");

        // Print values
        Lcd_Set_Cursor(1,3);
        Lcd_Send_Float(RMS_voltage,2);
        Lcd_Set_Cursor(2,3);
        Lcd_Send_Float(RMS_current,2);
        Lcd_Set_Cursor(1,10);
        Lcd_Send_Float(frecuency,0);
      }
      else
      {
        // Clear voltage LCD
        Lcd_Set_Cursor(1,3);
        Lcd_Send_String("      ");
        Lcd_Set_Cursor(2,3);
        Lcd_Send_String("      ");

        // Print values
        Lcd_Set_Cursor(1,3);
        Lcd_Send_Float(S,2);
        Lcd_Set_Cursor(2,3);
        Lcd_Send_Float(Q,2);
        Lcd_Set_Cursor(1,12);
        Lcd_Send_Float(PF,2);
      }

      Lcd_Set_Cursor(2,9);
      Lcd_Send_Float(P,2);
    }
    else if (htim->Instance == TIM3)
    {
      if (sample_count < max_samples)
      {
        sample_count++;
        voltage_sum += (adc2_value - adc_cross_zero_v);
        current_sum += (adc1_value - adc_cross_zero_i);
        power_sum += (adc2_value - adc_cross_zero_v) * (adc1_value - adc_cross_zero_i);
      }
      else
      {
        RMS_voltage = sqrt(voltage_sum * (317.77 / 1205) / max_samples);
        RMS_current = sqrt(current_sum * (0.622 / 112) / max_samples);
        P = power_sum * (317.77 / 1205) * (0.622 / 112) / max_samples;

        S = RMS_voltage * RMS_current;
        Q = sqrt( (S * S) - (P * P) );
        PF = P / S;

        // Reset all variables
        sample_count = 0;
        voltage_sum = 0;
        current_sum = 0;
        power_sum = 0;
      }
    }
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  if (GPIO_Pin == MOD_MENU_Pin)
  {
    if (menu == 1)
    {
      menu = 0;

      // Print fixed messages
      Lcd_Set_Cursor(1,1);
      Lcd_Send_String("V=");
      Lcd_Set_Cursor(2,1);
      Lcd_Send_String("I=");
      Lcd_Set_Cursor(1,9);
      Lcd_Send_String("f");
      Lcd_Set_Cursor(1,14);
      Lcd_Send_String("Hz");
      Lcd_Set_Cursor(2,16);
      Lcd_Send_String("W");
    }
    else
    {
      menu = 1;

      // Print fixed messages
      Lcd_Set_Cursor(1,1);
      Lcd_Send_String("S=");
      Lcd_Set_Cursor(2,1);
      Lcd_Send_String("Q=");
      Lcd_Set_Cursor(1,9);
      Lcd_Send_String("Fp=");
      Lcd_Set_Cursor(2,16);
      Lcd_Send_String("W");
    }
  }
  if (GPIO_Pin == WRITE_SD_Pin)
  {
    if (write_sd == 0)
      write_sd = 1;
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
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
