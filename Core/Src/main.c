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
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "i2c.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <stdbool.h>
#include "bmp2_config.h"
#include "pid_regulator.h"
#include "lcd_i2c.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
double temp = 0.0f;     // [degC]
int newTemp;	//[mdegC]
char UART_zadajnik[]="000";
float current_temp=0.0;
float RezystorTest=-1.0;
float RezystorTestDrive=-1.0;
unsigned int zadana_wart=0;
unsigned int zadane_rezystora=0;
unsigned int zadane_obiektu = 30000;
uint32_t pulse_count;
uint32_t pulse_count_prev;
int Enc = 0;
struct lcd_disp disp;
unsigned int pier_ur = 1;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
pid_t2 pid1={.param.Kp=1.5,.param.Ki=0.00002, .param.Kd=0.01,.param.dt=1.0, .previous_error=0,
		.previous_integral=0, .max_output=800.0, .min_output=-800.0};
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  if(htim == &htim2)
  {
    static unsigned int cnt = 0;		//zmienna licznika
    temp = BMP2_ReadTemperature_degC(&bmp2dev_1);	//wczytywanie wartości z czujnika temperatury
    newTemp = (int)(temp*1000);		//wartość temperatury zamieniona na int usuwając cyfry po przecinku

    cnt++;

    if(cnt == 40) 												//If aby rzadziej przesyłała się informacja do Terminala
    {
      uint8_t dane[64];									//utworzenie zmiennej buforu
      int afterKropka = newTemp%1000;
         	  int daneD=sprintf(dane," TEMP: %d.%d [degC]\r\n",newTemp/1000, afterKropka);
         	  HAL_UART_Transmit_IT(&huart3,(uint8_t*)dane , daneD);;	//przesyłanie wiadomości do terminala
      cnt = 0;													//zerowanie licznika
    }




  	  }

  //Regulator-PID
  if(htim == &htim7){

  }
  else{
    		float pwm_duty_f= (999.0*calculate_discrete_pid(&pid1,zadane_obiektu,newTemp));
    		RezystorTest = pwm_duty_f;
    		uint16_t pwm_duty=0;

    		//saturacja
    		if(pwm_duty_f<0)pwm_duty=0;else
    		if(150000>pwm_duty_f)pwm_duty=pwm_duty_f/200;else
    		if(pwm_duty_f>150000)pwm_duty=pwm_duty_f/800;else
    			pwm_duty=(uint16_t)pwm_duty_f;
    		RezystorTestDrive = pwm_duty;

    		__HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_3,pwm_duty);

  }
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	 int value = strtol((uint8_t*)&UART_zadajnik[0], NULL, 10);
	 if(value<30){
		 zadane_obiektu = value*1000;
	 }else{
		 zadane_obiektu = 30000;
	 }
	 if(value>19){
		 zadane_obiektu = value*1000;
	 }else{
		 zadane_obiektu = 20000;
	 }
	 HAL_UART_Receive_IT(&huart3,(uint8_t*)UART_zadajnik , 3);


//	if (UART_zadajnik[0] == 't')
//	{
//		int value = (UART_zadajnik[1]*10);
//		zadane_obiektu = value;
//	}
//
//	HAL_UART_Receive_IT(&huart3, UART_zadajnik , 2);


}

//void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
//{
// int value = strtol((uint8_t*)&PWM[3], NULL, 10);
// __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, value*10);
// HAL_UART_Receive_IT(&huart3,(uint8_t*)PWM , 6);
//}


void display_function()
{
	if(pier_ur){
		lcd_clear(&disp);
		sprintf((char*)disp.f_line, "Hubert");
		sprintf((char*)disp.s_line, "Grzegorz");
		lcd_display(&disp);
		HAL_Delay(1000);
		pier_ur = 0;
	}
	else
	{
		sprintf((char*)disp.f_line, "Temp. akt.:%d.%02d", newTemp / 1000, newTemp % 1000);
		sprintf((char*)disp.s_line, "Temp. zad.:%d.%02d", zadane_obiektu / 1000, zadane_obiektu % 1000);
		lcd_display(&disp);
	}

	HAL_Delay(2000);

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

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART3_UART_Init();
  MX_SPI4_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_TIM7_Init();
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */
  BMP2_Init(&bmp2dev_1);// inicjalizacja czujnika
  HAL_TIM_Base_Start_IT(&htim2);// uruchomienie timerow
  HAL_TIM_Base_Start_IT(&htim7);// uruchomienie timerow
  HAL_TIM_Base_Start_IT(&htim4);// uruchomienie timerow
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);
  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, 0);
  HAL_UART_Receive_IT(&huart3, UART_zadajnik, 3);
  //wyswietlacz
    disp.addr = (0x27 << 1);
    disp.bl = true;
    lcd_init(&disp);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

	  	  	pulse_count = __HAL_TIM_GET_COUNTER (& htim4 );

	     	if(pulse_count > pulse_count_prev){
	     	  		  Enc++;
	     	  		  zadane_obiektu = zadane_obiektu + Enc*1;
	     	  		  if (zadane_obiektu>30000)zadane_obiektu = 30000;
	     	  		  Enc = 0;
	     	}
	     	if(pulse_count < pulse_count_prev){
	     	  		  Enc++;
	     	  		  zadane_obiektu = zadane_obiektu - Enc*1;
	     	  		  if (zadane_obiektu<20000)zadane_obiektu = 35000;
	     	  		  Enc = 0;
	     	}


	     	pulse_count_prev = pulse_count;
//
//	  HAL_Delay (200) ;


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

  /** Configure LSE Drive Capability
  */
  HAL_PWR_EnableBkUpAccess();

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 216;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 3;
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
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV4;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_7) != HAL_OK)
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
