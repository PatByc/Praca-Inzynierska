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
#include "string.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
	#include <stdio.h>
	#include <stdlib.h>
	#include <math.h> // Do fabs()
	#include <stdbool.h>


/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

	#ifndef HSEM_ID_0
	#define HSEM_ID_0 (0U) /* HW semaphore 0*/
	#endif

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
#if defined ( __ICCARM__ ) /*!< IAR Compiler */
#pragma location=0x30000000
ETH_DMADescTypeDef  DMARxDscrTab[ETH_RX_DESC_CNT]; /* Ethernet Rx DMA Descriptors */
#pragma location=0x30000200
ETH_DMADescTypeDef  DMATxDscrTab[ETH_TX_DESC_CNT]; /* Ethernet Tx DMA Descriptors */

#elif defined ( __CC_ARM )  /* MDK ARM Compiler */

__attribute__((at(0x30000000))) ETH_DMADescTypeDef  DMARxDscrTab[ETH_RX_DESC_CNT]; /* Ethernet Rx DMA Descriptors */
__attribute__((at(0x30000200))) ETH_DMADescTypeDef  DMATxDscrTab[ETH_TX_DESC_CNT]; /* Ethernet Tx DMA Descriptors */

#elif defined ( __GNUC__ ) /* GNU Compiler */
ETH_DMADescTypeDef DMARxDscrTab[ETH_RX_DESC_CNT] __attribute__((section(".RxDecripSection"))); /* Ethernet Rx DMA Descriptors */
ETH_DMADescTypeDef DMATxDscrTab[ETH_TX_DESC_CNT] __attribute__((section(".TxDecripSection")));   /* Ethernet Tx DMA Descriptors */

#endif

ETH_TxPacketConfig TxConfig;

ADC_HandleTypeDef hadc3;
DMA_HandleTypeDef hdma_adc3;

ETH_HandleTypeDef heth;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim5;
TIM_HandleTypeDef htim6;
TIM_HandleTypeDef htim7;
TIM_HandleTypeDef htim8;

UART_HandleTypeDef huart3;

PCD_HandleTypeDef hpcd_USB_OTG_FS;

/* USER CODE BEGIN PV */


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void PeriphCommonClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC3_Init(void);
static void MX_USB_OTG_FS_PCD_Init(void);
static void MX_ETH_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_TIM8_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM7_Init(void);
static void MX_TIM4_Init(void);
static void MX_TIM6_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM5_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

int PLC_IN1=0;
int PLC_IN2=0;
int PLC_IN_SUM=0;

int PLC_OUT=0;

	// Parametry
	#define NUM_SAMPLES 50 // Liczba próbek do uśredniania

	// Tablica do przechowywania próbek dla każdego kanału
	float sampleBuffer[7][NUM_SAMPLES] = {0};

	// Indeksy do zarządzania buforami próbek
	int sampleCounts[7] = {0};
	// Tablica na średnie wartości kątów
	float averagedAngles[7] = {0};



	float angles[7];
	float adc_angle[7];
	volatile uint16_t rawValues[7];
	volatile int adcConversionComplete = 0;

	void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc)
	{
		if (hadc->Instance == ADC3)
		{
			// Maksymalna wartość dla 16-bitowego ADC
			const uint32_t maxAdcValue = 65536;

			for (int i = 0; i < 7; i++) // Iteracja po kanałach
			{
				float current_angle = (rawValues[i] * 180.0f) / maxAdcValue;
				angles[i]=current_angle;
				// Zaokrąglanie do jednego miejsca po przecinku
				angles[i] = round(angles[i]);
			}

			// Flaga zakończenia konwersji
			adcConversionComplete = 1;
		}
	}






	/* Tablica Servo z dodatkowymi parametrami min/max pulsu */
	typedef struct {
		TIM_HandleTypeDef *htim;     // Wskaźnik na timer
		uint8_t pwm_channel_in1;     // Kanał PWM dla IN1
		float target_angle;          // Docelowy kąt
		float min_angle;             // Minimalny kąt pracy serwa
		float max_angle;             // Maksymalny kąt pracy serwa
		uint32_t min_pulse;          // Minimalny puls dla kąta 0 stopni
		uint32_t max_pulse;          // Maksymalny puls dla kąta 180 stopni
		uint32_t ADC_MIN;          // Minimalny puls dla kąta 0 stopni
		uint32_t ADC_MAX;          // Maksymalny puls dla kąta 180 stopni
	} Servo2;


	Servo2 servos2[7] = {


		{&htim2, TIM_CHANNEL_1, 60.0f, 0.0f, 180.0f, 250, 1250, 18, 172},
		{&htim2, TIM_CHANNEL_2, 160.0f, 0.0f, 180.0f, 250, 1250, 18, 172},
		{&htim2, TIM_CHANNEL_3, 180.0f, 0.0f, 180.0f, 250, 1250, 18, 172},
		{&htim2, TIM_CHANNEL_4, 102.0f, 0.0f, 180.0f, 250, 1250, 18, 172},
		{&htim3, TIM_CHANNEL_1, 90.0f, 0.0f, 180.0f, 250, 1250, 43, 150},
		{&htim3, TIM_CHANNEL_2, 170.0f, 0.0f, 180.0f, 250, 1250, 43, 150},
		{&htim3, TIM_CHANNEL_4, 26.0f, 10.0f, 80.0f, 250, 1250, 43, 150}
	};


	void ADC_To_Angle() {
		for (int i = 0; i < 7; i++) // Iteracja po kanałach
		{
			adc_angle[i] = (angles[i] - servos2[i].ADC_MIN) * 180.0f / (servos2[i].ADC_MAX - servos2[i].ADC_MIN);
		}

	}




	void Servo_SetAngle(Servo2 *servo) {

		// Sprawdzenie, czy kąt mieści się w zakresie
		if (servo->target_angle < servo->min_angle) {
			servo->target_angle = servo->min_angle;
		} else if (servo->target_angle > servo->max_angle) {
			servo->target_angle = servo->max_angle;
		}

		// Przeliczenie kąta na wartość impulsu PWM
		// Minimalny puls odpowiada kątowi 0 stopni, maksymalny puls odpowiada kątowi 180 stopni
		uint32_t pulse_range = servo->max_pulse - servo->min_pulse; // Zakres pulsu
		uint32_t pulse = servo->min_pulse + (uint32_t)((servo->target_angle / 180.0f) * pulse_range);

		// Ustawienie wartości PWM dla wybranego kanału
		__HAL_TIM_SET_COMPARE(servo->htim, servo->pwm_channel_in1, pulse);
	}




	void AverageSamplesByTimer()
	{

		for (int i = 0; i < 7; i++) // Iteracja po kanałach
		{
			// Dodanie próbki do bufora
			sampleBuffer[i][sampleCounts[i]] = angles[i];
			sampleCounts[i]++;

			// Jeśli zgromadzono wystarczającą liczbę próbek
			if (sampleCounts[i] >= NUM_SAMPLES)
			{
				// Obliczanie średniej
				float sum = 0.0f;
				for (int j = 0; j < NUM_SAMPLES; j++)
				{
					sum += sampleBuffer[i][j];
				}
				averagedAngles[i] = round(sum / NUM_SAMPLES);

				// Resetowanie licznika próbek
				sampleCounts[i] = 0;
			}
		}
	}

void PLC_check()
{
	  if ((HAL_GPIO_ReadPin(PD1_IN1_GPIO_Port,PD1_IN1_Pin)) == GPIO_PIN_RESET )
	  {
		  PLC_IN1=1;
	  }
	  else
	  {
		  PLC_IN1=0;
	  }

	  if ((HAL_GPIO_ReadPin(PD2_IN2_GPIO_Port,PD2_IN2_Pin)) == GPIO_PIN_RESET )
	  {
		  PLC_IN2=1;
	  }
	  else
	  {
		  PLC_IN2=0;
	  }

	  if (PLC_IN1==0 && PLC_IN2==0)
	  {
		  PLC_IN_SUM=0;
	  }
	  else if (PLC_IN1==1 && PLC_IN2==0)
	  {
		  PLC_IN_SUM=1;
	  }
	  else if (PLC_IN1==0 && PLC_IN2==1)
	  {
		  PLC_IN_SUM=2;
	  }
	  else if (PLC_IN1==1 && PLC_IN2==1)
	  {
		  PLC_IN_SUM=3;
	  }
	  else
	  {
		  PLC_IN_SUM=9;
	  }

	  if(PLC_OUT == 1)
	  {
		  HAL_GPIO_WritePin(PD3_OUT_GPIO_Port,PD3_OUT_Pin, GPIO_PIN_SET);
	  }
	  else
	  {
		  HAL_GPIO_WritePin(PD3_OUT_GPIO_Port,PD3_OUT_Pin, GPIO_PIN_RESET);
	  }
}

int alarm=0;
int alarm_set=0;
int alarm_led_on=0;
int alarm_led_off=0;
int PLC_polka=0;

void ROBOT_ALARM()
{
	if (alarm == 1)
	{
		if (alarm_set == 0)
		{
			HAL_GPIO_WritePin(LD1_GPIO_Port, LD1_Pin , GPIO_PIN_SET);
			HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin , GPIO_PIN_SET);
			HAL_GPIO_WritePin(LD3_GPIO_Port, LD3_Pin , GPIO_PIN_SET);
			if (alarm_led_on>50)
			{
				alarm_set=1;
				alarm_led_on=0;
			}
			alarm_led_on++;
		}
		else
		{
			HAL_GPIO_WritePin(LD1_GPIO_Port, LD1_Pin , GPIO_PIN_RESET);
			HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin , GPIO_PIN_RESET);
			HAL_GPIO_WritePin(LD3_GPIO_Port, LD3_Pin , GPIO_PIN_RESET);

			if (alarm_led_off>50)
			{
				alarm_set=0;
				alarm_led_off=0;
			}
			alarm_led_off++;
		}




	}
}



	void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
		if (htim->Instance == TIM7) {

				for (int i = 0; i < 7; i++)
				{
					Servo_SetAngle(&servos2[i]);
				}
		}
		if (htim->Instance == TIM6)
		{
			AverageSamplesByTimer();
			ADC_To_Angle();
			PLC_check();
			ROBOT_ALARM();


		}
	}



/* Struktura pozycji */
typedef struct {
    float angles[7]; // Kąty dla 7 serwomechanizmów
} RobotPosition;

/* Funkcja do płynnego przechodzenia między kątami */
void Robot_SetPosition(Servo2 servos[], RobotPosition *current_position, RobotPosition target_position, float step) {
    for (int i = 0; i < 7; i++) {
        if (fabsf(current_position->angles[i] - target_position.angles[i]) > step) {
            // Zwiększanie lub zmniejszanie kąta w stronę docelowego
            if (current_position->angles[i] < target_position.angles[i]) {
                current_position->angles[i] += step;
            } else {
                current_position->angles[i] -= step;
            }
        } else {
            // Ustawianie dokładnego kąta, jeśli różnica jest mniejsza od kroku
            current_position->angles[i] = target_position.angles[i];
        }
        // Aktualizacja serwomechanizmu
        servos[i].target_angle = current_position->angles[i];
        Servo_SetAngle(&servos[i]);
    }
}

/* Funkcja obsługująca logikę przejścia między pozycjami */
void Robot_MoveToPosition(Servo2 servos[], RobotPosition *current_position, RobotPosition target_position, float step, uint16_t delay_ms) {
    while (1) {
        bool all_positions_reached = true;

        // Aktualizuj pozycję serwomechanizmów
        Robot_SetPosition(servos, current_position, target_position, step);

        // Sprawdzanie, czy wszystkie kąty osiągnęły docelowe wartości
        for (int i = 0; i < 7; i++) {
            if (fabsf(current_position->angles[i] - target_position.angles[i]) > step) {
                all_positions_reached = false;
                break;
            }
        }

        if (all_positions_reached) {
            break; // Wyjdź z pętli, jeśli wszystkie kąty są w docelowych pozycjach
        }

        HAL_Delay(delay_ms); // Opóźnienie dla płynności ruchu
    }
}

RobotPosition home_position = {{60.0f, 160.0f, 180.0f, 102.0f, 10.0f, 180.0f, 26.0f}};
RobotPosition pick_position_1 = {{60.0f, 160.0f, 180.0f, 102.0f, 90.0f, 170.0f, 26.0f}};
RobotPosition pick_position_2 = {{15.0f, 160.0f, 180.0f, 102.0f, 90.0f, 170.0f, 26.0f}};
RobotPosition pick_position_3 = {{15.0f, 160.0f, 180.0f, 102.0f, 150.0f, 170.0f, 26.0f}};
RobotPosition pick_position_4 = {{15.0f, 120.0f, 180.0f, 102.0f, 150.0f, 175.0f, 26.0f}};
RobotPosition pick_position_5 = {{15.0f, 110.0f, 180.0f, 102.0f, 150.0f, 175.0f, 26.0f}};
RobotPosition grip_position_1 = {{15.0f, 110.0f, 180.0f, 102.0f, 150.0f, 175.0f, 60.0f}};
RobotPosition grip_position_2 = {{15.0f, 130.0f, 180.0f, 102.0f, 150.0f, 175.0f, 60.0f}};
RobotPosition home_grip = {{60.0f, 160.0f, 180.0f, 90.0f, 120.0f, 175.0f, 60.0f}};

RobotPosition shelf1_position_1 = {{50.0f, 160.0f, 180.0f, 90.0f, 120.0f, 180.0f, 60.0f}};
RobotPosition shelf1_position_2 = {{50.0f, 138.0f, 150.0f, 90.0f, 120.0f, 180.0f, 60.0f}};
RobotPosition shelf1_position_3 = {{50.0f, 132.0f, 160.0f, 90.0f, 115.0f, 180.0f, 60.0f}};
RobotPosition shelf1_position_4 = {{50.0f, 132.0f, 160.0f, 90.0f, 115.0f, 180.0f, 26.0f}};
RobotPosition shelf1_position_5 = {{50.0f, 138.0f, 150.0f, 90.0f, 120.0f, 180.0f, 26.0f}};
RobotPosition shelf1_position_6 = {{50.0f, 160.0f, 180.0f, 90.0f, 120.0f, 180.0f, 26.0f}};
RobotPosition shelf1_position_7 = {{60.0f, 160.0f, 180.0f, 102.0f, 10.0f, 180.0f, 26.0f}};


RobotPosition shelf2_position_2 = {{60.0f, 160.0f, 170.0f, 90.0f, 90.0f, 180.0f, 60.0f}};
RobotPosition shelf2_position_3 = {{60.0f, 160.0f, 170.0f, 90.0f, 90.0f, 180.0f, 60.0f}};
RobotPosition shelf2_position = {{85.0f, 115.0f, 90.0f, 125.0f, 60.0f, 155.0f, 26.0f}};
RobotPosition shelf3_position = {{90.0f, 110.0f, 80.0f, 130.0f, 70.0f, 150.0f, 26.0f}};





/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */
/* USER CODE BEGIN Boot_Mode_Sequence_0 */
	  int32_t timeout;
/* USER CODE END Boot_Mode_Sequence_0 */

/* USER CODE BEGIN Boot_Mode_Sequence_1 */
	  /* Wait until CPU2 boots and enters in stop mode or timeout*/


	  timeout = 0xFFFF;
	  while((__HAL_RCC_GET_FLAG(RCC_FLAG_D2CKRDY) != RESET) && (timeout-- > 0));
	  if ( timeout < 0 )
	  {
	  Error_Handler();
	  }
/* USER CODE END Boot_Mode_Sequence_1 */
  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

/* Configure the peripherals common clocks */
  PeriphCommonClock_Config();
/* USER CODE BEGIN Boot_Mode_Sequence_2 */
	/* When system initialization is finished, Cortex-M7 will release Cortex-M4 by means of
	HSEM notification */
	/*HW semaphore Clock enable*/
	__HAL_RCC_HSEM_CLK_ENABLE();
	/*Take HSEM */
	HAL_HSEM_FastTake(HSEM_ID_0);
	/*Release HSEM in order to notify the CPU2(CM4)*/
	HAL_HSEM_Release(HSEM_ID_0,0);
	/* wait until CPU2 wakes up from stop mode */
	timeout = 0xFFFF;
	while((__HAL_RCC_GET_FLAG(RCC_FLAG_D2CKRDY) == RESET) && (timeout-- > 0));
	if ( timeout < 0 )
	{
	Error_Handler();
	}
/* USER CODE END Boot_Mode_Sequence_2 */

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_ADC3_Init();
  MX_USB_OTG_FS_PCD_Init();
  MX_ETH_Init();
  MX_USART3_UART_Init();
  MX_TIM8_Init();
  MX_TIM3_Init();
  MX_TIM7_Init();
  MX_TIM4_Init();
  MX_TIM6_Init();
  MX_TIM2_Init();
  MX_TIM5_Init();
  /* USER CODE BEGIN 2 */

	 // HAL_ADCEx_Calibration_Start(&hadc3, ADC_CALIB_OFFSET, ADC_SINGLE_ENDED);
	  HAL_ADC_Start_DMA(&hadc3, (uint32_t*)rawValues, 7);
	  HAL_TIM_Base_Start(&htim8);
	  HAL_TIM_Base_Start_IT(&htim7);
	  HAL_TIM_Base_Start_IT(&htim6);

	  HAL_TIM_IC_Start(&htim4, TIM_CHANNEL_1);
	  HAL_TIM_IC_Start(&htim4, TIM_CHANNEL_2);
	  HAL_TIM_IC_Start(&htim4, TIM_CHANNEL_3);
	  HAL_TIM_IC_Start(&htim4, TIM_CHANNEL_4);
	  HAL_TIM_Base_Start_IT(&htim4);

	  HAL_TIM_IC_Start(&htim2, TIM_CHANNEL_1);
	  HAL_TIM_IC_Start(&htim2, TIM_CHANNEL_2);
	  HAL_TIM_IC_Start(&htim2, TIM_CHANNEL_3);
	  HAL_TIM_IC_Start(&htim2, TIM_CHANNEL_4);
	  HAL_TIM_Base_Start_IT(&htim2);

	  HAL_TIM_IC_Start(&htim3, TIM_CHANNEL_1);
	  HAL_TIM_IC_Start(&htim3, TIM_CHANNEL_2);
	  HAL_TIM_IC_Start(&htim3, TIM_CHANNEL_4);
	  HAL_TIM_Base_Start_IT(&htim3);

	  Servo_SetAngle(&servos2[0]);
	  Servo_SetAngle(&servos2[1]);
	  Servo_SetAngle(&servos2[2]);
	  Servo_SetAngle(&servos2[3]);
	  Servo_SetAngle(&servos2[4]);
	  Servo_SetAngle(&servos2[5]);
	  Servo_SetAngle(&servos2[6]);



  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	  while (1)
	  {
		  //Sterowanie2();

		  if (PLC_IN_SUM>0)
		  {
			  PLC_polka=PLC_IN_SUM;
			  PLC_OUT=1;
			  RobotPosition current_position = home_position;

			  Robot_MoveToPosition(servos2, &current_position, home_position, 10.0f, 50);

			  // Przejście do pozycji odbioru
			  Robot_MoveToPosition(servos2, &current_position, pick_position_1, 10.0f, 50);
			  Robot_MoveToPosition(servos2, &current_position, pick_position_2, 10.0f, 50);
			  Robot_MoveToPosition(servos2, &current_position, pick_position_3, 1.0f, 80);
			  Robot_MoveToPosition(servos2, &current_position, pick_position_4, 1.0f, 80);
			  Robot_MoveToPosition(servos2, &current_position, pick_position_5, 1.0f, 80);
			  // Przejście do pozycji chwytania
			  HAL_Delay(2000);
			  Robot_MoveToPosition(servos2, &current_position, grip_position_1, 10.0f, 100);
			  HAL_Delay(2000);
			  Robot_MoveToPosition(servos2, &current_position, grip_position_2, 10.0f, 100);
			  Robot_MoveToPosition(servos2, &current_position, home_grip, 1.0f, 50);



			  switch (PLC_polka) {
			      case 1:
			          Robot_MoveToPosition(servos2, &current_position, shelf1_position_1, 10.0f, 100);
			          Robot_MoveToPosition(servos2, &current_position, shelf1_position_2, 10.0f, 100);
			          HAL_Delay(2000);
			          Robot_MoveToPosition(servos2, &current_position, shelf1_position_3, 1.0f, 80);
			          HAL_Delay(2000);
			          Robot_MoveToPosition(servos2, &current_position, shelf1_position_4, 1.0f, 80);
			          Robot_MoveToPosition(servos2, &current_position, shelf1_position_5, 1.0f, 80);
			          Robot_MoveToPosition(servos2, &current_position, shelf1_position_6, 1.0f, 80);
			          Robot_MoveToPosition(servos2, &current_position, shelf1_position_6, 1.0f, 80);
			          Robot_MoveToPosition(servos2, &current_position, home_position, 1.0f, 80);

			          PLC_polka=0;

			          break;
			      case 2:
			          Robot_MoveToPosition(servos2, &current_position, shelf2_position, 1.0f, 10);
			          break;
			      case 3:
			          Robot_MoveToPosition(servos2, &current_position, shelf3_position, 1.0f, 10);
			          break;
			  }
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

  /** Supply configuration update enable
  */
  HAL_PWREx_ConfigSupply(PWR_DIRECT_SMPS_SUPPLY);

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
  RCC_OscInitStruct.HSIState = RCC_HSI_DIV2;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 30;
  RCC_OscInitStruct.PLL.PLLP = 2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  RCC_OscInitStruct.PLL.PLLR = 2;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_3;
  RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1VCOWIDE;
  RCC_OscInitStruct.PLL.PLLFRACN = 0;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
                              |RCC_CLOCKTYPE_D3PCLK1|RCC_CLOCKTYPE_D1PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV1;
  RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
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
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_CKPER;
  PeriphClkInitStruct.CkperClockSelection = RCC_CLKPSOURCE_HSI;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC3_Init(void)
{

  /* USER CODE BEGIN ADC3_Init 0 */

  /* USER CODE END ADC3_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC3_Init 1 */

  /* USER CODE END ADC3_Init 1 */

  /** Common config
  */
  hadc3.Instance = ADC3;
  hadc3.Init.Resolution = ADC_RESOLUTION_16B;
  hadc3.Init.ScanConvMode = ADC_SCAN_ENABLE;
  hadc3.Init.EOCSelection = ADC_EOC_SEQ_CONV;
  hadc3.Init.LowPowerAutoWait = DISABLE;
  hadc3.Init.ContinuousConvMode = DISABLE;
  hadc3.Init.NbrOfConversion = 7;
  hadc3.Init.DiscontinuousConvMode = DISABLE;
  hadc3.Init.ExternalTrigConv = ADC_EXTERNALTRIG_T8_TRGO;
  hadc3.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_RISING;
  hadc3.Init.ConversionDataManagement = ADC_CONVERSIONDATA_DMA_CIRCULAR;
  hadc3.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc3.Init.LeftBitShift = ADC_LEFTBITSHIFT_NONE;
  hadc3.Init.OversamplingMode = DISABLE;
  if (HAL_ADC_Init(&hadc3) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  sConfig.OffsetSignedSaturation = DISABLE;
  if (HAL_ADC_ConfigChannel(&hadc3, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = ADC_REGULAR_RANK_2;
  if (HAL_ADC_ConfigChannel(&hadc3, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_2;
  sConfig.Rank = ADC_REGULAR_RANK_3;
  if (HAL_ADC_ConfigChannel(&hadc3, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_3;
  sConfig.Rank = ADC_REGULAR_RANK_4;
  if (HAL_ADC_ConfigChannel(&hadc3, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_6;
  sConfig.Rank = ADC_REGULAR_RANK_5;
  if (HAL_ADC_ConfigChannel(&hadc3, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_7;
  sConfig.Rank = ADC_REGULAR_RANK_6;
  if (HAL_ADC_ConfigChannel(&hadc3, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_8;
  sConfig.Rank = ADC_REGULAR_RANK_7;
  if (HAL_ADC_ConfigChannel(&hadc3, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC3_Init 2 */

  /* USER CODE END ADC3_Init 2 */

}

/**
  * @brief ETH Initialization Function
  * @param None
  * @retval None
  */
static void MX_ETH_Init(void)
{

  /* USER CODE BEGIN ETH_Init 0 */

  /* USER CODE END ETH_Init 0 */

   static uint8_t MACAddr[6];

  /* USER CODE BEGIN ETH_Init 1 */

  /* USER CODE END ETH_Init 1 */
  heth.Instance = ETH;
  MACAddr[0] = 0x00;
  MACAddr[1] = 0x80;
  MACAddr[2] = 0xE1;
  MACAddr[3] = 0x00;
  MACAddr[4] = 0x00;
  MACAddr[5] = 0x00;
  heth.Init.MACAddr = &MACAddr[0];
  heth.Init.MediaInterface = HAL_ETH_RMII_MODE;
  heth.Init.TxDesc = DMATxDscrTab;
  heth.Init.RxDesc = DMARxDscrTab;
  heth.Init.RxBuffLen = 1524;

  /* USER CODE BEGIN MACADDRESS */

  /* USER CODE END MACADDRESS */

  if (HAL_ETH_Init(&heth) != HAL_OK)
  {
    Error_Handler();
  }

  memset(&TxConfig, 0 , sizeof(ETH_TxPacketConfig));
  TxConfig.Attributes = ETH_TX_PACKETS_FEATURES_CSUM | ETH_TX_PACKETS_FEATURES_CRCPAD;
  TxConfig.ChecksumCtrl = ETH_CHECKSUM_IPHDR_PAYLOAD_INSERT_PHDR_CALC;
  TxConfig.CRCPadCtrl = ETH_CRC_PAD_INSERT;
  /* USER CODE BEGIN ETH_Init 2 */

  /* USER CODE END ETH_Init 2 */

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
  htim2.Init.Prescaler = 239;
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
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 239;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 9999;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 119;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 99;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */
  HAL_TIM_MspPostInit(&htim4);

}

/**
  * @brief TIM5 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM5_Init(void)
{

  /* USER CODE BEGIN TIM5_Init 0 */

  /* USER CODE END TIM5_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM5_Init 1 */

  /* USER CODE END TIM5_Init 1 */
  htim5.Instance = TIM5;
  htim5.Init.Prescaler = 11999;
  htim5.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim5.Init.Period = 19999;
  htim5.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim5.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim5) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim5, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim5) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim5, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim5, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM5_Init 2 */

  /* USER CODE END TIM5_Init 2 */
  HAL_TIM_MspPostInit(&htim5);

}

/**
  * @brief TIM6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM6_Init(void)
{

  /* USER CODE BEGIN TIM6_Init 0 */

  /* USER CODE END TIM6_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM6_Init 1 */

  /* USER CODE END TIM6_Init 1 */
  htim6.Instance = TIM6;
  htim6.Init.Prescaler = 11999;
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim6.Init.Period = 99;
  htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim6) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim6, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM6_Init 2 */

  /* USER CODE END TIM6_Init 2 */

}

/**
  * @brief TIM7 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM7_Init(void)
{

  /* USER CODE BEGIN TIM7_Init 0 */

  /* USER CODE END TIM7_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM7_Init 1 */

  /* USER CODE END TIM7_Init 1 */
  htim7.Instance = TIM7;
  htim7.Init.Prescaler = 63999;
  htim7.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim7.Init.Period = 1;
  htim7.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim7) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim7, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM7_Init 2 */

  /* USER CODE END TIM7_Init 2 */

}

/**
  * @brief TIM8 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM8_Init(void)
{

  /* USER CODE BEGIN TIM8_Init 0 */

  /* USER CODE END TIM8_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM8_Init 1 */

  /* USER CODE END TIM8_Init 1 */
  htim8.Instance = TIM8;
  htim8.Init.Prescaler = 11999;
  htim8.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim8.Init.Period = 999;
  htim8.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim8.Init.RepetitionCounter = 0;
  htim8.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim8) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim8, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim8, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM8_Init 2 */

  /* USER CODE END TIM8_Init 2 */

}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  huart3.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart3.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart3.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart3, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart3, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

/**
  * @brief USB_OTG_FS Initialization Function
  * @param None
  * @retval None
  */
static void MX_USB_OTG_FS_PCD_Init(void)
{

  /* USER CODE BEGIN USB_OTG_FS_Init 0 */

  /* USER CODE END USB_OTG_FS_Init 0 */

  /* USER CODE BEGIN USB_OTG_FS_Init 1 */

  /* USER CODE END USB_OTG_FS_Init 1 */
  hpcd_USB_OTG_FS.Instance = USB_OTG_FS;
  hpcd_USB_OTG_FS.Init.dev_endpoints = 9;
  hpcd_USB_OTG_FS.Init.speed = PCD_SPEED_FULL;
  hpcd_USB_OTG_FS.Init.dma_enable = DISABLE;
  hpcd_USB_OTG_FS.Init.phy_itface = PCD_PHY_EMBEDDED;
  hpcd_USB_OTG_FS.Init.Sof_enable = DISABLE;
  hpcd_USB_OTG_FS.Init.low_power_enable = DISABLE;
  hpcd_USB_OTG_FS.Init.lpm_enable = DISABLE;
  hpcd_USB_OTG_FS.Init.battery_charging_enable = ENABLE;
  hpcd_USB_OTG_FS.Init.vbus_sensing_enable = ENABLE;
  hpcd_USB_OTG_FS.Init.use_dedicated_ep1 = DISABLE;
  if (HAL_PCD_Init(&hpcd_USB_OTG_FS) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USB_OTG_FS_Init 2 */

  /* USER CODE END USB_OTG_FS_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream0_IRQn);

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
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();
  __HAL_RCC_GPIOE_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, LD1_Pin|LD3_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(PD3_OUT_GPIO_Port, PD3_OUT_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : LD1_Pin LD3_Pin */
  GPIO_InitStruct.Pin = LD1_Pin|LD3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PD1_IN1_Pin PD2_IN2_Pin */
  GPIO_InitStruct.Pin = PD1_IN1_Pin|PD2_IN2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pin : PD3_OUT_Pin */
  GPIO_InitStruct.Pin = PD3_OUT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(PD3_OUT_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

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
