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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define APDS9960_I2C_ADDR 0x39 << 1
#define HDC1080_I2C_ADDR 0x40 << 1

/* On/Off definitions */
#define OFF                     0
#define ON                      1

/* Acceptable parameters for setMode */
#define POWER                   0
#define AMBIENT_LIGHT           1
#define PROXIMITY               2
#define WAIT                    3
#define AMBIENT_LIGHT_INT       4
#define PROXIMITY_INT           5
#define GESTURE                 6
#define ALL                     7

/* LED Drive values */
#define LED_DRIVE_100MA         0
#define LED_DRIVE_50MA          1
#define LED_DRIVE_25MA          2
#define LED_DRIVE_12_5MA        3

/* Proximity Gain (PGAIN) values */
#define PGAIN_1X                0
#define PGAIN_2X                1
#define PGAIN_4X                2
#define PGAIN_8X                3

/* ALS Gain (AGAIN) values */
#define AGAIN_1X                0
#define AGAIN_4X                1
#define AGAIN_16X               2
#define AGAIN_64X               3

/* Gesture Gain (GGAIN) values */
#define GGAIN_1X                0
#define GGAIN_2X                1
#define GGAIN_4X                2
#define GGAIN_8X                3

/* LED Boost values */
#define LED_BOOST_100           0
#define LED_BOOST_150           1
#define LED_BOOST_200           2
#define LED_BOOST_300           3

/* Gesture wait time values */
#define GWTIME_0MS              0
#define GWTIME_2_8MS            1
#define GWTIME_5_6MS            2
#define GWTIME_8_4MS            3
#define GWTIME_14_0MS           4
#define GWTIME_22_4MS           5
#define GWTIME_30_8MS           6
#define GWTIME_39_2MS           7

/* Bit fields */
#define PON            0b00000001
#define AEN            0b00000010
#define PEN            0b00000100
#define WEN            0b00001000
#define AIEN           0b00010000
#define PIEN           0b00100000
#define GEN            0b01000000
#define GVALID         0b00000001

/* Status bit fields */
#define AVALID         0b0000001
#define PVALID         0b0000010
#define GINT           0b0000100
#define AINT           0b0010000
#define PGSAT          0b0100000
#define CPSAT          0b1000000

/* APDS-9960 register addresses */
#define ENABLE         0x80
#define ATIME          0x81
#define WTIME          0x83
#define AILTL          0x84
#define AILTH          0x85
#define AIHTL          0x86
#define AIHTH          0x87
#define PILT           0x89
#define PIHT           0x8B
#define PERS           0x8C
#define CONFIG1        0x8D
#define PPULSE         0x8E
#define CONTROL        0x8F
#define CONFIG2        0x90
#define ID             0x92
#define STATUS         0x93
#define CDATAL         0x94
#define CDATAH         0x95
#define RDATAL         0x96
#define RDATAH         0x97
#define GDATAL         0x98
#define GDATAH         0x99
#define BDATAL         0x9A
#define BDATAH         0x9B
#define PDATA          0x9C
#define POFFSET_UR     0x9D
#define POFFSET_DL     0x9E
#define CONFIG3        0x9F
#define GPENTH         0xA0
#define GEXTH          0xA1
#define GCONF1         0xA2
#define GCONF2         0xA3
#define GOFFSET_U      0xA4
#define GOFFSET_D      0xA5
#define GOFFSET_L      0xA7
#define GOFFSET_R      0xA9
#define GPULSE         0xA6
#define GCONF3         0xAA
#define GCONF4         0xAB
#define GFLVL          0xAE
#define GSTATUS        0xAF
#define IFORCE         0xE4
#define PICLEAR        0xE5
#define CICLEAR        0xE6
#define AICLEAR        0xE7
#define GFIFO_U        0xFC
#define GFIFO_D        0xFD
#define GFIFO_L        0xFE
#define GFIFO_R        0xFF

/* Default values */
#define DEFAULT_ATIME           219     // 103ms
#define DEFAULT_WTIME           246     // 27ms
#define DEFAULT_PROX_PPULSE     0x87    // 16us, 8 pulses
#define DEFAULT_GESTURE_PPULSE  0x89    // 16us, 10 pulses
#define DEFAULT_POFFSET_UR      0       // 0 offset
#define DEFAULT_POFFSET_DL      0       // 0 offset
#define DEFAULT_CONFIG1         0x60    // No 12x wait (WTIME) factor
#define DEFAULT_LDRIVE          LED_DRIVE_100MA
#define DEFAULT_PGAIN           PGAIN_4X
#define DEFAULT_AGAIN           AGAIN_4X
#define DEFAULT_PILT            0       // Low proximity threshold
#define DEFAULT_PIHT            50      // High proximity threshold
#define DEFAULT_AILT            0xFFFF  // Force interrupt for calibration
#define DEFAULT_AIHT            0
#define DEFAULT_PERS            0x11    // 2 consecutive prox or ALS for int.
#define DEFAULT_CONFIG2         0x01    // No saturation interrupts or LED boost
#define DEFAULT_CONFIG3         0       // Enable all photodiodes, no SAI
#define DEFAULT_GPENTH          40      // Threshold for entering gesture mode
#define DEFAULT_GEXTH           30      // Threshold for exiting gesture mode
#define DEFAULT_GCONF1          0x40    // 4 gesture events for int., 1 for exit
#define DEFAULT_GGAIN           GGAIN_4X
#define DEFAULT_GLDRIVE         LED_DRIVE_100MA
#define DEFAULT_GWTIME          GWTIME_2_8MS
#define DEFAULT_GOFFSET         0       // No offset scaling for gesture mode
#define DEFAULT_GPULSE          0xC9    // 32us, 10 pulses
#define DEFAULT_GCONF3          0       // All photodiodes active during gesture
#define DEFAULT_GIEN            0       // Disable gesture interrupts

//MEMORY DEFINES
#define MAX_ADDRESS 0x07F000
#define MIN_ADDRESS 0x000000

//HUMIDITY DEFINES
#define TEMPERATURE 0x00
#define HUMIDITY 0x01

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
ADC_HandleTypeDef hadc2;

I2C_HandleTypeDef hi2c1;

RTC_HandleTypeDef hrtc;

SPI_HandleTypeDef hspi2;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart3;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void PeriphCommonClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_SPI2_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_RTC_Init(void);
static void MX_ADC1_Init(void);
static void MX_ADC2_Init(void);
static void MX_USART1_UART_Init(void);
/* USER CODE BEGIN PFP */
void ADPS_init();
void Ambient_init_();
void writeDataByte(uint8_t reg, uint8_t val);
void readDataByte(uint8_t reg, uint8_t* val);
void setMode(uint8_t mode, uint8_t enable);
uint8_t getMode();
void setLEDDrive(uint8_t drive);
void setProximityGain(uint8_t drive);
void setAmbientLightGain(uint8_t drive);
void setAmbientLightIntEnable(uint8_t enable);
void setProxIntLowThresh(uint8_t threshold);
void setProxIntHighThresh(uint8_t threshold);
void setLightIntLowThresh(uint16_t threshold);
void setLightIntHighThresh (uint16_t threshold);
void enableLightSensor(uint8_t interrupts);
void readAmbientLight(uint16_t *val);
void readRedLight(uint16_t *val);
void readGreenLight(uint16_t *val);
void readBlueLight(uint16_t *val);
void storeData();
void readall();
void getEntryCount();
void readEntry(int pos, UART_HandleTypeDef *huart);
void totalEntries();
void clearMemory();
void getTime();
void deletePage();
void fetchMoisture();
void readTempHum();
void readBattery();
//SETS FOR MEMORY FUNCTIONS




/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
uint16_t ambientLight = 0;
uint8_t tempBits[2];
uint8_t humBits[2];
RTC_DateTypeDef gDate;
RTC_TimeTypeDef gTime;


uint8_t flash_Start_Command = 0x06;
uint8_t flash_Wipe_Command = 0x60;
uint8_t flash_Write_Command = 0x02;
uint8_t flash_Read_Command = 0x03;
uint8_t flash_Read_Register = 0x05;


uint8_t UART_Recieve_Command[10];
uint8_t circleBuffer[5000];
uint8_t UART_Recieve_Buffer[1];
uint8_t SPI_RX[4000] = {0};
int Buffer_Count = 0;

uint32_t end_Address;

uint8_t MSG1[] = "Messages Recieved";
uint8_t tempSend = 0x00;
uint8_t RX_BUFFER[4000];

UART_HandleTypeDef *who;
int channel = 0;
int memory_Position = 0;

uint32_t moisture;
int ratio = 0;
double temp = 0;
double hum = 0;
int batteryLife = 0;



//USE THIS STRUCT AND DEFINITION TO MAKE A DIRECTORY
//EACH OBJECT SHOULD HOLD THE TRUE POSITION IN THE DIRECTORY
//FOR MEMORY REASONS, THE OTHER DIR NUM SHOULD BE USED FOR PUTTING
//ALL THE ENTRIES IN ORDER FOR WHEN WE SEND THOSE VALUES TO THE COMP
int total_Entries = 0;
int cirPos = 0;

int TimeFlag = 0;
int DateFlag = 0;
int hourlyReading = 0;
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
  MX_SPI2_Init();
  MX_USART3_UART_Init();
  MX_RTC_Init();
  MX_ADC1_Init();
  MX_ADC2_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0,GPIO_PIN_SET);

  HAL_UART_Receive_IT(&huart3, UART_Recieve_Buffer, 1);

  int track = 0;

  int reset = 0;
  ADPS_init();
  enableLightSensor(0);
  getEntryCount();

  char dateTime[9];
  int first,second,third;
  RTC_TimeTypeDef sTime = {0};
  RTC_DateTypeDef sDate = {0};

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

	  HAL_Delay(100);
	  if(hourlyReading){
		  hourlyReading = 0;
		  storeData();
	  }
	  UART_Recieve_Command[track] = circleBuffer[cirPos];
	  if(UART_Recieve_Command[track] != 0x00)
		  track++;

	  else{
		  reset++;
		  if(reset >= 100){
				track = 0;
				for(int i = 0; i < 20; i++)
				{
					UART_Recieve_Command[i] = 0x00;
				}
		  }
	  }
	  //DATA AND TIME REGEX CHECKS
	  if(track == 8){
		  for(int i = 0; i < 8; i++){
			  dateTime[i] = UART_Recieve_Command[i];
			  if(dateTime[i] == ':')
				  TimeFlag = 1;
			  else if(dateTime[i] == '/')
				  DateFlag = 1;
		  }
		  dateTime[8] = '/';
		  if(TimeFlag){
			  char *tok;
			  tok = strtok(dateTime,":");
			  first = atoi(tok);
			  tok = strtok(NULL,":");
			  second = atoi(tok);
			  tok = strtok(NULL,"/");
			  third = atoi(tok);
			  sTime.Hours = (first % 10 * 0x01) + ((first / 10) * 0x10);
			  sTime.Minutes = (second % 10 * 0x01) + ((second / 10) * 0x10);
			  sTime.Seconds = (third % 10 * 0x01) + ((third / 10) * 0x10);
			  HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BCD);

		  }
		  if(DateFlag){
			  char *tok;
			  tok = strtok(dateTime,"/");
			  first = atoi(tok);
			  tok = strtok(NULL,"/");
			  second = atoi(tok);
			  tok = strtok(NULL,"/");
			  third = atoi(tok);
			  sDate.Month = (first % 10 * 0x01) + ((first / 10) * 0x10);;
			  sDate.Date = (second % 10 * 0x01) + ((second / 10) * 0x10);
			  sDate.Year = (third % 10 * 0x01) + ((third / 10) * 0x10);
			  HAL_RTC_SetDate(&hrtc, &sDate, RTC_FORMAT_BCD);
		  }
		  TimeFlag = 0;
		  DateFlag = 0;
	  }

	  circleBuffer[cirPos] = 0x00;
	  if(cirPos != Buffer_Count)
	  cirPos++;
	  if(cirPos == 4000)
		  cirPos = 0;
	  if(track == 10)
		  track = 0;
	  if(track == 3)
		  track = 3;


	  if(channel == 0){
		  who = &huart3;
	  }
	    if(!strncmp((char*)UART_Recieve_Command, "read", 4)){
	    	storeData();
			track = 0;
			char *MSGNO = "DATA READ";
			for(int i = 0; i < strlen(MSGNO); i++){
				tempSend = MSGNO[i];
				HAL_UART_Transmit(who, &tempSend, sizeof(tempSend), 100);
			}
			tempSend = '\n';
			HAL_UART_Transmit(who, &tempSend, sizeof(tempSend), 100);
			for(int i = 0; i < 20; i++)
			{
				UART_Recieve_Command[i] = 0x00;
			}
	    }
	    else if(!strncmp((char*)UART_Recieve_Command, "last", 4)){
	    	if(total_Entries == 0){
	    		char *MSGNO = "NO ENTRIES";
	    		for(int i = 0; i < strlen(MSGNO); i++){
	    			tempSend = MSGNO[i];
	    			HAL_UART_Transmit(who, &tempSend, sizeof(tempSend), 100);
	    		}
	    	}else{
	    		readEntry(total_Entries, who);
	    	}
	    	tempSend = '\n';
	    	HAL_UART_Transmit(who, &tempSend, sizeof(tempSend), 100);
	    	track = 0;
			for(int i = 0; i < 20; i++)
			{
				UART_Recieve_Command[i] = 0x00;
			}
	    }
	    else if(!strncmp((char*)UART_Recieve_Command, "all", 3)){
	    	if(total_Entries == 0){
	    		char *MSGNO = "NO ENTRIES";
				for(int i = 0; i < strlen(MSGNO); i++){
					tempSend = MSGNO[i];
					HAL_UART_Transmit(who, &tempSend, sizeof(tempSend), 100);
				}
			}else{
				for(int i = 1; i <= total_Entries; i++){
					readEntry(i, who);
					tempSend = '\n';
					HAL_UART_Transmit(who, &tempSend, sizeof(tempSend), 100);
				}

			}

			track = 0;
			for(int i = 0; i < 20; i++)
			{
				UART_Recieve_Command[i] = 0x00;
			}

	    }
	    else if(!strncmp((char*)UART_Recieve_Command , "entries", 7)){
	    	totalEntries();
	    	track = 0;
			for(int i = 0; i < 20; i++)
			{
				UART_Recieve_Command[i] = 0x00;
			}
	    }

	    else if(!strncmp((char*)UART_Recieve_Command, "clear", 5)){
	    		clearMemory();
	    		total_Entries = 0;
				char *MSGNO = "DATA CLEARED";
				for(int i = 0; i < strlen(MSGNO); i++){
					tempSend = MSGNO[i];
					HAL_UART_Transmit(who, &tempSend, sizeof(tempSend), 100);
				}
				tempSend = '\n';
				HAL_UART_Transmit(who, &tempSend, sizeof(tempSend), 100);
				track = 0;
				for(int i = 0; i < 20; i++)
				{
					UART_Recieve_Command[i] = 0x00;
				}
	    	}
	    HAL_PWR_EnterSLEEPMode(PWR_MAINREGULATOR_ON,PWR_SLEEPENTRY_WFI);
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
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure LSE Drive Capability
  */
  HAL_PWR_EnableBkUpAccess();
  __HAL_RCC_LSEDRIVE_CONFIG(RCC_LSEDRIVE_LOW);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_LSI
                              |RCC_OSCILLATORTYPE_LSE|RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = 0;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_MSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }

  /** Enable MSI Auto calibration
  */
  HAL_RCCEx_EnableMSIPLLMode();
}

/**
  * @brief Peripherals Common Clock Configuration
  * @retval None
  */
void PeriphCommonClock_Config(void)
{
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the peripherals clock
  */
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCCLKSOURCE_PLLSAI1;
  PeriphClkInit.PLLSAI1.PLLSAI1Source = RCC_PLLSOURCE_HSI;
  PeriphClkInit.PLLSAI1.PLLSAI1M = 1;
  PeriphClkInit.PLLSAI1.PLLSAI1N = 8;
  PeriphClkInit.PLLSAI1.PLLSAI1P = RCC_PLLP_DIV7;
  PeriphClkInit.PLLSAI1.PLLSAI1Q = RCC_PLLQ_DIV2;
  PeriphClkInit.PLLSAI1.PLLSAI1R = RCC_PLLR_DIV2;
  PeriphClkInit.PLLSAI1.PLLSAI1ClockOut = RCC_PLLSAI1_ADC1CLK;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_MultiModeTypeDef multimode = {0};
  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc1.Init.OversamplingMode = DISABLE;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure the ADC multi-mode
  */
  multimode.Mode = ADC_MODE_INDEPENDENT;
  if (HAL_ADCEx_MultiModeConfigChannel(&hadc1, &multimode) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_6;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_2CYCLES_5;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief ADC2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC2_Init(void)
{

  /* USER CODE BEGIN ADC2_Init 0 */

  /* USER CODE END ADC2_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC2_Init 1 */

  /* USER CODE END ADC2_Init 1 */

  /** Common config
  */
  hadc2.Instance = ADC2;
  hadc2.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
  hadc2.Init.Resolution = ADC_RESOLUTION_12B;
  hadc2.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc2.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc2.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc2.Init.LowPowerAutoWait = DISABLE;
  hadc2.Init.ContinuousConvMode = DISABLE;
  hadc2.Init.NbrOfConversion = 1;
  hadc2.Init.DiscontinuousConvMode = DISABLE;
  hadc2.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc2.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc2.Init.DMAContinuousRequests = DISABLE;
  hadc2.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc2.Init.OversamplingMode = DISABLE;
  if (HAL_ADC_Init(&hadc2) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_7;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_2CYCLES_5;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC2_Init 2 */

  /* USER CODE END ADC2_Init 2 */

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
  hi2c1.Init.Timing = 0x00100D14;
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
  * @brief RTC Initialization Function
  * @param None
  * @retval None
  */
static void MX_RTC_Init(void)
{

  /* USER CODE BEGIN RTC_Init 0 */

  /* USER CODE END RTC_Init 0 */

  RTC_TimeTypeDef sTime = {0};
  RTC_DateTypeDef sDate = {0};
  RTC_AlarmTypeDef sAlarm = {0};

  /* USER CODE BEGIN RTC_Init 1 */

  /* USER CODE END RTC_Init 1 */

  /** Initialize RTC Only
  */
  hrtc.Instance = RTC;
  hrtc.Init.HourFormat = RTC_HOURFORMAT_24;
  hrtc.Init.AsynchPrediv = 127;
  hrtc.Init.SynchPrediv = 255;
  hrtc.Init.OutPut = RTC_OUTPUT_DISABLE;
  hrtc.Init.OutPutRemap = RTC_OUTPUT_REMAP_NONE;
  hrtc.Init.OutPutPolarity = RTC_OUTPUT_POLARITY_HIGH;
  hrtc.Init.OutPutType = RTC_OUTPUT_TYPE_OPENDRAIN;
  if (HAL_RTC_Init(&hrtc) != HAL_OK)
  {
    Error_Handler();
  }

  /* USER CODE BEGIN Check_RTC_BKUP */

  /* USER CODE END Check_RTC_BKUP */

  /** Initialize RTC and set the Time and Date
  */
  sTime.Hours = 0x22;
  sTime.Minutes = 0x59;
  sTime.Seconds = 0x55;
  sTime.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
  sTime.StoreOperation = RTC_STOREOPERATION_RESET;
  if (HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BCD) != HAL_OK)
  {
    Error_Handler();
  }
  sDate.WeekDay = RTC_WEEKDAY_SATURDAY;
  sDate.Month = RTC_MONTH_FEBRUARY;
  sDate.Date = 0x17;
  sDate.Year = 0x24;

  if (HAL_RTC_SetDate(&hrtc, &sDate, RTC_FORMAT_BCD) != HAL_OK)
  {
    Error_Handler();
  }

  /** Enable the Alarm A
  */
  sAlarm.AlarmTime.Hours = 0x0;
  sAlarm.AlarmTime.Minutes = 0x0;
  sAlarm.AlarmTime.Seconds = 0x0;
  sAlarm.AlarmTime.SubSeconds = 0x0;
  sAlarm.AlarmTime.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
  sAlarm.AlarmTime.StoreOperation = RTC_STOREOPERATION_RESET;
  sAlarm.AlarmMask = RTC_ALARMMASK_DATEWEEKDAY|RTC_ALARMMASK_HOURS;
  sAlarm.AlarmSubSecondMask = RTC_ALARMSUBSECONDMASK_ALL;
  sAlarm.AlarmDateWeekDaySel = RTC_ALARMDATEWEEKDAYSEL_DATE;
  sAlarm.AlarmDateWeekDay = 0x1;
  sAlarm.Alarm = RTC_ALARM_A;
  if (HAL_RTC_SetAlarm_IT(&hrtc, &sAlarm, RTC_FORMAT_BCD) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN RTC_Init 2 */

  /* USER CODE END RTC_Init 2 */

}

/**
  * @brief SPI2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI2_Init(void)
{

  /* USER CODE BEGIN SPI2_Init 0 */

  /* USER CODE END SPI2_Init 0 */

  /* USER CODE BEGIN SPI2_Init 1 */

  /* USER CODE END SPI2_Init 1 */
  /* SPI2 parameter configuration*/
  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_MASTER;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES;
  hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi2.Init.NSS = SPI_NSS_SOFT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 7;
  hspi2.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi2.Init.NSSPMode = SPI_NSS_PULSE_DISABLE;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI2_Init 2 */

  /* USER CODE END SPI2_Init 2 */

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
  huart1.Init.BaudRate = 57600;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

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
  huart3.Init.BaudRate = 57600;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  huart3.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart3.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

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
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PB0 */
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void writeDataByte(uint8_t reg, uint8_t val){
	uint8_t send[2];
	send[0] = reg;
	send[1] = val;
	HAL_I2C_Master_Transmit(&hi2c1,(uint16_t)APDS9960_I2C_ADDR, send,sizeof(send), 1000);
	//HAL_Delay(100);
}
void readDataByte(uint8_t reg, uint8_t* val){
	HAL_I2C_Master_Transmit(&hi2c1,(uint16_t)APDS9960_I2C_ADDR, &reg,sizeof(reg), 1000);

	HAL_I2C_Master_Receive(&hi2c1,(uint16_t)APDS9960_I2C_ADDR, val,1, 1000);
	//HAL_Delay(100);
}
void setMode(uint8_t mode, uint8_t enable){
	uint8_t reg_val;

	reg_val = getMode();

	enable = enable & 0x01;
	if(mode >= 0 && mode <= 6){
		if(enable){
			reg_val |= (1 << mode);
		}else {
			reg_val &= ~(1 << mode);
		}
	} else if( mode == ALL){
		if(enable){
			reg_val = 0x7F;
		} else {
			reg_val = 0x00;
		}
	}

	writeDataByte(ENABLE, reg_val);
}
uint8_t getMode(){
	uint8_t enable_value;

	readDataByte(ENABLE, &enable_value);
	return enable_value;
}
void ADPS_init(){
	setMode(ALL, OFF);
	writeDataByte(ATIME, DEFAULT_ATIME);
	writeDataByte(WTIME,DEFAULT_WTIME);
	writeDataByte(PPULSE, DEFAULT_PROX_PPULSE);
	writeDataByte(POFFSET_UR, DEFAULT_POFFSET_UR);
	writeDataByte(POFFSET_DL, DEFAULT_POFFSET_DL);
	writeDataByte(CONFIG1, DEFAULT_CONFIG1);
	setLEDDrive(DEFAULT_LDRIVE);
	setProximityGain(DEFAULT_PGAIN);
	setAmbientLightGain(DEFAULT_AGAIN);
	setProxIntLowThresh(DEFAULT_PILT);
	setProxIntHighThresh(DEFAULT_PIHT);
	setLightIntLowThresh(DEFAULT_AILT);
	setLightIntHighThresh(DEFAULT_AIHT);
	writeDataByte(PERS, DEFAULT_PERS);
	writeDataByte(CONFIG2, DEFAULT_CONFIG2);
	writeDataByte(CONFIG3, DEFAULT_CONFIG3);
}
void setLEDDrive(uint8_t drive){
	uint8_t val;

	readDataByte(CONTROL, &val);

	drive &= 0b00000011;
	drive = drive << 6;
	val &= 0b00111111;
	val |= drive;

	writeDataByte(CONTROL, val);
}
void setAmbientLightIntEnable(uint8_t enable){
	uint8_t val;

	readDataByte(ENABLE, &val);

	enable &= 0b00000001;
	enable = enable << 4;
	val &= 0b11101111;
	val |= enable;

	writeDataByte(ENABLE, val);
}
void setProximityGain(uint8_t drive){
	uint8_t val;

	readDataByte(CONTROL, &val);

	drive &= 0b00000011;
	drive = drive << 2;
	val &= 0b00111111;
	val |= drive;

	writeDataByte(CONTROL, val);
}
void setAmbientLightGain(uint8_t drive){
	uint8_t val;

	readDataByte(CONTROL, &val);

	drive &= 0b00000011;
	val &= 0b00111111;
	val |= drive;

	writeDataByte(CONTROL, val);
}
void setProxIntLowThresh(uint8_t threshold){
	writeDataByte(PILT, threshold);
}
void setProxIntHighThresh(uint8_t threshold){
	writeDataByte(PIHT, threshold);
}
void setLightIntLowThresh(uint16_t threshold){
	uint8_t val_low;
	uint8_t val_high;

	val_low = threshold & 0x00FF;
	val_high = (threshold & 0xFF00) >> 8;

	writeDataByte(AILTL, val_low);
	writeDataByte(AILTH, val_high);
}
void setLightIntHighThresh (uint16_t threshold){
	uint8_t val_low;
	uint8_t val_high;

	val_low = threshold & 0x00FF;
	val_high = (threshold & 0xFF00) >> 8;

	writeDataByte(AIHTL, val_low);
	writeDataByte(AIHTH, val_high);
}

void enableLightSensor(uint8_t interrupts){

	setAmbientLightGain(DEFAULT_AGAIN);
	if(interrupts){
		setAmbientLightIntEnable(1);
	}else {
		setAmbientLightIntEnable(0);
	}
	setMode(POWER,1);
	setMode(AMBIENT_LIGHT, 1);
}
void readAmbientLight(uint16_t *val){
	uint8_t val_bytes;
	*val = 0;

	readDataByte(CDATAL, &val_bytes);
	*val = val_bytes;

	readDataByte(CDATAH, &val_bytes);
	*val = *val + ((uint16_t)val_bytes << 8);
}
void readRedLight(uint16_t *val){
	uint8_t val_bytes;
	*val = 0;

	readDataByte(RDATAL, &val_bytes);
	*val = val_bytes;

	readDataByte(RDATAH, &val_bytes);
	*val = *val + ((uint16_t)val_bytes << 8);
}
void readGreenLight(uint16_t *val){
	uint8_t val_bytes;
	*val = 0;

	readDataByte(GDATAL, &val_bytes);
	*val = val_bytes;

	readDataByte(GDATAH, &val_bytes);
	*val = *val + ((uint16_t)val_bytes << 8);
}
void readBlueLight(uint16_t *val){
	uint8_t val_bytes;
	*val = 0;

	readDataByte(BDATAL, &val_bytes);
	*val = val_bytes;

	readDataByte(BDATAH, &val_bytes);
	*val = *val + ((uint16_t)val_bytes << 8);
}
void storeData(){
	//Do all the read functions then store that data into memory
	uint8_t Write_Enable = 0x06;
	uint8_t Page_Program = 0x02;
	char storage[256];
	uint8_t SPI_storage[256];
	tempSend = 0x00;
	uint8_t tx[4];
	uint32_t address;

	readall();

	for(int i = 0; i < 256; i++)
		SPI_storage[i] = '\0';

	sprintf(storage, "DATE:%02d-%02d-%02d\nTIME:%02d:%02d:%02d\nAMBIENT LIGHT(0-4000): %u\nTEMPERATURE: %0.2lfF\nHUMIDITY: %d%%\nSOIL MOISTURE(0-100): %d\nBATTERY: %d%%\n",gDate.Month,gDate.Date,gDate.Year, gTime.Hours, gTime.Minutes, gTime.Seconds, (unsigned int)ambientLight, temp, (int)hum, ratio, batteryLife);
	for (int i = 0; i < strlen(storage); i++){
		SPI_storage[i] = storage[i];
	}

	//Clear the page being written too this keeps the circular buffer alive
	deletePage();

	//ENABLE WRITE
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0,GPIO_PIN_RESET);
	for (int i = 0; i < 10; i++);
	HAL_SPI_Transmit(&hspi2, &Write_Enable, sizeof(Write_Enable), 1000);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0,GPIO_PIN_SET);
	for (int i = 0; i < 10; i++);

	//START CS PIN
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0,GPIO_PIN_RESET);
	for (int i = 0; i < 10; i++);

	//TRANSMIT ADDRESS WITH FIRST COMMAND
	address = 0x100 * total_Entries;
	tx[0] = Page_Program;
	tx[1] = (address & 0x00FF0000) >> 16;
	tx[2] = (address & 0x0000FF00) >> 8;
	tx[3] = (address & 0x000000FF);

	HAL_SPI_Transmit(&hspi2,tx, 4, 100);

	//TRANSMIT BUFFER MAX SIZE 256 BYTES
	HAL_SPI_Transmit(&hspi2,SPI_storage, sizeof(SPI_storage), 1000);

	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0,GPIO_PIN_SET);
	for (int i = 0; i < 10; i++);
	total_Entries++;
	//This is 2 months of data saves at 1 hour intervals
	if(total_Entries == 1600){
		total_Entries = 0;
		clearMemory();
		total_Entries = 0;
	}

}
void readall(){
	//call all the read functions
	readAmbientLight(&ambientLight);
	getTime();
	fetchMoisture();
	readTempHum();
	readBattery();
}
void getEntryCount(){
	total_Entries = 0;
	uint8_t Read_Data = 0x03;
	uint8_t tx[4];
	uint32_t Address = 0;
	uint8_t check_bytes[256];

	for(int i = 0; i < 256; i++){
		check_bytes[i] = 0x00;
	}
	do{
		Address = 0x100 * total_Entries;
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0,GPIO_PIN_RESET);
		for (int j = 0; j < 10; j++);

		//SEND COMMAND AND ADDRESSING
		tx[0] = Read_Data;
		tx[1] = (Address & 0x00FF0000) >> 16;
		tx[2] = (Address & 0x0000FF00) >> 8;
		tx[3] = (Address & 0x000000FF);
		HAL_SPI_Transmit(&hspi2,tx, 4, 100);

		HAL_SPI_Receive(&hspi2, check_bytes, sizeof(check_bytes) , 2000);

		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0,GPIO_PIN_SET);
		for (int j = 0; j < 10; j++);

		if((check_bytes[0] != 0x00) && (check_bytes[0] != 0xFF)){
			total_Entries++;
		}
	}while(check_bytes[0] != 0xFF && check_bytes[0] != 0x00);
}
void readEntry(int pos, UART_HandleTypeDef *huart){
	uint32_t Address = (pos - 1) * 0x0000100;
	uint8_t Read_Data = 0x03;
	uint8_t read_buff[256];
	uint8_t tx[4];
	uint8_t tempSend = 0x00;
	char *MSG = "EMPTY\n";

	for(int i = 0; i < 256; i++)
			read_buff[i] = '\0';

	//ENABLE CS PIN
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0,GPIO_PIN_RESET);
	for (int i = 0; i < 10; i++);

	//SEND COMMAND AND ADDRESSING
	tx[0] = Read_Data;
	tx[1] = (Address & 0x00FF0000) >> 16;
	tx[2] = (Address & 0x0000FF00) >> 8;
	tx[3] = (Address & 0x000000FF);
	HAL_SPI_Transmit(&hspi2,tx, 4, 100);

	HAL_SPI_Receive(&hspi2, read_buff, sizeof(read_buff) , 2000);

	//END SPI READ
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0,GPIO_PIN_SET);
	for (int i = 0; i < 10; i++);

	//SEND VALUES TO COMP VIA USART
	if((read_buff[0] == 0xFF) || (read_buff[0] == 0x00)){
		for(int i = 0; i <= sizeof(MSG); i++){
			tempSend = MSG[i];
			if((tempSend != 0x00) && (tempSend != 0xFF))
				HAL_UART_Transmit(huart, &tempSend, sizeof(tempSend), 1000);
			tempSend = 0x00;
		}
	}
	else{
		for(int j = 0; j < 256; j++){
				tempSend = read_buff[j];
				if((read_buff[j] != 0x00) && (read_buff[j] != 0xFF)){
					HAL_UART_Transmit(huart, &read_buff[j], 1, 100);
					HAL_Delay(5);
				}
			}
	}

}
void totalEntries(){
	tempSend = total_Entries;
	char sendStuff[50];
	sprintf(sendStuff, "%d TOTAL DATA ENTRIES\n",tempSend);
	for(int i = 0; i < strlen(sendStuff); i++){
		tempSend = sendStuff[i];
		HAL_UART_Transmit(who, &tempSend, sizeof(tempSend), 100);
	}
}
void clearMemory(){
	//LATCH CS PIN
    uint8_t Erase_Chip = 0x60;
    uint8_t Write_Enable = 0x06;

    //WEL BIT ENABLED
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0,GPIO_PIN_RESET);
	for (int i = 0; i < 10; i++);
	HAL_SPI_Transmit(&hspi2, &Write_Enable, sizeof(Write_Enable), 100);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0,GPIO_PIN_SET);
	for (int i = 0; i < 10; i++);

	//SEND CLEAR CHIP SIGNAL
    HAL_GPIO_WritePin(GPIOB,GPIO_PIN_0,RESET);     // CS to low
	for (int i = 0; i < 10; i++);
    HAL_SPI_Transmit(&hspi2,&Erase_Chip,sizeof(Erase_Chip),100);   // Erase Chip Command
    HAL_GPIO_WritePin(GPIOB,GPIO_PIN_0,SET);       // CS to high
	for (int i = 0; i < 1000; i++);
	//RELEASE CS PIN
}
void getTime(){
	HAL_RTC_GetTime(&hrtc, &gTime, RTC_FORMAT_BIN);
	HAL_RTC_GetDate(&hrtc, &gDate, RTC_FORMAT_BIN);
}
void deletePage(){
	uint8_t Write_Enable = 0x06;
	uint8_t Page_Erase = 0x81;
	uint8_t tx[4];
	uint32_t Address = total_Entries * 0x0000100;

	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0,GPIO_PIN_RESET);
	for (int i = 0; i < 10; i++);
	HAL_SPI_Transmit(&hspi2, &Write_Enable, sizeof(Write_Enable), 1000);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0,GPIO_PIN_SET);
	for (int i = 0; i < 10; i++);

	//START CS PIN
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0,GPIO_PIN_RESET);
	for (int i = 0; i < 10; i++);

	//TRANSMIT ADDRESS WITH FIRST COMMAND
	tx[0] = Page_Erase;
	tx[1] = (Address & 0x00FF0000) >> 16;
	tx[2] = (Address & 0x0000FF00) >> 8;
	tx[3] = (Address & 0x000000FF);

	HAL_SPI_Transmit(&hspi2,tx, 4, 100);

	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0,GPIO_PIN_SET);
	for (int i = 0; i < 10; i++);
	//spam swap until 128
}
void fetchMoisture(){
	double temp;
	HAL_ADC_Start(&hadc1);
	HAL_ADC_PollForConversion(&hadc1, HAL_MAX_DELAY);
	moisture = HAL_ADC_GetValue(&hadc1);
	temp = ((((double)moisture - 2700.0)/ 1300.0) * 100.0);
	ratio = 100 - temp;
}
void readBattery(){
	double works;
	HAL_ADC_Start(&hadc2);
	HAL_ADC_PollForConversion(&hadc2, HAL_MAX_DELAY);
	works = HAL_ADC_GetValue(&hadc2);
	batteryLife = (works / 1300.0) * 100;

}
void readTempHum(){
	uint8_t data[4];
	uint8_t reg[2];
	uint16_t tempConversion = 0, humConversion = 0;

	//init for reading humidity and temp in one transaction
	reg[0] = 0x02;
	reg[1] = 0x10;
	HAL_I2C_Master_Transmit(&hi2c1,(uint16_t)HDC1080_I2C_ADDR, reg,sizeof(reg), 1000);
	HAL_I2C_Master_Receive(&hi2c1,(uint16_t)HDC1080_I2C_ADDR, tempBits, 2, 1000);



	uint8_t ref = 0x00;
	HAL_I2C_Master_Transmit(&hi2c1,(uint16_t)HDC1080_I2C_ADDR, &ref,sizeof(ref), 1000);

	for(int i = 0; i < 30000; i++);

	HAL_I2C_Master_Receive(&hi2c1,(uint16_t)HDC1080_I2C_ADDR, data, 4, 1000);

	tempConversion = tempConversion << 8 | data[0];
	tempConversion = tempConversion << 8 | data[1];
	humConversion = humConversion << 8 | data[2];
	humConversion = humConversion << 8 | data[3];

	temp = (((tempConversion) / 65536.0) * 165.0) - 40.0;
	hum = (((humConversion) / 65536.0) * 100.0);

	//temp C to F
	temp = (temp  * 9.0/5.0) + 32;



}
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){
	circleBuffer[Buffer_Count++] = UART_Recieve_Buffer[0];


	if(Buffer_Count == 4000)
		Buffer_Count = 0;
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
