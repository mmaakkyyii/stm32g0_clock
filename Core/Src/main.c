/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

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
I2C_HandleTypeDef hi2c2;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C2_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
uint8_t disp_addr=0x7c;
int writedata(uint8_t rw,uint8_t data){
	uint8_t buf[]={rw,data};
	int ret=HAL_I2C_Master_Transmit(&hi2c2, disp_addr, &buf, 2, 1000);
	HAL_Delay(1);
	return ret;
}

void initdisplay(){
	  HAL_Delay(50);

	writedata(0x00,0x38);
	writedata(0x00,0x39);
	writedata(0x00,0x14);
	writedata(0x00,0x70);
	writedata(0x00,0x56);
	writedata(0x00,0x6c);
	HAL_Delay(200);
	writedata(0x00,0x38);
	writedata(0x00,0xc);

	writedata(0x00,0x01);

}

void printkawaii(){
	  writedata(0x40,0b10110110);
	  writedata(0x40,0b11011100);
	  writedata(0x40,0b10110010);
	  writedata(0x40,0b10110010);

}

void print(const char *p,int col){
	if(col==0)  writedata(0x00,0x80);
	if(col==1)  writedata(0x00,0x80+0x40);

	for(;*p;p++){
		writedata(0x40,*p);
	}

}

int SERIAL_STRING;
unsigned char DATA_U,DATA_D,DATA_L,DATA_R;
unsigned char OLD_U,OLD_D,OLD_L,OLD_R;
unsigned char work;
unsigned char U_PEAK_END_FLAG,D_PEAK_END_FLAG,L_PEAK_END_FLAG,R_PEAK_END_FLAG;
unsigned char STATUS_UD,STATUS_LR;
unsigned char OLD_STATUS_UD,OLD_STATUS_LR;
unsigned char DISP_FLAG;
unsigned char NOISE_LEVEL = 2;
unsigned char DECIDE_FLAG;
unsigned int  PHASE_COUNTER;
unsigned int  U_PEAK,D_PEAK,L_PEAK,R_PEAK;

void RESET_VARIABLE(void)
{
  PHASE_COUNTER = 0;
  U_PEAK=0;
  D_PEAK=0;
  L_PEAK=0;
  R_PEAK=0;
  OLD_U = 0;
  OLD_D = 0;
  OLD_L = 0;
  OLD_R = 0;
  U_PEAK_END_FLAG = 0;
  D_PEAK_END_FLAG = 0;
  L_PEAK_END_FLAG = 0;
  R_PEAK_END_FLAG = 0;
  STATUS_UD = 0;
  STATUS_LR = 0;
  OLD_STATUS_UD = 0;
  OLD_STATUS_LR = 0;
  SERIAL_STRING = 0;
  DISP_FLAG = 0;
  DECIDE_FLAG = 0;
}


int read_APDS9960(int addr){
	uint8_t buff[1]={0};
	int a1=HAL_I2C_Master_Transmit(&hi2c2, 0x39<<1, &addr, 1, 1000);
	int a2=HAL_I2C_Master_Receive(&hi2c2, (0x39<<1)+1, &buff, 1, 1000);

	return buff[0];
}

int write_APDS9960(int addr,int data){
	uint8_t buff[2]={addr,data};
	int ret=HAL_I2C_Master_Transmit(&hi2c2, 0x39<<1, &buff, 2, 1000);
	return ret;
}

int read_L3GD20H(int addr){
	uint8_t buff[1]={0};
	int a1=HAL_I2C_Master_Transmit(&hi2c2, 0x6A, &addr, 1, 1000);
	int a2=HAL_I2C_Master_Receive(&hi2c2, 0x6A+1, &buff, 1, 1000);

	return buff[0];
}

int write_L3GD20H(int addr,int data){
	uint8_t buff[2]={addr,data};
	int ret=HAL_I2C_Master_Transmit(&hi2c2, 0x6A<<1, &buff, 2, 1000);
	return ret;
}


int read_RTC(int addr){
	uint8_t buff[1]={0};
	int a1=HAL_I2C_Master_Transmit(&hi2c2, 0b11010000, &addr, 1, 1000);
	int a2=HAL_I2C_Master_Receive(&hi2c2,  0b11010000, &buff, 1, 1000);

	return buff[0];
}

int write_RTC(int addr,int data){
	uint8_t buff[2]={addr,data};
	int ret=HAL_I2C_Master_Transmit(&hi2c2, 0b11010000, &buff, 2, 1000);
	return ret;
}

void setRTC(int sec,int min,int hour,int day,int date, int month, int year){
	  write_RTC(0x00,(sec/10)<<4 | (sec%10));
	  write_RTC(0x01,(min/10)<<4 | (min%10));
	  write_RTC(0x02,(hour/10)<<4 | (hour%10));
	  write_RTC(0x03,day);
	  write_RTC(0x04,(date/10)<<4 | (date%10));
	  write_RTC(0x05,(month/10)<<4 | (month%10));
	  write_RTC(0x06,((year-2000)/10)<<4 | ((year-2000)%10));
}

void DATA_SYORI(void)
{
  if (DATA_U > OLD_U)                //IF NEW_DATA > OLD_DATA_BUFFER(APROACH TO PEAK)
  {
    OLD_U = DATA_U;                  //SAVE NEW_DATA TO OLD_DATA_BUFFER
    U_PEAK = PHASE_COUNTER;          //PEAK_PHASE RENEWAL
    U_PEAK_END_FLAG = 0;             //STILL PEAK or APROACH TO PEAK
  }
  else
  {
    U_PEAK_END_FLAG = 1;             //PEAK WAS GONE
  }
  //**************************
  if (DATA_D > OLD_D)
  {
    OLD_D = DATA_D;
    D_PEAK = PHASE_COUNTER;
    D_PEAK_END_FLAG = 0;
  }
  else
  {
    D_PEAK_END_FLAG = 1;
  }
  //**************************
  if (DATA_L > OLD_L)
  {
    OLD_L = DATA_L;
    L_PEAK = PHASE_COUNTER;
    L_PEAK_END_FLAG = 0;
  }
  else
  {
    L_PEAK_END_FLAG = 1;
  }
  //*************************
  if (DATA_R > OLD_R)
  {
    OLD_R = DATA_R;
    R_PEAK = PHASE_COUNTER;
    R_PEAK_END_FLAG = 0;
  }
  else
  {
    R_PEAK_END_FLAG = 1;
  }
  //**************************
  if(U_PEAK_END_FLAG && D_PEAK_END_FLAG && L_PEAK_END_FLAG && R_PEAK_END_FLAG) //IF ALL PEAK WAS GONE
  {
    DECIDE_FLAG = 0;
    if ((U_PEAK > D_PEAK) & (U_PEAK >= L_PEAK) & (U_PEAK >= R_PEAK))           //U_PEAK WAS LAST
    {
      SERIAL_STRING = 1;
      DECIDE_FLAG = 1;
    }
    if ((D_PEAK > U_PEAK) & (D_PEAK >= L_PEAK) & (D_PEAK >= R_PEAK))           //D_PEAK WAS LAST
    {
      SERIAL_STRING = 2;
      DECIDE_FLAG = 1;
    }
    if ((L_PEAK >= U_PEAK) & (L_PEAK >= D_PEAK) & (L_PEAK > R_PEAK))           //L_PEAK WAS LAST
    {
      SERIAL_STRING = 3;
      DECIDE_FLAG =1;
    }
    if ((R_PEAK >= U_PEAK) & (R_PEAK >= D_PEAK) & (R_PEAK > L_PEAK))           //R_PEAK WAS LAST
    {
      SERIAL_STRING = 4;
      DECIDE_FLAG = 1;
    }
    if (!DECIDE_FLAG)SERIAL_STRING = 0;                                   //CAN'T DECIDE
 }
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
  MX_I2C2_Init();
  /* USER CODE BEGIN 2 */
  uint8_t data;

  initdisplay();
  print("Hello world",0);

  writedata(0x00,0x80+0x40);
  printkawaii();
  writedata(0x40,'!');
  HAL_Delay(500);

  print("                ",0);
  print("                ",1);

  HAL_Delay(50);

  uint8_t whoami=read_APDS9960(0x92);
  uint8_t whoami_acc=read_L3GD20H(0x0f);

  write_APDS9960(0x80,0b01000101);
  write_APDS9960(0x90,0b00110000);
  write_APDS9960(0xA3,0b01100100);

  write_APDS9960(0xA4,70);
  write_APDS9960(0xA5,0);
  write_APDS9960(0xA7,10);
  write_APDS9960(0xA9,34);

  write_APDS9960(0xAB,0b00000001);


  write_RTC(0x00,read_RTC(0x00)&0b01111111);
/* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_12);
	  HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_9);
	  HAL_Delay(1);
	  uint8_t s[16]={};

	  int work=read_APDS9960(0xAE);

	  work = read_APDS9960(0xAE);  //READ GESTUR FIFO LEVEL REGISTER
	  if(work != 0)           //IF FIFO HAS SOME DATA
	  {
	    DATA_U = read_APDS9960(0xFC);
	    DATA_D = read_APDS9960(0xFD);
	    DATA_L = read_APDS9960(0xFE);
	    DATA_R = read_APDS9960(0xFF);
	    if((DATA_U > NOISE_LEVEL) && (DATA_D > NOISE_LEVEL)&& (DATA_L> NOISE_LEVEL) && (DATA_R > NOISE_LEVEL)) //NOISE CANCEL
	    {
	      DATA_SYORI();       //
	      PHASE_COUNTER++;    //
	      DISP_FLAG = 1;      //
	    }
	    else
	    {
	      if(DISP_FLAG)
	      {
	        DISP_FLAG = 0;
	      }
	      RESET_VARIABLE();
	    }
	  }


	  int sec_addr=read_RTC(0x00);
	  int min_addr=read_RTC(0x01);
	  int hour_addr=read_RTC(0x02);
	  int sec =((sec_addr&0b01111111)>>4)*10+(sec_addr&0b1111);
	  int min =((min_addr&0b01111111)>>4)*10+(min_addr&0b1111);
	  int hour=((hour_addr&0b0111111)>>4)*10+(hour_addr&0b1111);

	  //if(read_APDS9960(0xFC)==255)setRTC(0,49,23,1,29,11,2020);
	  sprintf(s,"%2d:%2d:%2d",hour,min,sec);
	  print(s,0);
	  char week[4]="  \0";
	  switch(read_RTC(0x03)){
	  case 1:
		  week[0]='S';
		  week[1]='u';
		  week[2]='n';
		  break;
	  case 2:
		  week[0]='M';
		  week[1]='o';
		  week[2]='n';
		  break;
	  case 3:
		  week[0]='T';
		  week[1]='u';
		  week[2]='e';
		  break;
	  case 4:
		  week[0]='W';
		  week[1]='e';
		  week[2]='d';
		  break;
	  case 5:
		  week[0]='T';
		  week[1]='h';
		  week[2]='u';
		  break;
	  case 6:
		  week[0]='F';
		  week[1]='r';
		  week[2]='i';
		  break;
	  case 7:
		  week[0]='S';
		  week[1]='a';
		  week[2]='t';
		  break;
	  default:
		  week[0]=' ';
		  week[1]=' ';
		  week[2]=' ';

	  }

	  int date_addr=read_RTC(0x04);
	  int date=(date_addr>>4)*10+(date_addr&0b1111);
	  int month_addr=read_RTC(0x05);
	  int month=(month_addr>>4)*10+(month_addr&0b1111);
	  int year_addr=read_RTC(0x06);
	  int year=(year_addr>>4)*10+(year_addr&0b1111)+2000;

	  //sprintf(s,"%d|%3d,%3d,%3d,%3d",SERIAL_STRING,read_APDS9960(0xFC),read_APDS9960(0xFD),read_APDS9960(0xFE),read_APDS9960(0xFF) );

	  sprintf(s,"%4d/%2d/%2d(%s)",year,month,date,week);
	  print(s,1);
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
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSIDiv = RCC_HSI_DIV1;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C2_Init(void)
{

  /* USER CODE BEGIN I2C2_Init 0 */

  /* USER CODE END I2C2_Init 0 */

  /* USER CODE BEGIN I2C2_Init 1 */

  /* USER CODE END I2C2_Init 1 */
  hi2c2.Instance = I2C2;
  hi2c2.Init.Timing = 0x00303D5B;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Analogue filter 
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c2, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Digital filter 
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c2, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C2_Init 2 */

  /* USER CODE END I2C2_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, GPIO_PIN_RESET);

  /*Configure GPIO pin : PB9 */
  GPIO_InitStruct.Pin = GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

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
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
