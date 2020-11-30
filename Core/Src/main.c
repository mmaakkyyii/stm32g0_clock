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
enum State{
	clock,
	setting
};

enum State state;

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
	if(col==2)  writedata(0x00,0x80+17);

	for(;*p;p++){
		writedata(0x40,*p);
	}

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

void initRTC(){
	write_RTC(0x00,read_RTC(0x00)&0b01111111);//clear CH bit
}
void set_week(char* week,int day){
	week[3]='\0';
	switch(day){
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

}
int get_sec(){
	int sec_addr=read_RTC(0x00);
	return ((sec_addr&0b01111111)>>4)*10+(sec_addr&0b1111);
}
int get_min(){
	int min_addr=read_RTC(0x01);
	return ((min_addr&0b01111111)>>4)*10+(min_addr&0b1111);
}
int get_hour(){
	int hour_addr=read_RTC(0x02);
	return ((hour_addr&0b0111111)>>4)*10+(hour_addr&0b1111);
}
int get_day(){
	return read_RTC(0x03);
}
int get_date(){
	int date_addr=read_RTC(0x04);
	return (date_addr>>4)*10+(date_addr&0b1111);
}
int get_month(){
	int month_addr=read_RTC(0x05);
	return (month_addr>>4)*10+(month_addr&0b1111);
}
int get_year(){
	int year_addr=read_RTC(0x06);
	return (year_addr>>4)*10+(year_addr&0b1111)+2000;
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

enum Dir{
	non,
	left,
	right,
	up,
	down,
	near,
	far
};

void initAPDS9960(){
	//uint8_t whoami=read_APDS9960(0x92);

	write_APDS9960(0x80,0b01000101);//POWER ON<0>, GESTURE ENABLE<6>, PROXIMITY DETECT ENALBE<2>,AEN=0
	write_APDS9960(0x90,0b00110000);//Gesture LED Drive Strength 300%(max)
	write_APDS9960(0xA3,0b01100100);//Reserve0, Gain x8(11), LED Drive 100mA(00), Wait Time see under number
									  //111=39.2mS 110=30.8mS 101=22.4mS 100=14.0mS 011=8.4mS 010=5.6mS 001=2.8ms 000=0mS

	write_APDS9960(0xA4,(char)(90));//U MINUS OFFSET
	write_APDS9960(0xA5,(char)(80)); //D MINUS OFFSET
	write_APDS9960(0xA7,0);//L MINUS OFFSET
	write_APDS9960(0xA9,0);//R MINUS OFFSET

	write_APDS9960(0xAB,0b00000001);//GIEN off<1>(INTERRUPT DISABLE), GMODE ON<0>
}

int gesture(){
	const int gesture_thr=50;
	const int GESTURE_SENSITIVITY_1=50;
	const int GESTURE_SENSITIVITY_2=50;

	char gstatus=read_APDS9960(0xAF);//GSTATUS
	if((gstatus&0x01) !=1)return -1;

	char fifo_level = read_APDS9960(0xAE);  //READ GESTUR FIFO LEVEL REGISTER
	char DATA_U[32];
	char DATA_D[32];
	char DATA_L[32];
	char DATA_R[32];
	char u_first=1;
	char d_first=1;
	char l_first=1;
	char r_first=1;
	char u_last=1;
	char d_last=1;
	char l_last=1;
	char r_last=1;

	int ud_ratio_first;
	int lr_ratio_first;
	int ud_ratio_last;
	int lr_ratio_last;
	int ud_delta;
	int lr_delta;

    int gesture_ud_delta_ = 0;
    int gesture_lr_delta_ = 0;
    int gesture_ud_count_ = 0;
    int gesture_lr_count_ = 0;
    int gesture_near_count_ = 0;
    int gesture_far_count_ = 0;
    int gesture_state = 0;


	if(fifo_level > 0)           //IF FIFO HAS SOME DATA
	{
		for(int i=0;i<fifo_level;i++){
			DATA_U[i] = read_APDS9960(0xFC);
			DATA_D[i] = read_APDS9960(0xFD);
			DATA_L[i] = read_APDS9960(0xFE);
			DATA_R[i] = read_APDS9960(0xFF);
		}

		for(int i=0;i<fifo_level;i++){
			if( (DATA_U[i]>gesture_thr) &&
				(DATA_U[i]>gesture_thr) ||
				(DATA_U[i]>gesture_thr) &&
				(DATA_U[i]>gesture_thr)	){

				u_first=DATA_U[i];
				d_first=DATA_D[i];
				l_first=DATA_L[i];
				r_first=DATA_R[i];
				break;
			}
		}
		//char s[16];
		//sprintf(s,"%d|%3d:%3d:%3d:%3d",gesture_state,DATA_U[0],DATA_D[0],DATA_L[0],DATA_R[0]);
		//print(s,2);

		for(int i=fifo_level;i>=0;i--){
			if( (DATA_U[i]>gesture_thr) &&
				(DATA_U[i]>gesture_thr) ||
				(DATA_U[i]>gesture_thr) &&
				(DATA_U[i]>gesture_thr)	){

				u_last=DATA_U[i];
				d_last=DATA_D[i];
				l_last=DATA_L[i];
				r_last=DATA_R[i];
				break;
			}
		}
	}
	/* Calculate the first vs. last ratio of up/down and left/right */
	ud_ratio_first = ((u_first - d_first) * 100) / (u_first + d_first);
	lr_ratio_first = ((l_first - r_first) * 100) / (l_first + r_first);
	ud_ratio_last = ((u_last - d_last) * 100) / (u_last + d_last);
	lr_ratio_last = ((l_last - r_last) * 100) / (l_last + r_last);

	/* Determine the difference between the first and last ratios */
	ud_delta = ud_ratio_last - ud_ratio_first;
	lr_delta = lr_ratio_last - lr_ratio_first;

	/* Accumulate the UD and LR delta values */
    gesture_ud_delta_ += ud_delta;
    gesture_lr_delta_ += lr_delta;

    /* Determine U/D gesture */
    if( gesture_ud_delta_ >= GESTURE_SENSITIVITY_1 ) {
        gesture_ud_count_ = 1;
    } else if( gesture_ud_delta_ <= -GESTURE_SENSITIVITY_1 ) {
        gesture_ud_count_ = -1;
    } else {
        gesture_ud_count_ = 0;
    }

    /* Determine L/R gesture */
    if( gesture_lr_delta_ >= GESTURE_SENSITIVITY_1 ) {
        gesture_lr_count_ = 1;
    } else if( gesture_lr_delta_ <= -GESTURE_SENSITIVITY_1 ) {
        gesture_lr_count_ = -1;
    } else {
        gesture_lr_count_ = 0;
    }

    /* Determine Near/Far gesture */
    if( (gesture_ud_count_ == 0) && (gesture_lr_count_ == 0) ) {
        if( (abs(ud_delta) < GESTURE_SENSITIVITY_2) && \
            (abs(lr_delta) < GESTURE_SENSITIVITY_2) ) {

            if( (ud_delta == 0) && (lr_delta == 0) ) {
                gesture_near_count_++;
            } else if( (ud_delta != 0) || (lr_delta != 0) ) {
                gesture_far_count_++;
            }

            if( (gesture_near_count_ >= 10) && (gesture_far_count_ >= 2) ) {
                if( (ud_delta == 0) && (lr_delta == 0) ) {
                    return near;
                } else if( (ud_delta != 0) && (lr_delta != 0) ) {
                    return far;
                }
            }
        }
    } else {
        if( (abs(ud_delta) < GESTURE_SENSITIVITY_2) && \
            (abs(lr_delta) < GESTURE_SENSITIVITY_2) ) {

            if( (ud_delta == 0) && (lr_delta == 0) ) {
                gesture_near_count_++;
            }

            if( gesture_near_count_ >= 10 ) {
                gesture_ud_count_ = 0;
                gesture_lr_count_ = 0;
                gesture_ud_delta_ = 0;
                gesture_lr_delta_ = 0;
            }
        }
    }


    /* Determine swipe direction */
    if( (gesture_ud_count_ == -1) && (gesture_lr_count_ == 0) ) {
        gesture_state = up;
    } else if( (gesture_ud_count_ == 1) && (gesture_lr_count_ == 0) ) {
        gesture_state = down;
    } else if( (gesture_ud_count_ == 0) && (gesture_lr_count_ == 1) ) {
        gesture_state = right;
    } else if( (gesture_ud_count_ == 0) && (gesture_lr_count_ == -1) ) {
        gesture_state = left;
    } else if( (gesture_ud_count_ == -1) && (gesture_lr_count_ == 1) ) {
        if( abs(gesture_ud_delta_) > abs(gesture_lr_delta_) ) {
            gesture_state = up;
        } else {
            gesture_state = right;
        }
    } else if( (gesture_ud_count_ == 1) && (gesture_lr_count_ == -1) ) {
        if( abs(gesture_ud_delta_) > abs(gesture_lr_delta_) ) {
            gesture_state = down;
        } else {
            gesture_state = left;
        }
    } else if( (gesture_ud_count_ == -1) && (gesture_lr_count_ == -1) ) {
        if( abs(gesture_ud_delta_) > abs(gesture_lr_delta_) ) {
            gesture_state = up;
        } else {
            gesture_state = left;
        }
    } else if( (gesture_ud_count_ == 1) && (gesture_lr_count_ == 1) ) {
        if( abs(gesture_ud_delta_) > abs(gesture_lr_delta_) ) {
            gesture_state = down;
        } else {
            gesture_state = right;
        }
    } else {
        return -1;
    }

    return gesture_state;
}

void Clock(){
	HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_9);
	HAL_Delay(1);
	uint8_t s[16]={};

	if(gesture()==up)state=setting;
	HAL_Delay(100);
	//if(read_APDS9960(0xFC)==255)setRTC(0,49,23,1,29,11,2020);
	sprintf(s,"%2d:%2d:%2d",get_hour(),get_min(),get_sec());
	print(s,0);

	char week[4]="  \0";
	set_week(week,get_day());
	sprintf(s,"%4d/%2d/%2d(%s)",get_year(),get_month(),get_date(),week);
	print(s,1);

}
void Setting(){
	writedata(0x00,0b00001111);//cursor on

	int setting_sec=get_sec();
	int setting_min=get_min();
	int setting_hour=get_hour();
	int setting_day=get_day();
	int setting_date=get_date();
	int setting_month=get_month();
	int setting_year=get_year();
	setting_year-=2000;

	char s[16];
	sprintf(s,"%2d:%2d:%2d Set 0 *",setting_hour,setting_min,setting_sec);
	print(s,0);
	char week[4]="  \0";
	set_week(week,setting_day);
	sprintf(s,"%4d/%2d/%2d(%s)*",2000+setting_year,setting_month,setting_date,week);
	print(s,1);
	HAL_Delay(500);


	int setting_flag=1;
	int cursor=0;
	int colum=0;
	while(setting_flag){
		int input=gesture();
		if(input>non){
			if(input==left){
				if(colum==0){
					if(cursor==3||cursor==6){
						cursor-=2;
					}else if(cursor<=0){
						cursor=15;
						colum=1;
					}else{
						cursor--;
					}
				}else if(colum==1){
					if(cursor==5||cursor==8||cursor==11){
						cursor-=2;
					}else if(cursor==15){
						cursor=11;
					}else if(cursor<=2){
						cursor=6;
						colum=0;
					}else{
						cursor--;
					}
				}
			}else if(input==right){
				if(colum==0){
					if(cursor==1||cursor==4){
						cursor+=2;
					}else if(cursor>=7){
						cursor=2;
						colum=1;
					}else{
						cursor++;
					}
				}else if(colum==1){
					if(cursor==3||cursor==6||cursor==9){
						cursor+=2;
					}else if(cursor==11){
						cursor=15;
					}else if(cursor>=15){
						colum=0;
						cursor=0;
					}else{
						cursor++;
					}
				}
			}else if(input==up){
				switch(colum){
				case 0:
					switch(cursor){
					case 0:
						if(setting_hour<20)setting_hour+=10;
						else setting_hour=setting_hour%10;
						break;
					case 1:
						if(setting_hour<23)setting_hour+=1;
						else setting_hour=0;
						break;

					case 3:
						if(setting_min<50)setting_min+=10;
						else setting_min=setting_min%10;
						break;
					case 4:
						if(setting_min<59)setting_min+=1;
						else setting_min=0;
						break;

					case 6:
						if(setting_sec<50)setting_sec+=10;
						else setting_sec=setting_sec%10;
						break;
					case 7:
						if(setting_sec<59)setting_sec+=1;
						else setting_sec=0;
						break;
					default:
						break;

					}
					break;

				case 1:
					switch(cursor){
					//year
					case 2:
						if(setting_year<=90)setting_year+=10;
						else setting_year=setting_year%10;
						break;
					case 3:
						if(setting_year<99)setting_year+=1;
						else setting_year=0;
						break;

					//month
					case 5:
						if(setting_month<10)setting_month+=10;
						else setting_month=setting_month%10;
						break;
					case 6:
						if(setting_month<12)setting_month+=1;
						else setting_month=1;
						break;

					//date TODO
					case 8:
						if(setting_date<30)setting_date+=10;
						else setting_date=setting_date%10;
						break;
					case 9:
						if(setting_date<31)setting_date+=1;
							else setting_date=1;
							break;
					//day
					case 11:
						if(setting_day<7)setting_day+=1;
							else setting_day=1;
							break;
					//enter
					case 15:
						setRTC(setting_sec, setting_min, setting_hour, setting_day, setting_date, setting_month, 2000+setting_year);
						print("setting         ",0);
						print("clock           ",1);
						for(int i=0;i<500;i++){
							gesture();
							HAL_Delay(1);
						}
						state=clock;
						return;
						break;
					default:
						break;
					}
					break;

				}
			}else if(input==down){
				switch(colum){
				case 0:
					switch(cursor){
					case 0:
						if(setting_hour>10)setting_hour-=10;
						else setting_hour=setting_hour+20;//
						break;
					case 1:
						if(setting_hour>0)setting_hour-=1;
						else setting_hour=24;
						break;

					case 3:
						if(setting_min>10)setting_min-=10;
						else setting_min=setting_min+50;
						break;
					case 4:
						if(setting_min>0)setting_min-=1;
						else setting_min=59;
						break;

					case 6:
						if(setting_sec>10)setting_sec-=10;
						else setting_sec=setting_sec+50;
						break;
					case 7:
						if(setting_sec>0)setting_sec-=1;
						else setting_sec=59;
						break;
					default:
						break;

					}
					break;

				case 1:
					switch(cursor){
					//year
					case 2:
						if(setting_year>=10)setting_year-=10;
						else setting_year=setting_year+90;
						break;
					case 3:
						if(setting_year>0)setting_year-=1;
						else setting_year=99;
						break;

					//month
					case 5:
						if(setting_month>10)setting_month-=10;
						else setting_month=setting_month+10;//
						break;
					case 6:
						if(setting_month>1)setting_month-=1;
						else setting_month=12;
						break;

					//date TODO
					case 8:
						if(setting_date>10)setting_date-=10;
						else setting_date=setting_date+20;
						break;
					case 9:
						if(setting_date>1)setting_date-=1;
							else setting_date=30;
							break;
					//day
					case 11:
						if(setting_day>1)setting_day-=1;
							else setting_day=7;
							break;
					//enter
					case 15:
						print("setting         ",0);
						print("cancel          ",1);
						for(int i=0;i<500;i++){
							gesture();
							HAL_Delay(1);
						}
						state=clock;
						return;

					default:
						break;
					}
					break;
				}
			}

			char s[16];
			sprintf(s,"%2d:%2d:%2d Set %d",setting_hour,setting_min,setting_sec,input);
			print(s,0);
			char week[4]="  \0";
			set_week(week,setting_day);
			sprintf(s,"%4d/%2d/%2d(%s)*",2000+setting_year,setting_month,setting_date,week);
			print(s,1);

		}
		if(cursor>=15)cursor=15;
		if(cursor<=0)cursor=0;
		writedata(0x00,0x80+0x40*colum+cursor);//set curso position
		HAL_Delay(500);

	}

	writedata(0x00,0b00001100);//cursor off

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

  initdisplay();
  print("STM32G0 clock   ",0);
  print("   by mmaakkyyii",1);

  HAL_Delay(500);

  print("                ",0);
  print("                ",1);

  initAPDS9960();
  initRTC();
  state=clock;

/* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  switch(state){
	  case clock:
	  	  Clock();
	  	  break;
	  case setting:
	  	  Setting();
	  	  writedata(0x00,0b00001100);//cursor off
	  	  break;
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
