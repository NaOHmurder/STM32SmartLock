
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  ** This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * COPYRIGHT(c) 2019 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f1xx_hal.h"

/* USER CODE BEGIN Includes */
#include "keyboard.h"
#include "OLED.h"
#include "string.h"
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim2;

UART_HandleTypeDef huart4;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM2_Init(void);
static void MX_UART4_Init(void);
                                    
void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);
                                

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

void key_interrupt_fun(uint16_t);
int main_menu_flash(void);
int password_flash(void);
int tips_flash(void);
int locked_flash(void);
int time(void);
int str2int();
void beep(int);
void unlock(void);
void locked(void);
void resetpass(void);
void timechanging(void);

#define MAIN_MENU 0
#define PASS_MENU 1
#define TIPS_MENU 2
#define LOCKED_MENU 3

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */
int menu_flag=MAIN_MENU;
int year=19,month=7,day=14;
int hour=10,min=26,sec=0;
int yeartemp=0,monthtemp=0,daytemp=0;
int hourtemp=0,mintemp=0,sectemp=0;
char tipsbuf[16];
char starbuf[16];
char pswdbuf[16];
char pswdprintbuf[16];
char configbuf[16];
char timeconfig[16];
char timetempbuf[6];
char rxbuf[100];
char rxprintbuf[100];
char orginpswd[16]="123456";
char orginpswdbuf[16];
char uart_rxbuf[1];
char judgebuf[16];
char *ptchar;
int pos=0;

/*void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	sec++;
	if(sec==60){
		sec=0;min++;
		if(min==60){
			//......
		}
	}
}**/

void HAL_UART_RxCpltCallback (UART_HandleTypeDef *huart)
{
	rxbuf[pos]=uart_rxbuf[0];
	pos++;
	if(uart_rxbuf[0]==';'){
		HAL_UART_Transmit(&huart4,rxbuf,strlen(rxbuf),100);
		pos=0;
		strcpy(rxprintbuf,rxbuf);
		if(strncmp("config:unlock",rxbuf,13)==0){
			unlock();
		}else if(strncmp("config:resetpass",rxbuf,16)==0){
			resetpass();
		}else if(strncmp("config:locked",rxbuf,13)==0){
			locked();
		}else if(strncmp("config:timechanging",rxbuf,19)==0){
			timechanging();
		}
		memset(rxbuf,0,sizeof(rxbuf));
	}
	HAL_UART_Receive_IT(&huart4,uart_rxbuf,1);
}

void HAL_GPIO_EXTI_Callback (uint16_t GPIO_PIN)
{
	/*按键消抖*/
	static int pre_ticks=0;
	int ticks;
	ticks=HAL_GetTick();
	if(ticks-pre_ticks<300){
		return;
	}
	pre_ticks=ticks;
	/*读取按键中断*/
	char keynum;
	keynum=Get_KeyNum(GPIO_PIN);
	if(keynum>='0'&&keynum<='9'){
		OLED_CLS();
		menu_flag=PASS_MENU;
		starbuf[pos]='*';
		pswdbuf[pos]=keynum;
		pos++;
		//beep(40);
	}
	
	if(keynum=='C'){
		OLED_CLS();
		menu_flag=PASS_MENU;
		for(int i=0;i<=pos;i++){
		
			starbuf[i]=NULL;
			pswdbuf[i]=NULL;
		}
		pos=0;
	}
	
	if(keynum=='D'){
		OLED_CLS();
		menu_flag=PASS_MENU;
		pos--;
		starbuf[pos]=NULL;
		pswdbuf[pos]=NULL;
	}
	
	if(keynum=='E'){
		OLED_CLS();
		int ret;
		ret=strncmp(orginpswd,pswdbuf,6);
		if(ret==0){
			HAL_GPIO_WritePin(GPIOB,GPIO_PIN_12,GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOC,GPIO_PIN_7,GPIO_PIN_RESET);
			//HAL_Delay(1000);
			//HAL_GPIO_WritePin(GPIOB,GPIO_PIN_12,GPIO_PIN_RESET);
			//HAL_GPIO_WritePin(GPIOC,GPIO_PIN_7,GPIO_PIN_SET);
			menu_flag=TIPS_MENU;
			strcpy(tipsbuf,"unlock success");
			memset(starbuf,0,sizeof(starbuf));
			memset(pswdbuf,0,sizeof(pswdbuf));
			pos=0;
		}else{
			HAL_GPIO_WritePin(GPIOB,GPIO_PIN_12,GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOC,GPIO_PIN_6,GPIO_PIN_RESET);
			//HAL_Delay(1000);
			//HAL_GPIO_WritePin(GPIOC,GPIO_PIN_8,GPIO_PIN_SET);
			menu_flag=TIPS_MENU;
			strcpy(tipsbuf,"unlock fail");
			memset(starbuf,0,sizeof(starbuf));
			memset(pswdbuf,0,sizeof(pswdbuf));
			pos=0;
		}
	}
	
	if(keynum=='Q'){
		OLED_CLS();
		menu_flag=MAIN_MENU;
		memset(starbuf,0,sizeof(starbuf));
		memset(pswdbuf,0,sizeof(pswdbuf));
		pos=0;
	}
}

void unlock(void)
{
	//config:unlock,keypass="password"
	ptchar=strstr(rxprintbuf,"keypass=");
	int i=0;
	ptchar=ptchar+strlen("keypass="); 
	memset(pswdbuf,0,sizeof(pswdbuf));
	while(*ptchar!=','&& *ptchar!=';'){
		pswdbuf[i]=*ptchar;
		ptchar++;i++;
	}
	
	strcpy(pswdprintbuf,pswdbuf);
	int ret;
	ret=strncmp(orginpswd,pswdbuf,6);
	if(ret==0){
		HAL_GPIO_WritePin(GPIOB,GPIO_PIN_12,GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOC,GPIO_PIN_7,GPIO_PIN_RESET);
		//HAL_Delay(1000);
		//HAL_GPIO_WritePin(GPIOB,GPIO_PIN_12,GPIO_PIN_RESET);
		//HAL_GPIO_WritePin(GPIOC,GPIO_PIN_7,GPIO_PIN_SET);
		menu_flag=TIPS_MENU;
		strcpy(tipsbuf,"unlock success");
		memset(pswdbuf,0,sizeof(pswdbuf));
		pos=0;
	}else{
		HAL_GPIO_WritePin(GPIOB,GPIO_PIN_12,GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOC,GPIO_PIN_6,GPIO_PIN_RESET);
		//HAL_Delay(1000);
		//HAL_GPIO_WritePin(GPIOC,GPIO_PIN_8,GPIO_PIN_SET);
		menu_flag=TIPS_MENU;
		strcpy(tipsbuf,"unlock fail");
		memset(pswdbuf,0,sizeof(pswdbuf));
		pos=0;
	}
}

void resetpass(void)
{
	//config:resetpass=14564,keypass=123456
	int i=0;
	int ret;
	ptchar=strstr(rxprintbuf,"keypass=");
	ptchar=ptchar+strlen("keypass=");
	while(*ptchar !=','&&*ptchar !=';'){
		pswdbuf[i]=*ptchar;
		ptchar++; i++;
	}
	
	ret=strncmp(orginpswd,pswdbuf,6);
	if(ret==0){
		i=0;
		ptchar=strstr(rxprintbuf,"resetpass=");
		ptchar=ptchar+strlen("resetpass=");
		while(*ptchar !=','&&*ptchar !=';'){
		
			orginpswdbuf[i]=*ptchar;
			ptchar++;i++;
		}
		/*for(int k=0;k<=strlen(orginpswd);k++){
			orginpswd[k]=orginpswdbuf[k];
		}*/
		strcpy(orginpswd,orginpswdbuf);
		strcpy(tipsbuf," reset success ");
		menu_flag=TIPS_MENU;
		HAL_GPIO_WritePin(GPIOC,GPIO_PIN_7,GPIO_PIN_RESET);
		memset(orginpswdbuf,0,sizeof(orginpswdbuf));
	}else{
		strcpy(tipsbuf," wrong oldpass ");
		menu_flag=TIPS_MENU;
		HAL_GPIO_WritePin(GPIOC,GPIO_PIN_6,GPIO_PIN_RESET);
	}
}

void locked(void)
{
	ptchar=strstr(rxprintbuf,"locked");
	int i=0;
	//memset(pswdbuf,0,sizeof(pswdbuf));
	while(*ptchar!=';'){
		configbuf[i]=*ptchar;
		ptchar++;
		i++;
	}
	if(strcmp(configbuf,"locked")==0){
		HAL_GPIO_WritePin(GPIOB,GPIO_PIN_12,GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOC,GPIO_PIN_6,GPIO_PIN_RESET);
		menu_flag=LOCKED_MENU;
	}
}

void timechanging(void)
{
	int i=0;
	ptchar=strstr(rxprintbuf,"set:");
	ptchar=ptchar+strlen("set:");
	while(*ptchar!=','&& *ptchar!=';'){
		timeconfig[i]=*ptchar;
		ptchar++;
		i++;
	}
	if(strncmp(timeconfig,"sec",3)==0){
		i=0;
		ptchar=strstr(rxprintbuf,"set:");
		ptchar=ptchar+strlen("set:sec=");
		while(*ptchar!=','&&*ptchar!=';')
		{
			timetempbuf[i]=*ptchar;
			ptchar++;
			i++;
		}
		sectemp=str2int(timetempbuf);
		if(sectemp>=60){
			strcpy(tipsbuf,"setsec fail");
			menu_flag=TIPS_MENU;
		}else{
			sec=sectemp;
			strcpy(tipsbuf,"setsec success");
			menu_flag=TIPS_MENU;
		}
		
		memset(timetempbuf,0,sizeof(timetempbuf));
		
	}else if(strncmp(timeconfig,"min",3)==0){
		i=0;
		ptchar=strstr(rxprintbuf,"set:");
		ptchar=ptchar+strlen("set:min=");
		while(*ptchar!=','&&*ptchar!=';')
		{
			timetempbuf[i]=*ptchar;
			ptchar++;
			i++;
		}
		mintemp=str2int(timetempbuf);
		if(mintemp>=60){
			strcpy(tipsbuf,"setmin fail");
			menu_flag=TIPS_MENU;
		}else{
			min=mintemp;
			strcpy(tipsbuf,"setmin success");
		menu_flag=TIPS_MENU;
		}
		
		memset(timetempbuf,0,sizeof(timetempbuf));
	}else if(strncmp(timeconfig,"hour",4)==0){
		i=0;
		ptchar=strstr(rxprintbuf,"set:");
		ptchar=ptchar+strlen("set:hour=");
		while(*ptchar!=','&&*ptchar!=';')
		{
			timetempbuf[i]=*ptchar;
			ptchar++;
			i++;
		}
		hourtemp=str2int(timetempbuf);
		if(hourtemp>=24){
			strcpy(tipsbuf,"sethour fail");
			menu_flag=TIPS_MENU;
		}else{
			hour=hourtemp;
			strcpy(tipsbuf,"sethour success");
			menu_flag=TIPS_MENU;
		}
		
		memset(timetempbuf,0,sizeof(timetempbuf));
	}else if(strncmp(timeconfig,"day",3)==0){
		i=0;
		ptchar=strstr(rxprintbuf,"set:");
		ptchar=ptchar+strlen("set:day=");
		while(*ptchar!=','&&*ptchar!=';')
		{
			timetempbuf[i]=*ptchar;
			ptchar++;
			i++;
		}
		daytemp=str2int(timetempbuf);
		if(daytemp>31){
			strcpy(tipsbuf,"setday fail");
			menu_flag=TIPS_MENU;
		}else{
			day=daytemp;
			strcpy(tipsbuf,"setday success");
			menu_flag=TIPS_MENU;
		}
		memset(timetempbuf,0,sizeof(timetempbuf));
	}
}

int str2int(char *str)
{
	int result=0;
	for(int i=0;str[i]!='\0';i++)
	{
		result=result*10+str[i]-'0';
	}
	return result;
}

void beep(int period)
{
	int cmp=period/2;
	HAL_TIM_PWM_Start(&htim2,TIM_CHANNEL_1);
	cmp=period/2;
	__HAL_TIM_SET_COMPARE(&htim2,TIM_CHANNEL_1,cmp);
	__HAL_TIM_SET_AUTORELOAD(&htim2,period);
	period+=10;
	HAL_Delay(200);
}

int time(void)
{
	HAL_Delay(1000);
	sec++;
	if(sec==60){
		sec=0;min++;
		if(min==60){
			//......
		}
	}
}

int main_menu_flash(void)
{
	/*HAL_Delay(1000);
	sec++;
	if(sec==60){
		sec=0;min++;
		if(min==60){
			//......
		}
	}*/
	char datebuf[16];
	char timebuf[16];
	sprintf(datebuf,"    %02d-%02d-%02d   ",year,month,day);
	sprintf(timebuf,"    %02d:%02d:%02d   ",hour,min,sec);
	OLED_ShowStr(0,0,datebuf,CODE8X16);
	OLED_ShowStr(0,16,timebuf,CODE8X16);
	OLED_ShowStr(0,32," SMARTLOCKMARKI ",CODE8X16);
	OLED_ShowStr(0,64,"WIFI            ",CODE8X16);
  fflash();
}

int password_flash(void)
{
	//OLED_CLS();
	OLED_ShowStr(0,16,"  Enter passwd  ",CODE8X16);
	OLED_ShowStr(0,32,starbuf,CODE8X16);
	fflash();
}

int locked_flash(void)
{
	OLED_CLS();
	OLED_ShowStr(0,16,"LOCKED",CODE8X16);
	HAL_GPIO_WritePin(GPIOC,GPIO_PIN_6,GPIO_PIN_SET);
	fflash();
}

int tips_flash(void)
{
	OLED_CLS();
	//OLED_ShowStr(0,0,rxprintbuf,CODE8X16);
	OLED_ShowStr(0,16,tipsbuf,CODE8X16);
	//OLED_ShowStr(0,32,pswdprintbuf,CODE8X16);
	fflash();
	if(strncmp("unlock success",tipsbuf,14)==0){
		//HAL_Delay(1000*2);
		HAL_GPIO_WritePin(GPIOB,GPIO_PIN_12,GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOC,GPIO_PIN_7,GPIO_PIN_SET);
	}else if(strncmp("unlock fail",tipsbuf,11)==0){
		//HAL_Delay(1000);
		HAL_GPIO_WritePin(GPIOC,GPIO_PIN_6,GPIO_PIN_SET);	
	}else if(strncmp(" reset success ",tipsbuf,15)==0){
		HAL_GPIO_WritePin(GPIOC,GPIO_PIN_7,GPIO_PIN_SET);
	}else if(strncmp(" wrong oldpass ",tipsbuf,15)==0){
		HAL_GPIO_WritePin(GPIOC,GPIO_PIN_6,GPIO_PIN_SET);
	}
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  *
  * @retval None
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

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
  MX_TIM2_Init();
  MX_UART4_Init();
  /* USER CODE BEGIN 2 */
	Key_Bord_Init();
	OLED_Init();
	HAL_GPIO_WritePin(GPIOB,GPIO_PIN_12,GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOC,GPIO_PIN_6,GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOC,GPIO_PIN_7,GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOC,GPIO_PIN_8,GPIO_PIN_SET);
	HAL_UART_Receive_IT(&huart4,uart_rxbuf,1);
	/*
	HAL_TIM_Base_Start_IT(&htim1);
	HAL_Delay(1000);
	HAL_TIM_Base_Stop_IT(&htim1);*/
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

		switch(menu_flag){
		
			case MAIN_MENU:
				main_menu_flash();
				time();
			break;
			case PASS_MENU:
				password_flash();
				time();
			break;
			case TIPS_MENU:
				tips_flash();
				time();
				HAL_Delay(1000);
				menu_flag=MAIN_MENU;
			break;
			case LOCKED_MENU:
				locked_flash();
				time();
				HAL_Delay(1000);
				menu_flag=MAIN_MENU;
			break;
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

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = 16;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI_DIV2;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL16;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure the Systick interrupt time 
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick 
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* I2C1 init function */
static void MX_I2C1_Init(void)
{

  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* TIM2 init function */
static void MX_TIM2_Init(void)
{

  TIM_ClockConfigTypeDef sClockSourceConfig;
  TIM_MasterConfigTypeDef sMasterConfig;
  TIM_OC_InitTypeDef sConfigOC;

  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 6400;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 0;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  HAL_TIM_MspPostInit(&htim2);

}

/* UART4 init function */
static void MX_UART4_Init(void)
{

  huart4.Instance = UART4;
  huart4.Init.BaudRate = 115200;
  huart4.Init.WordLength = UART_WORDLENGTH_8B;
  huart4.Init.StopBits = UART_STOPBITS_1;
  huart4.Init.Parity = UART_PARITY_NONE;
  huart4.Init.Mode = UART_MODE_TX_RX;
  huart4.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart4.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart4) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/** Configure pins as 
        * Analog 
        * Input 
        * Output
        * EVENT_OUT
        * EXTI
*/
static void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6|GPIO_PIN_7|GPIO_PIN_8, GPIO_PIN_RESET);

  /*Configure GPIO pins : PA0 PA1 PA2 PA3 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PA4 PA5 PA6 PA7 */
  GPIO_InitStruct.Pin = GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PB12 */
  GPIO_InitStruct.Pin = GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PC6 PC7 PC8 */
  GPIO_InitStruct.Pin = GPIO_PIN_6|GPIO_PIN_7|GPIO_PIN_8;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI4_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI4_IRQn);

  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  file: The file name as string.
  * @param  line: The line in file as a number.
  * @retval None
  */
void _Error_Handler(char *file, int line)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  while(1)
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
void assert_failed(uint8_t* file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
