#include "keyboard.h"
#include "stm32f1xx_hal.h"



/*		
	PA0-3	作为输出模式,输出低电平
	PA4-7  作为中断模式,触发中断		中断优先级在cubmux配置
*/
static void key_out_and_EXTI_mode(void)
{

  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3, GPIO_PIN_RESET);

  /*Configure GPIO pins : PA0 PA1 PA2 PA3 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PA4 PA5 PA6 PA7 */
  GPIO_InitStruct.Pin = GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

#if   0
  HAL_NVIC_SetPriority(EXTI4_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI4_IRQn);

  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);
#endif
}


static void Key_Input_Mode(void)// A0-3output low level,   4-7input
{

  GPIO_InitTypeDef GPIO_InitStruct;

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3, GPIO_PIN_RESET);

  /*Configure GPIO pins : PA0 PA1 PA2 PA3 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PA4 PA5 PA6 PA7 */
  GPIO_InitStruct.Pin = GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
}

static void Key_OutPut_Mode(void)// A0-3input   4-7output lowlevel
{

  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7, GPIO_PIN_RESET);

  /*Configure GPIO pins : PA0 PA1 PA2 PA3 */
  GPIO_InitStruct.Pin = GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PA4 PA5 PA6 PA7 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
}




static uint8_t Get_Key(uint8_t num)
{
	uint8_t key=0;
	switch(num)
	{
		case 0x11:key='A';break;
		case 0x21:key='B';break;
		case 0x41:key='C';break;
		case 0x12:key='3';break;
		case 0x22:key='6';break;
		case 0x42:key='9';break;
		case 0x14:key='2';break;
		case 0x24:key='5';break;
		case 0x44:key='8';break;
		case 0x28:key='4';break;
		case 0x81:key='D';break;
		case 0x82:key='E';break;
		case 0x84:key='0';break;
		case 0x88:key='Q';break;
		case 0x18:key='1';break;
		case 0x48:key='7';break;		

	}
	return key;
}

uint8_t Get_KeyNum(uint16_t GPIO_Pin)
{
	uint8_t i=0,num=0;
	uint8_t key = 0;
	
	Key_Input_Mode();				// A0-3output low level,   4-7input
	for(i=4;i<8;i++)
	{
		if(HAL_GPIO_ReadPin(GPIOA,(1<<i)) == GPIO_PIN_RESET)
			num |= (1<<i);
	}

	
	if(num)
	{
		Key_OutPut_Mode();		// A0-3 input ,   4-7 output low level
		for(i=0;i<4;i++)
		{
			if(HAL_GPIO_ReadPin(GPIOA,(1<<i)) == GPIO_PIN_RESET)
				num |= (1<<i);
		}

		key = Get_Key(num);
	}
	
	key_out_and_EXTI_mode();
	
	return  key; 
}


void Key_Bord_Init(void)
{
	key_out_and_EXTI_mode();	
}


