#include "codetab.h"
#include "OLED.h"

unsigned char REG_FLASH[8][128];//oled flash
extern I2C_HandleTypeDef hi2c1;

void I2C_WriteByte(uint8_t REG_Address,uint8_t REG_data)
{
    uint8_t TxData[2] = {REG_Address,REG_data};
		HAL_I2C_Master_Transmit(&hi2c1,0X78,(uint8_t*)TxData,2,1000);
}
void WriteCmd(unsigned char I2C_Command)//写命令
{
	I2C_WriteByte(0x00, I2C_Command);
}

void WriteDat(unsigned char I2C_Data)//写数据
{
	I2C_WriteByte(0x40, I2C_Data);
}

void OLED_Init(void) 
{
	HAL_Delay(100);
	
	WriteCmd(0xAE); //display off
	WriteCmd(0x20);	//Set Memory Addressing Mode	
	WriteCmd(0x10);	//00,Horizontal Addressing Mode;01,Vertical Addressing Mode;10,Page Addressing Mode (RESET);11,Invalid
	WriteCmd(0xb0);	//Set Page Start Address for Page Addressing Mode,0-7
	WriteCmd(0xc8);	//Set COM Output Scan Direction
	WriteCmd(0x00); //---set low column address
	WriteCmd(0x10); //---set high column address
	WriteCmd(0x40); //--set start line address
	WriteCmd(0x81); //--set contrast control register
	WriteCmd(0xff); //亮度调节 0x00~0xff
	WriteCmd(0xa1); //--set segment re-map 0 to 127
	WriteCmd(0xa6); //--set normal display
	WriteCmd(0xa8); //--set multiplex ratio(1 to 64)
	WriteCmd(0x3F); //
	WriteCmd(0xa4); //0xa4,Output follows RAM content;0xa5,Output ignores RAM content
	WriteCmd(0xd3); //-set display offset
	WriteCmd(0x00); //-not offset
	WriteCmd(0xd5); //--set display clock divide ratio/oscillator frequency
	WriteCmd(0xf0); //--set divide ratio
	WriteCmd(0xd9); //--set pre-charge period
	WriteCmd(0x22); //
	WriteCmd(0xda); //--set com pins hardware configuration
	WriteCmd(0x12);
	WriteCmd(0xdb); //--set vcomh
	WriteCmd(0x20); //0x20,0.77xVcc
	WriteCmd(0x8d); //--set DC-DC enable
	WriteCmd(0x14); //
	WriteCmd(0xaf); //--turn on oled panel
}

//更新显存到显示器
void fflash(void)
{
	unsigned char m,n;
	for(m=0;m<8;m++)
	{
		WriteCmd(0xb0+m);		//page0-page1
		WriteCmd(0x00);		//low column start address
		WriteCmd(0x10);		//high column start address
		for(n=0;n<128;n++)
			{
				WriteDat(REG_FLASH[m][n]);
			}
	}
}

void OLED_CLS_FLASH(void)//清除显存
{
	unsigned char m,n;
	for(m=0;m<8;m++)
		for(n=0;n<128;n++)
		{REG_FLASH[m][n] = 0;}
}

void OLED_CLS(void)//清屏
{
	OLED_CLS_FLASH();
	fflash();
}

void OLED_SET_POINT(unsigned char x,unsigned char y,unsigned char set)
{
	if((y > 64) || (x > 128))
		return ;
	if(set)
		REG_FLASH[y/8][x] |= 1<<(y%8);
	else
		REG_FLASH[y/8][x] &= ~(1<<(y%8));
}

//--------------------------------------------------------------
// Prototype      : void OLED_ON(void)
// Calls          : 
// Parameters     : none
// Description    : 将OLED从休眠中唤醒
//--------------------------------------------------------------
void OLED_ON(void)
{
	WriteCmd(0X8D);  //设置电荷泵
	WriteCmd(0X14);  //开启电荷泵
	WriteCmd(0XAF);  //OLED唤醒
}

//--------------------------------------------------------------
// Prototype      : void OLED_OFF(void)
// Calls          : 
// Parameters     : none
// Description    : 让OLED休眠 -- 休眠模式下,OLED功耗不到10uA
//--------------------------------------------------------------
void OLED_OFF(void)
{
	WriteCmd(0X8D);  //设置电荷泵
	WriteCmd(0X10);  //关闭电荷泵
	WriteCmd(0XAE);  //OLED休眠
}


void OLED_ShowStr(unsigned char x, unsigned char y, unsigned char ch[], unsigned char TextSize)
{
	uint8_t j=0;
	if(x > 128 || y > 63)
		return ;
	switch(TextSize)
	{
		case 1:
			while(ch[j] != '\0')
			{
				if(ch[j] >= 32)
				{
					OLED_DrawBMP_p(x,y,6,1,(unsigned char *)F6x8[(ch[j]-32)]);
				}
				j++;
				x+=6;
			}
		break;
		case 2:
			while(ch[j] != '\0')
			{
				if(ch[j] >= 32){
					OLED_DrawBMP_p(x,y,8,2,(unsigned char *)F8X16[(ch[j]-32)]);
				}
				j++;
				x+=8;
			}
		break;	
	}
}

//区域图像绘制，(x0,y0)坐标长L宽H区域绘制图像BMP 图像高度必须为8的倍数
//0<=x0<=127 0<=y0<=63 L <=128 H <= 8 图像取模 纵向取模，字节倒序
void OLED_DrawBMP_p(unsigned char x0,unsigned char y0,unsigned char L,unsigned char H,unsigned char BMP[])
{
	uint8_t x,y,j;

	for(y=0;y<H;y++)
		for(x=x0;x<L+x0;x++)
		{
			for(j=0;j<8;j++)
				if((x<128) && (y0+y*8 < 64))
				OLED_SET_POINT(x,y0+y*8+j,((BMP[y*L+x-x0] >>j) &1));
		}	
}
//矩形区域填充，(x0,y0)-(x1,y1)区域 set =0 清除   > 0 设置点亮
void wind_full(uint8_t x0,uint8_t y0,uint8_t x1,uint8_t y1,uint8_t set)
{
	uint8_t x,y;
	for(x=0;x<128;x++)
		for(y=0;y<64;y++)
		{
			if((x>= x0 && x < x1)&& (y >= y0 && y < y1))
			OLED_SET_POINT(x,y,set);
		}
}
//获取点的状态
uint8_t oled_get_point(uint8_t x,uint8_t y)
{
	return (REG_FLASH[y/8][x] >> y%8 ) & 1;	
}

//矩形区域取反
void oled_wind_point_toggle(uint8_t x0,uint8_t y0,uint8_t x1,uint8_t y1)
{
	uint8_t x,y;
	for(x=0;x<128;x++)
		for(y=0;y<64;y++)
		{
			if((x>= x0 && x < x1)&& (y >= y0 && y < y1))		
			OLED_SET_POINT(x,y,!(oled_get_point(x,y)));
		}
}

//16*16中文显示
void oled_show_CN(uint8_t x,uint8_t y,uint8_t* CH)
{
	OLED_DrawBMP_p(x,y,16,2,CH);	
}
