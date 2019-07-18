#ifndef __oled_H
#define __oled_H
#include "stm32f1xx_hal.h"

#define	CODE6X8		1
#define	CODE8X16	2



extern unsigned char REG_FLASH[8][128];


void OLED_Init(void);//初始化OLED
void fflash(void);//更新显存
void OLED_CLS_FLASH(void);//清除显存
void OLED_CLS(void);//清屏
void OLED_SET_POINT(unsigned char x,unsigned char y,unsigned char set);//设置显存的点

void OLED_ON(void);//开启电荷泵
void OLED_OFF(void);//关闭电荷泵

//加载图片
//区域图像绘制，(x0,y0)坐标长L宽H区域绘制图像BMP
//0<=x0<=127 0<=y0<=63 L <=128 H <= 8,宽必须为8的倍数
void OLED_DrawBMP_p(unsigned char x0,unsigned char y0,unsigned char L,unsigned char H,unsigned char BMP[]);


//显示英文字符串
void OLED_ShowStr(unsigned char x, unsigned char y, unsigned char ch[], unsigned char TextSize);

//矩形区域填充，(x0,y0)-(x1,y1)区域 set =0 清除   > 0 设置点亮
void wind_full(uint8_t x0,uint8_t y0,uint8_t x1,uint8_t y1,uint8_t set);

//获取点的状态
uint8_t oled_get_point(uint8_t x,uint8_t y);

//矩形区域取反
void oled_wind_point_toggle(uint8_t x0,uint8_t y0,uint8_t x1,uint8_t y1);

//16*16中文显示
void oled_show_CN(uint8_t x,uint8_t y,uint8_t* CH);

#endif 
