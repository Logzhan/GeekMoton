#include "lcd_driver.h"
#include "lcd.h"
#include "lcdfont.h"
#include "pic.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"

static const char *TAG        = "GEEKLCD";
int lcd_enable_flag = 0;

/**----------------------------------------------------------------------
* Function    : LCD_Fill
* Description : xsta,ysta   起始坐标
                xend,yend   终止坐标
				color       要填充的颜色
* Author      : zhanli&719901725@qq.com
* Date        : 2021/12/17  zhanli
*---------------------------------------------------------------------**/
void LCD_Fill(u16 xsta,u16 ysta,u16 xend,u16 yend,u16 color)
{          
	u16 i,j; 
	lcd_address_set(xsta,ysta,xend-1,yend-1);
	for(i=ysta;i<yend;i++){													   	 	
		for(j=xsta;j<xend;j++){
			lcd_write_data(color);
		}
	} 					  	    
}

/**----------------------------------------------------------------------
* Function    : LCD_DrawPoint
* Description : 在指定位置画点
				x,y 画点坐标
                color 点的颜色
* Author      : zhanli&719901725@qq.com
* Date        : 2021/12/17 zhanli
*---------------------------------------------------------------------**/
void LCD_DrawPoint(u16 x,u16 y,u16 color)
{
	// 设置光标位置 
	lcd_address_set(x,y,x,y);
	lcd_write_data(color);
} 

/**----------------------------------------------------------------------
* Function    : LCD_DrawLine
* Description : 画线
				x1,y1   起始坐标
                x2,y2   终止坐标
                color   线的颜色
* Author      : zhanli&719901725@qq.com
* Date        : 2021/12/17 zhanli
*---------------------------------------------------------------------**/
void LCD_DrawLine(u16 x1,u16 y1,u16 x2,u16 y2,u16 color)
{
	u16 t; 
	int xerr=0,yerr=0,delta_x,delta_y,distance;
	int incx,incy,uRow,uCol;
	// 计算坐标增量 
	delta_x=x2-x1; 
	delta_y=y2-y1;
	//画线起点坐标
	uRow=x1;
	uCol=y1;
	// 设置单步方向 
	if(delta_x>0)incx=1; 
	else if (delta_x==0)incx=0;  // 垂直线 
	else {incx=-1;delta_x=-delta_x;}
	
	if(delta_y>0)incy=1;
	else if (delta_y==0)incy=0;  // 水平线 
	else {incy=-1;delta_y=-delta_y;}
	// 选取基本增量坐标轴 
	if(delta_x>delta_y)distance=delta_x;
	else distance=delta_y;
	for(t=0;t<distance+1;t++)
	{
		// 画点
		LCD_DrawPoint(uRow,uCol,color);
		xerr += delta_x;
		yerr += delta_y;
		if(xerr>distance){
			xerr -= distance;
			uRow += incx;
		}
		if(yerr > distance){
			yerr -= distance;
			uCol += incy;
		}
	}
}
/**----------------------------------------------------------------------
* Function    : LCD_DrawRectangle
* Description : 画矩形
                x1,y1   起始坐标
                x2,y2   终止坐标
                color   矩形的颜色
* Author      : zhanli&719901725@qq.com
* Date        : 2021/12/17 zhanli
*---------------------------------------------------------------------**/
void LCD_DrawRectangle(u16 x1, u16 y1, u16 x2, u16 y2,u16 color)
{
	LCD_DrawLine(x1,y1,x2,y1,color);
	LCD_DrawLine(x1,y1,x1,y2,color);
	LCD_DrawLine(x1,y2,x2,y2,color);
	LCD_DrawLine(x2,y1,x2,y2,color);
}

/**----------------------------------------------------------------------
* Function    : Draw_Circle
* Description : 画圆
       		  ：x0,y0   圆心坐标
                r       半径
                color   圆的颜色
* Author      : zhanli&719901725@qq.com
* Date        : 2021/12/17 zhanli
*---------------------------------------------------------------------**/
void Draw_Circle(u16 x0,u16 y0,u8 r,u16 color)
{
	int a,b;
	a=0;b=r;	  
	while(a<=b)
	{
		LCD_DrawPoint(x0-b,y0-a,color);             //3           
		LCD_DrawPoint(x0+b,y0-a,color);             //0           
		LCD_DrawPoint(x0-a,y0+b,color);             //1                
		LCD_DrawPoint(x0-a,y0-b,color);             //2             
		LCD_DrawPoint(x0+b,y0+a,color);             //4               
		LCD_DrawPoint(x0+a,y0-b,color);             //5
		LCD_DrawPoint(x0+a,y0+b,color);             //6 
		LCD_DrawPoint(x0-b,y0+a,color);             //7
		a++;
		if((a*a+b*b)>(r*r)) // 判断要画的点是否过远
		{
			b--;
		}
	}
}
/**----------------------------------------------------------------------
* Function    : LCD_ShowChinese
* Description : 显示汉字串
      			x,y显示坐标
                *s 要显示的汉字串
                fc 字的颜色
                bc 字的背景色
                sizey 字号 可选 16 24 32
                mode:  0非叠加模式  1叠加模式
* Author      : zhanli&719901725@qq.com
* Date        : 2021/12/17 zhanli
*---------------------------------------------------------------------**/
void LCD_ShowChinese(u16 x,u16 y,u8 *s,u16 fc,u16 bc,u8 sizey,u8 mode)
{
	while(*s!=0)
	{
		if(sizey==12) LCD_ShowChinese12x12(x,y,s,fc,bc,sizey,mode);
		else if(sizey==16) LCD_ShowChinese16x16(x,y,s,fc,bc,sizey,mode);
		else if(sizey==24) LCD_ShowChinese24x24(x,y,s,fc,bc,sizey,mode);
		else if(sizey==32) LCD_ShowChinese32x32(x,y,s,fc,bc,sizey,mode);
		else return;
		s+=2;
		x+=sizey;
	}
}

/**----------------------------------------------------------------------
* Function    : LCD_ShowChinese12x12
* Description : 显示单个12x12汉字
      			x,y显示坐标
                *s 要显示的汉字
                fc 字的颜色
                bc 字的背景色
                sizey 字号
                mode:  0非叠加模式  1叠加模式
* Description : 显示单个12x12汉字
* Author      : zhanli&719901725@qq.com
* Date        : 2021/12/17 zhanli
*---------------------------------------------------------------------**/
void LCD_ShowChinese12x12(u16 x,u16 y,u8 *s,u16 fc,u16 bc,u8 sizey,u8 mode)
{
	// u8 i,j,m=0;
	// u16 k;
	// u16 HZnum;//???????
	// u16 TypefaceNum;//?????????????��
	// u16 x0=x;
	// TypefaceNum=(sizey/8+((sizey%8)?1:0))*sizey;
	                         
	// HZnum=sizeof(tfont12)/sizeof(typFNT_GB12);	//?????????
	// for(k=0;k<HZnum;k++) 
	// {
	// 	if((tfont12[k].Index[0]==*(s))&&(tfont12[k].Index[1]==*(s+1)))
	// 	{ 	
	// 		LCD_Address_Set(x,y,x+sizey-1,y+sizey-1);
	// 		for(i=0;i<TypefaceNum;i++)
	// 		{
	// 			for(j=0;j<8;j++)
	// 			{	
	// 				if(!mode)//???????
	// 				{
	// 					if(tfont12[k].Msk[i]&(0x01<<j))LCD_WR_DATA(fc);
	// 					else LCD_WR_DATA(bc);
	// 					m++;
	// 					if(m%sizey==0)
	// 					{
	// 						m=0;
	// 						break;
	// 					}
	// 				}
	// 				else//??????
	// 				{
	// 					if(tfont12[k].Msk[i]&(0x01<<j))	LCD_DrawPoint(x,y,fc);//???????
	// 					x++;
	// 					if((x-x0)==sizey)
	// 					{
	// 						x=x0;
	// 						y++;
	// 						break;
	// 					}
	// 				}
	// 			}
	// 		}
	// 	}				  	
	// 	continue;  //??????????????????????????????????????????????
	// }
} 
/**----------------------------------------------------------------------
* Function    : LCD_ShowChinese16x16
* Description : 显示单个16x16汉字
				x,y显示坐标
                *s 要显示的汉字
                fc 字的颜色
                bc 字的背景色
                sizey 字号
                mode:  0非叠加模式  1叠加模式
* Description : 显示单个12x12汉字
* Author      : zhanli&719901725@qq.com
* Date        : 2021/12/17 zhanli
*---------------------------------------------------------------------**/
void LCD_ShowChinese16x16(u16 x,u16 y,u8 *s,u16 fc,u16 bc,u8 sizey,u8 mode)
{
	// u8 i,j,m=0;
	// u16 k;
	// u16 HZnum;//???????
	// u16 TypefaceNum;//?????????????��
	// u16 x0=x;
	// TypefaceNum=(sizey/8+((sizey%8)?1:0))*sizey;
	// HZnum=sizeof(tfont16)/sizeof(typFNT_GB16);	//?????????
	// for(k=0;k<HZnum;k++) 
	// {
	// 	if ((tfont16[k].Index[0]==*(s))&&(tfont16[k].Index[1]==*(s+1)))
	// 	{ 	
	// 		LCD_Address_Set(x,y,x+sizey-1,y+sizey-1);
	// 		for(i=0;i<TypefaceNum;i++)
	// 		{
	// 			for(j=0;j<8;j++)
	// 			{	
	// 				if(!mode)//???????
	// 				{
	// 					if(tfont16[k].Msk[i]&(0x01<<j))LCD_WR_DATA(fc);
	// 					else LCD_WR_DATA(bc);
	// 					m++;
	// 					if(m%sizey==0)
	// 					{
	// 						m=0;
	// 						break;
	// 					}
	// 				}
	// 				else//??????
	// 				{
	// 					if(tfont16[k].Msk[i]&(0x01<<j))	LCD_DrawPoint(x,y,fc);//???????
	// 					x++;
	// 					if((x-x0)==sizey)
	// 					{
	// 						x=x0;
	// 						y++;
	// 						break;
	// 					}
	// 				}
	// 			}
	// 		}
	// 	}				  	
	// 	continue;  //??????????????????????????????????????????????
	// }
} 

/**----------------------------------------------------------------------
* Function    : LCD_ShowChinese24x24
* Description : 显示单个24x24汉字
				x,y显示坐标
                *s 要显示的汉字
                fc 字的颜色
                bc 字的背景色
                sizey 字号
                mode:  0非叠加模式  1叠加模式
* Description : 显示单个12x12汉字
* Author      : zhanli&719901725@qq.com
* Date        : 2021/12/17 zhanli
*---------------------------------------------------------------------**/
void LCD_ShowChinese24x24(u16 x,u16 y,u8 *s,u16 fc,u16 bc,u8 sizey,u8 mode)
{
	// u8 i,j,m=0;
	// u16 k;
	// u16 HZnum;//???????
	// u16 TypefaceNum;//?????????????��
	// u16 x0=x;
	// TypefaceNum=(sizey/8+((sizey%8)?1:0))*sizey;
	// HZnum=sizeof(tfont24)/sizeof(typFNT_GB24);	//?????????
	// for(k=0;k<HZnum;k++) 
	// {
	// 	if ((tfont24[k].Index[0]==*(s))&&(tfont24[k].Index[1]==*(s+1)))
	// 	{ 	
	// 		LCD_Address_Set(x,y,x+sizey-1,y+sizey-1);
	// 		for(i=0;i<TypefaceNum;i++)
	// 		{
	// 			for(j=0;j<8;j++)
	// 			{	
	// 				if(!mode)//???????
	// 				{
	// 					if(tfont24[k].Msk[i]&(0x01<<j))LCD_WR_DATA(fc);
	// 					else LCD_WR_DATA(bc);
	// 					m++;
	// 					if(m%sizey==0)
	// 					{
	// 						m=0;
	// 						break;
	// 					}
	// 				}
	// 				else//??????
	// 				{
	// 					if(tfont24[k].Msk[i]&(0x01<<j))	LCD_DrawPoint(x,y,fc);//???????
	// 					x++;
	// 					if((x-x0)==sizey)
	// 					{
	// 						x=x0;
	// 						y++;
	// 						break;
	// 					}
	// 				}
	// 			}
	// 		}
	// 	}				  	
	// 	continue;  //??????????????????????????????????????????????
	// }
} 

/**----------------------------------------------------------------------
* Function    : LCD_ShowChinese32x32
* Description : 显示单个32x32汉字
				x,y显示坐标
                *s 要显示的汉字
                fc 字的颜色
                bc 字的背景色
                sizey 字号
                mode:  0非叠加模式  1叠加模式
* Description : 显示单个12x12汉字
* Author      : zhanli&719901725@qq.com
* Date        : 2021/12/17 zhanli
*---------------------------------------------------------------------**/
void LCD_ShowChinese32x32(u16 x,u16 y,u8 *s,u16 fc,u16 bc,u8 sizey,u8 mode)
{
	// u8 i,j,m=0;
	// u16 k;
	// u16 HZnum;//???????
	// u16 TypefaceNum;//?????????????��
	// u16 x0=x;
	// TypefaceNum=(sizey/8+((sizey%8)?1:0))*sizey;
	// HZnum=sizeof(tfont32)/sizeof(typFNT_GB32);	//?????????
	// for(k=0;k<HZnum;k++) 
	// {
	// 	if ((tfont32[k].Index[0]==*(s))&&(tfont32[k].Index[1]==*(s+1)))
	// 	{ 	
	// 		LCD_Address_Set(x,y,x+sizey-1,y+sizey-1);
	// 		for(i=0;i<TypefaceNum;i++)
	// 		{
	// 			for(j=0;j<8;j++)
	// 			{	
	// 				if(!mode)//???????
	// 				{
	// 					if(tfont32[k].Msk[i]&(0x01<<j))LCD_WR_DATA(fc);
	// 					else LCD_WR_DATA(bc);
	// 					m++;
	// 					if(m%sizey==0)
	// 					{
	// 						m=0;
	// 						break;
	// 					}
	// 				}
	// 				else//??????
	// 				{
	// 					if(tfont32[k].Msk[i]&(0x01<<j))	LCD_DrawPoint(x,y,fc);//???????
	// 					x++;
	// 					if((x-x0)==sizey)
	// 					{
	// 						x=x0;
	// 						y++;
	// 						break;
	// 					}
	// 				}
	// 			}
	// 		}
	// 	}				  	
	// 	continue;  //??????????????????????????????????????????????
	// }
}

/**----------------------------------------------------------------------
* Function    : LCD_ShowChinese32x32
* Description : 显示单个字符
				x,y显示坐标
                num 要显示的字符
                fc 字的颜色
                bc 字的背景色
                sizey 字号
                mode:  0非叠加模式  1叠加模式
* Description : 显示单个12x12汉字
* Author      : zhanli&719901725@qq.com
* Date        : 2021/12/17 zhanli
*---------------------------------------------------------------------**/
void LCD_ShowChar(u16 x,u16 y,u8 num,u16 fc,u16 bc,u8 sizey,u8 mode)
{
	u8 temp,sizex,t,m=0;
	u16 i,TypefaceNum;//?????????????��
	u16 x0=x;
	sizex=sizey/2;
	TypefaceNum=(sizex/8+((sizex%8)?1:0))*sizey;
	num=num-' ';    //?????????
	lcd_address_set(x,y,x+sizex-1,y+sizey-1);  //???��??��?? 
	for(i=0;i<TypefaceNum;i++)
	{ 
		if(sizey==12)temp=ascii_1206[num][i];		       //????6x12????
		else if(sizey==16)temp=ascii_1608[num][i];		 //????8x16????
		else if(sizey==24)temp=ascii_2412[num][i];		 //????12x24????
		else if(sizey==32)temp=ascii_3216[num][i];		 //????16x32????
		else return;
		for(t=0;t<8;t++)
		{
			if(!mode)//???????
			{
				if(temp&(0x01<<t))lcd_write_data(fc);
				else lcd_write_data(bc);
				m++;
				if(m%sizex==0)
				{
					m=0;
					break;
				}
			}
			else//??????
			{
				if(temp&(0x01<<t))LCD_DrawPoint(x,y,fc);//???????
				x++;
				if((x-x0)==sizex)
				{
					x=x0;
					y++;
					break;
				}
			}
		}
	}   	 	  
}

/**----------------------------------------------------------------------
* Function    : LCD_ShowString
* Description : 显示字符串
				x,y显示坐标
                *p 要显示的字符串
                fc 字的颜色
                bc 字的背景色
                sizey 字号
* Author      : zhanli&719901725@qq.com
* Date        : 2021/12/17 zhanli
*---------------------------------------------------------------------**/
void LCD_ShowString(u16 x,u16 y,const u8 *p,u16 fc,u16 bc,u8 sizey,u8 mode){         
	while(*p!='\0'){       
		LCD_ShowChar(x,y,*p,fc,bc,sizey,mode);
		x+=sizey/2;
		p++;
	}  
}

/**----------------------------------------------------------------------
* Function    : mypow
* Description : 显示数字
				m底数，n指数
* Author      : zhanli&719901725@qq.com
* Date        : 2021/12/17 zhanli
*---------------------------------------------------------------------**/
u32 mypow(u8 m,u8 n){
	u32 result=1;	 
	while(n--)result*=m;
	return result;
}

/**----------------------------------------------------------------------
* Function    : LCD_ShowIntNum
* Description : 显示整数变量
				x,y显示坐标
                num 要显示整数变量
                len 要显示的位数
                fc 字的颜色
                bc 字的背景色
                sizey 字号
* Author      : zhanli&719901725@qq.com
* Date        : 2021/12/17 zhanli
*---------------------------------------------------------------------**/
void LCD_ShowIntNum(u16 x,u16 y,u16 num,u8 len,u16 fc,u16 bc,u8 sizey)
{         	
	u8 t,temp;
	u8 enshow=0;
	u8 sizex=sizey/2;
	for(t=0;t<len;t++){
		temp=(num/mypow(10,len-t-1))%10;
		if(enshow==0&&t<(len-1))
		{
			if(temp==0)
			{
				LCD_ShowChar(x+t*sizex,y,' ',fc,bc,sizey,0);
				continue;
			}else enshow=1; 
		 	 
		}
	 	LCD_ShowChar(x+t*sizex,y,temp+48,fc,bc,sizey,0);
	}
} 

/**----------------------------------------------------------------------
* Function    : LCD_ShowFloatNum1
* Description : 显示两位小数变量
                x,y显示坐标
                num 要显示小数变量
                len 要显示的位数
                fc 字的颜色
                bc 字的背景色
                sizey 字号
* Author      : zhanli&719901725@qq.com
* Date        : 2021/12/17 zhanli
*---------------------------------------------------------------------**/
void LCD_ShowFloatNum1(u16 x,u16 y,float num,u8 len,u16 fc,u16 bc,u8 sizey)
{         	
	u8 t,temp,sizex;
	u16 num1;
	sizex=sizey/2;
	num1=num*100;
	for(t=0;t<len;t++){
		temp=(num1/mypow(10,len-t-1))%10;
		if(t==(len-2))
		{
			LCD_ShowChar(x+(len-2)*sizex,y,'.',fc,bc,sizey,0);
			t++;
			len+=1;
		}
	 	LCD_ShowChar(x+t*sizex,y,temp+48,fc,bc,sizey,0);
	}
}

/**----------------------------------------------------------------------
* Function    : LCD_ShowPicture
* Description : 显示图片
                x,y起点坐标
                length 图片长度
                width  图片宽度
                pic[]  图片数组  
* Author      : zhanli&719901725@qq.com
* Date        : 2021/12/17 zhanli
*---------------------------------------------------------------------**/
void LCD_ShowPicture(u16 x,u16 y, u16 length,u16 width, const u8 pic[])
{
	u16 i,j;
	u32 k=0;
	lcd_address_set(x,y,x+length-1,y+width-1);
	for(i=0;i<length;i++){
		for(j=0;j<width;j++)
		{
			lcd_write_data_u8(pic[k*2]);
			lcd_write_data_u8(pic[k*2+1]);
			k++;
		}
	}			
}

void lcd_init(void* para)
{
	float t=0;
	ESP_LOGI(TAG, "lcd hardware init.\n");
	lcd_hw_init();
	LCD_Fill(0,0,LCD_W,LCD_H,WHITE);
	LCD_ShowPicture(160,95,40,40,gImage_1);
	lcd_enable_flag = 1;
	while(1){
		lcd_enable_flag = 0;
		LCD_ShowFloatNum1(10,99,t,4,RED,WHITE,32);
		lcd_enable_flag = 1;
		t+=0.5;
		vTaskDelay(500 / portTICK_PERIOD_MS);
	}
}

void lcd_show_battery_voltage(float voltage){
	if(lcd_enable_flag == 1){
		//LCD_ShowFloatNum1(10,10,voltage,4,RED,WHITE,16);
	}
}

void lcd_show_battery_capacity(float capacity){
	if(lcd_enable_flag == 1){
		LCD_ShowIntNum(200,5,capacity,3,RED,WHITE,16);
		LCD_ShowString(225,5,(unsigned char*)"%",RED,WHITE,16, 0);
	}
}


