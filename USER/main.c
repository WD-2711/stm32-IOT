#include "delay.h"
#include "key.h"
#include "sys.h"
#include "lcd.h"
#include "usart.h"	 
#include "string.h"
#include "ov7725.h"
#include "timer.h"
#include "exti.h"
#include "usmart.h"

#define  OV7725 1

//由于OV7725传感器安装方式原因,OV7725_WINDOW_WIDTH相当于LCD的高度，OV7725_WINDOW_HEIGHT相当于LCD的宽度
//注意：此宏定义只对OV7725有效
#define  OV7725_WINDOW_WIDTH		320 // <=320
#define  OV7725_WINDOW_HEIGHT		240 // <=240


u8 *LMODE_TBL[6]={"Auto","Sunny","Cloudy","Office","Home","Night"};//6种光照模式	    
extern u8 ov_sta;	//在exit.c里 面定义

//更新LCD显示(OV7725)
void OV7725_camera_refresh(void)
{
	u32 i,j;
 	u16 color;	 
	if(ov_sta)//有帧中断更新
	{
		LCD_Scan_Dir(U2D_L2R);		//从上到下,从左到右
		LCD_Set_Window((lcddev.width-OV7725_WINDOW_WIDTH)/2,(lcddev.height-OV7725_WINDOW_HEIGHT)/2,OV7725_WINDOW_WIDTH,OV7725_WINDOW_HEIGHT);//将显示区域设置到屏幕中央
		if(lcddev.id==0X1963)
			LCD_Set_Window((lcddev.width-OV7725_WINDOW_WIDTH)/2,(lcddev.height-OV7725_WINDOW_HEIGHT)/2,OV7725_WINDOW_HEIGHT,OV7725_WINDOW_WIDTH);//将显示区域设置到屏幕中央
		LCD_WriteRAM_Prepare();     //开始写入GRAM	
		OV7725_RRST=0;				//开始复位读指针 
		OV7725_RCK_L;
		OV7725_RCK_H;
		OV7725_RCK_L;
		OV7725_RRST=1;				//复位读指针结束 
		OV7725_RCK_H; 
		for(i=0;i<OV7725_WINDOW_HEIGHT;i++)
		{
			for(j=0;j<OV7725_WINDOW_WIDTH;j++)
			{
				OV7725_RCK_L;
				color=GPIOC->IDR&0XFF;	//读数据
				OV7725_RCK_H; 
				color<<=8;  
				OV7725_RCK_L;
				color|=GPIOC->IDR&0XFF;	//读数据
				OV7725_RCK_H; 
				LCD->LCD_RAM=color;  
			}
		}
 		ov_sta=0;					//清零帧中断标记
		LCD_Scan_Dir(DFT_SCAN_DIR);	//恢复默认扫描方向 
	} 
}





int main(void)
{
	// u8, s8 数据结构定义在 stm32f10x.h 中
	u8 key;  
	u8 i=0;
	u8 lightmode=0;//,effect=0;

	delay_init();	    	 //延时函数初始化	  
	uart_init(115200);	 	//串口初始化为 115200
	KEY_Init();					//初始化按键
	LCD_Init();			   		//初始化LCD  
 	POINT_COLOR=BLACK;			//设置字体为黑色 
	LCD_ShowString(30,50,200,16,16,"wd2711's camera exp");	

	while(1)
	{
		if(OV7725_Init()==0)
		{
			LCD_ShowString(30,210,200,16,16,"camera init success       ");
			while(1)
			{
				key=KEY_Scan(0);
				if(key==KEY1_PRES)
				{
					OV7725_Window_Set(OV7725_WINDOW_WIDTH,OV7725_WINDOW_HEIGHT,1);//VGA模式输出
					break;
				}
				i++;
				if(i==100)LCD_ShowString(30,230,210,16,16,"press key1 to start"); //闪烁显示提示信息
				if(i==200)
				{	
					LCD_Fill(30,230,210,250+16,WHITE);
					i=0; 
				}
				delay_ms(5);
			}				
			OV7725_CS=0; // 用来表示OV7725摄像头的片选信号（CS）的引脚。它的意思是将PG15引脚作为输出，并用PGout(15)来控制它的高低电平。当PGout(15)为0时，表示选中摄像头，当为1时，表示不选中摄像头。
			break;
		}
		else
		{
			LCD_ShowString(30,210,200,16,16,"camera init error!!");
			delay_ms(200);
			LCD_Fill(30,210,239,246,WHITE);
			delay_ms(200);
		}
	}

	EXTI8_Init();						//使能外部中断8,捕获帧中断			
 	while(1)
	{	
		LCD_ShowString(30,250,210,16,16,"press key0 to change mode");
		LCD_ShowString(30,270,210,16,16,"now mode is ");
		LCD_ShowString(30,290,210,16,16,LMODE_TBL[lightmode]);
		key=KEY_Scan(0);//不支持连按
		if(key)
		{
			switch(key)
			{				    
				case KEY0_PRES:
					lightmode++;
					if(lightmode>5)
						lightmode=0;
					OV7725_Light_Mode(lightmode);
					break;
			}
		}	 
		OV7725_camera_refresh();//更新显示
	}	   
}













