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

//����OV7725��������װ��ʽԭ��,OV7725_WINDOW_WIDTH�൱��LCD�ĸ߶ȣ�OV7725_WINDOW_HEIGHT�൱��LCD�Ŀ��
//ע�⣺�˺궨��ֻ��OV7725��Ч
#define  OV7725_WINDOW_WIDTH		320 // <=320
#define  OV7725_WINDOW_HEIGHT		240 // <=240


u8 *LMODE_TBL[6]={"Auto","Sunny","Cloudy","Office","Home","Night"};//6�ֹ���ģʽ	    
extern u8 ov_sta;	//��exit.c�� �涨��

//����LCD��ʾ(OV7725)
void OV7725_camera_refresh(void)
{
	u32 i,j;
 	u16 color;	 
	if(ov_sta)//��֡�жϸ���
	{
		LCD_Scan_Dir(U2D_L2R);		//���ϵ���,������
		LCD_Set_Window((lcddev.width-OV7725_WINDOW_WIDTH)/2,(lcddev.height-OV7725_WINDOW_HEIGHT)/2,OV7725_WINDOW_WIDTH,OV7725_WINDOW_HEIGHT);//����ʾ�������õ���Ļ����
		if(lcddev.id==0X1963)
			LCD_Set_Window((lcddev.width-OV7725_WINDOW_WIDTH)/2,(lcddev.height-OV7725_WINDOW_HEIGHT)/2,OV7725_WINDOW_HEIGHT,OV7725_WINDOW_WIDTH);//����ʾ�������õ���Ļ����
		LCD_WriteRAM_Prepare();     //��ʼд��GRAM	
		OV7725_RRST=0;				//��ʼ��λ��ָ�� 
		OV7725_RCK_L;
		OV7725_RCK_H;
		OV7725_RCK_L;
		OV7725_RRST=1;				//��λ��ָ����� 
		OV7725_RCK_H; 
		for(i=0;i<OV7725_WINDOW_HEIGHT;i++)
		{
			for(j=0;j<OV7725_WINDOW_WIDTH;j++)
			{
				OV7725_RCK_L;
				color=GPIOC->IDR&0XFF;	//������
				OV7725_RCK_H; 
				color<<=8;  
				OV7725_RCK_L;
				color|=GPIOC->IDR&0XFF;	//������
				OV7725_RCK_H; 
				LCD->LCD_RAM=color;  
			}
		}
 		ov_sta=0;					//����֡�жϱ��
		LCD_Scan_Dir(DFT_SCAN_DIR);	//�ָ�Ĭ��ɨ�跽�� 
	} 
}





int main(void)
{
	// u8, s8 ���ݽṹ������ stm32f10x.h ��
	u8 key;  
	u8 i=0;
	u8 lightmode=0;//,effect=0;

	delay_init();	    	 //��ʱ������ʼ��	  
	uart_init(115200);	 	//���ڳ�ʼ��Ϊ 115200
	KEY_Init();					//��ʼ������
	LCD_Init();			   		//��ʼ��LCD  
 	POINT_COLOR=BLACK;			//��������Ϊ��ɫ 
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
					OV7725_Window_Set(OV7725_WINDOW_WIDTH,OV7725_WINDOW_HEIGHT,1);//VGAģʽ���
					break;
				}
				i++;
				if(i==100)LCD_ShowString(30,230,210,16,16,"press key1 to start"); //��˸��ʾ��ʾ��Ϣ
				if(i==200)
				{	
					LCD_Fill(30,230,210,250+16,WHITE);
					i=0; 
				}
				delay_ms(5);
			}				
			OV7725_CS=0; // ������ʾOV7725����ͷ��Ƭѡ�źţ�CS�������š�������˼�ǽ�PG15������Ϊ���������PGout(15)���������ĸߵ͵�ƽ����PGout(15)Ϊ0ʱ����ʾѡ������ͷ����Ϊ1ʱ����ʾ��ѡ������ͷ��
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

	EXTI8_Init();						//ʹ���ⲿ�ж�8,����֡�ж�			
 	while(1)
	{	
		LCD_ShowString(30,250,210,16,16,"press key0 to change mode");
		LCD_ShowString(30,270,210,16,16,"now mode is ");
		LCD_ShowString(30,290,210,16,16,LMODE_TBL[lightmode]);
		key=KEY_Scan(0);//��֧������
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
		OV7725_camera_refresh();//������ʾ
	}	   
}













