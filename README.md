# stm32-IOT-camera-exp

## 0x00 实验设备&软件

| 设备名称 | 设备型号                          |
| -------- | --------------------------------- |
| 开发板   | STM32 F103ZET6                    |
| 屏幕     | MCU电阻屏，IC驱动：ILI9341/ST7789 |
| 摄像头   | OV7725                            |
| 软件     | Keil uVision5                     |

## 0x01 头文件介绍

| 文件名             | 作用                                                         |
| ------------------ | ------------------------------------------------------------ |
| stm32f10x.h        | STM32F10x芯片的寄存器定义和外设结构体，使用HAL库进行开发时必须包含的文件，可以方便地访问和配置STM32F10x芯片的外设 |
| core_cm3.h         | Cortex-M3处理器的寄存器映射以及内联函数，是CMSIS（Cortex Microcontroller Software Interface Standard）标准的一部分，用来提供对Cortex-M3的访问和控制 |
| system_stm32f10x.h | STM32F10x芯片的系统配置函数，初始化时钟系统，设置系统时钟源和时钟频率 |
| stdint.h           | 定义了int8_t、uint32_t等                                     |
| stm32f10x_conf.h   | 包含所有外设的头文件，例如GPIO、TIM、USART等，使用标准外设库时必须包含此文件 |
| stm32f10x_exti.h   | 与外部中断相关的寄存器和函数定义，使用标准外设库时必须包含此文件 |
| stm32f10x_fsmc.h   | 连接并管理外部存储，例如NOR、NAND、SRAM等                    |
| stm32f10x_gpio.h   | 定义GPIO（通用输入输出）的数据结构和函数，使用GPIO可以控制LED灯、输出音频等 |
| stm32f10x_rcc.h    | 定义时钟控制与复位相关的数据结构和函数                       |
| stm32f10x_tim.h    | 定义测量时间间隔的数据结构和函数                             |
| stm32f10x_usart.h  | 定义USART的数据结构和函数；USART（通用同步异步收发器）是一种用于串行数据传输的外设 |
| misc.h             | 定义操作内核组件的函数与结构体                               |

<img src="/images/image-20230602150936740.png" alt="image-20230602150936740" style="zoom:67%;" />

补充：

```
NOR：可像随机存取存储器一样进行随机访问，存储程序代码，写入和擦除速度较慢。
NAND：不能随机访问，存储数据，写入和擦除速度较快。
SRAM：静态随机存取存储器，适用于高速缓存。
```

## 0x02 初始化操作

### 延时函数初始化

&emsp;方便调用延时函数。首先设置SysTick定时器，它可以用来实现延时，当计数到0时，会从重装载寄存器中自动重装载定时初值，并触发中断；之后，设置fac_us与fac_ms，设置fac_us为系统时钟的1/8，STM32 F103xxxx的频率为72MHz。

&emsp;函数代码：

```c++
// delay.c
void delay_init()
{
	SysTick_CLKSourceConfig(SysTick_CLKSource_HCLK_Div8);	
	fac_us=SystemCoreClock/8000000;				           //设置fac_us为系统时钟的1/8  
	fac_ms=(u16)fac_us*1000;					            
}								    
```

```c++
// misc.c
void SysTick_CLKSourceConfig(uint32_t SysTick_CLKSource)
{
    ...
    if (SysTick_CLKSource == SysTick_CLKSource_HCLK) {
    	SysTick->CTRL |= SysTick_CLKSource_HCLK;
    }
    else {
    	SysTick->CTRL &= SysTick_CLKSource_HCLK_Div8;      //设置SysTick定时器
    }
}
// misc.h
#define SysTick_CLKSource_HCLK_Div8    ((uint32_t)0xFFFFFFFB)
```

```c++
// system_stm32f10x.c
uint32_t SystemCoreClock = SYSCLK_FREQ_72MHz;             //STM32 F103xxxx的频率
```

### 串口初始化

&emsp;初始化USART1串口。其作用是实现与外部设备的全双工通用同步/异步串行通信。USART1可以支持多种数据帧格式、波特率、校验方式、流控制和中断方式，可以用于发送和接收数据，USART1可以通过GPIOA的9和10引脚分别作为TX和RX与外部设备连接。

&emsp;函数逻辑：（1）使能USART1与GPIOA的时钟；（2）配置GPIOA的9和10引脚分别作为USART1的TX和RX，使用复用推挽输出和浮空输入；（3）之后，配置NVIC，设置中断优先级为抢占优先级3和子优先级3，并使能中断；（4）配置USART1的波特率、数据位、停止位、奇偶校验等参数；（5）使能USART1的中断和串口。

&emsp;补充：

```
1. GPIOA是STM32的一组通用输入/输出端口，共有16个引脚，可以通过软件配置成输入/输出模式
2. 复用推挽输出：GPIO引脚可以输出来自其他外设的数据，比如定时器、DAC等，而不是输出数据寄存器中的数据。复用推挽输出既可以输出高电平，又可以输出低电平，速度快，损耗小。
3. 浮空输入：GPIO引脚可以接收数字信号，但是没有上拉或下拉电阻，所以当没有信号输入时，电压不确定，容易受到干扰。浮空输入可以检测到微弱的信号。
4. NVIC：中断控制器，用于控制和管理中断的优先级、使能、挂起。
5. 抢占优先级：中断能够打断其他中断的属性，抢占优先级高（数值小）的中断可以打断抢占优先级低（数值大）的中断。
6. 子优先级：指在抢占优先级相同的情况下，哪个中断先被响应的属性，子优先级高（数值小）的中断先被响应。
```

&emsp;函数定义如下：

```c++
// usart.c
void uart_init(u32 bound){
	... 
    
    //使能USART1，GPIOA时钟
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1|RCC_APB2Periph_GPIOA, ENABLE);	
  
	//配置GPIOA的9和10引脚
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;	//复用推挽输出
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;//浮空输入
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	//配置终端控制器NVIC
	NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=3 ;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;	
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;		
	NVIC_Init(&NVIC_InitStructure);
  
	//配置USART1的参数
	USART_InitStructure.USART_BaudRate = bound;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;

	USART_Init(USART1, &USART_InitStructure);
	USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
	USART_Cmd(USART1, ENABLE);               
}
```

&emsp;波特率（每秒传输的位数）设置为115200。115200是一种常用的波特率，可以实现较高的传输效率和较低的误码率。

### 按键与屏幕初始化

&emsp;对于按键而言，我们要初始化KEY0与KEY1，其中KEY0负责转换画面风格，KEY1负责进入摄像头页面：（1）使能GPIOA和GPIOE的时钟，这样才能对端口进行配置；（2）设置GPIO_InitStructure.GPIO_Pin为GPIO_Pin_4|GPIO_Pin_3，表示要配置的是GPIOE的第4和第3引脚，也就是KEY0和KEY1；（3）设置GPIO_InitStructure.GPIO_Mode为GPIO_Mode_IPU，表示：没有按键按下时，引脚的电平为高，当按键抬起时，电平为低，并进行输入。（4）调用GPIO_Init函数，将GPIO_InitStructure的参数传递给GPIOE，完成对KEY0和KEY1的初始化。

```c++
// key.c
void KEY_Init(void)
{ 
    ...
 	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA|RCC_APB2Periph_GPIOE,ENABLE);
	GPIO_InitStructure.GPIO_Pin  = GPIO_Pin_4|GPIO_Pin_3;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
 	GPIO_Init(GPIOE, &GPIO_InitStructure);
}
```

&emsp;对于LCD屏幕的初始化，其逻辑如下：（1）使能FSMC时钟，因为LCD屏幕要通过FSMC接口与STM32通信；（2）使能GPIOB、GPIOD、GPIOE、GPIOG的时钟，因为LCD屏幕的数据线和控制线要用到这些端口；（3）配置PB0为推挽输出，用于控制LCD屏幕的背光；（4）配置PD0、PD1、PD4、PD5、PD8、PD9、PD10、PD14、PD15为复用推挽输出，用于连接FSMC的数据线和地址线；配置PE7~PE15为复用推挽输出，用于连接FSMC的数据线；配置PG0和PG12为复用推挽输出，用于连接FSMC的RS和NE4信号；（5）配置FSMC的各种参数，例如读写时序、存储器类型等。（6）初始化FSMC。

&emsp;之后，使用FSMC接口向LCD屏幕发送初始化指令和数据。例如，0x11是退出睡眠模式的指令，0x36是设置内存访问控制的指令，0x3A是设置像素格式的指令等。初始化代码如下：

```
LCD_WR_REG(0x11);

delay_ms(120);

LCD_WR_REG(0x36);
LCD_WR_DATA(0x00);

LCD_WR_REG(0x3A);
LCD_WR_DATA(0X05);

LCD_WR_REG(0xB2);
LCD_WR_DATA(0x0C);
LCD_WR_DATA(0x0C);
LCD_WR_DATA(0x00);
LCD_WR_DATA(0x33);
LCD_WR_DATA(0x33);

LCD_WR_REG(0xB7);
LCD_WR_DATA(0x35);

LCD_WR_REG(0xBB);       //vcom
LCD_WR_DATA(0x32);      //30

LCD_WR_REG(0xC0);
LCD_WR_DATA(0x0C);

LCD_WR_REG(0xC2);
LCD_WR_DATA(0x01);

LCD_WR_REG(0xC3);       //vrh
LCD_WR_DATA(0x10);      //17 0D

LCD_WR_REG(0xC4);       //vdv
LCD_WR_DATA(0x20);      //20

LCD_WR_REG(0xC6);
LCD_WR_DATA(0x0f);

LCD_WR_REG(0xD0);
LCD_WR_DATA(0xA4);
LCD_WR_DATA(0xA1);

LCD_WR_REG(0xE0);       //Set Gamma
LCD_WR_DATA(0xd0);
LCD_WR_DATA(0x00);
LCD_WR_DATA(0x02);
LCD_WR_DATA(0x07);
LCD_WR_DATA(0x0a);
LCD_WR_DATA(0x28);
LCD_WR_DATA(0x32);
LCD_WR_DATA(0X44);
LCD_WR_DATA(0x42);
LCD_WR_DATA(0x06);
LCD_WR_DATA(0x0e);
LCD_WR_DATA(0x12);
LCD_WR_DATA(0x14);
LCD_WR_DATA(0x17);

LCD_WR_REG(0XE1);       //Set Gamma
LCD_WR_DATA(0xd0);
LCD_WR_DATA(0x00);
LCD_WR_DATA(0x02);
LCD_WR_DATA(0x07);
LCD_WR_DATA(0x0a);
LCD_WR_DATA(0x28);
LCD_WR_DATA(0x31);
LCD_WR_DATA(0x54);
LCD_WR_DATA(0x47);
LCD_WR_DATA(0x0e);
LCD_WR_DATA(0x1c);
LCD_WR_DATA(0x17);
LCD_WR_DATA(0x1b);
LCD_WR_DATA(0x1e);

LCD_WR_REG(0x2A);
LCD_WR_DATA(0x00);
LCD_WR_DATA(0x00);
LCD_WR_DATA(0x00);
LCD_WR_DATA(0xef);

LCD_WR_REG(0x2B);
LCD_WR_DATA(0x00);
LCD_WR_DATA(0x00);
LCD_WR_DATA(0x01);
LCD_WR_DATA(0x3f);

LCD_WR_REG(0x29);       //display on
```

&emsp;补充：

```
1. FSMC（灵活静态存储器控制器：Flexible Static Memory Controller）是STM32的一个外设，可以让STM32通过FSMC与SRAM、ROM等存储器相连，从而进行数据的交换。
2. PB0是STM32的一个GPIO引脚，可以用于控制LCD屏幕的背光。LCD屏幕的背光是指LCD屏幕后面的一层发光材料，可以提高LCD屏幕的亮度和对比度。
3. RS是一个控制信号，用于选择命令或数据。NE4是一个片选信号，用于选择第4个NOR/PSRAM设备。这样，就可以通过不同的地址来控制LCD的读写操作。NOR/PSRAM设备是指使用FSMC接口的NOR Flash或PSRAM存储器。NOR Flash是一种非易失性的闪存存储器，可以随机访问任意地址的数据，适合用于存储程序代码。PSRAM是一种伪静态随机存取器，可以实现较大的存储容量和较低的功耗。
```

### 摄像头初始化

&emsp;初始化步骤如下：（1）配置GPIO的时钟、模式等相关的端口时钟，设置PA8为输入上拉，PB3和PB4为推挽输出，PC0~7为输入上拉，PD6、PG14和PG15为推挽输出，禁用JTAG功能；（2）调用SCCB_Init函数初始化SCCB的IO口；（3）调用SCCB_Init函数初始化SCCB的IO口，SCCB是一种串行通信协议，用于控制摄像头的寄存器；（4）判断摄像头ID；（5）遍历camera_init_reg_tb1数组，向摄像头写入一系列的初始化指令和数据。

```cpp
u8 CAMERA_Init(void)
{
	...
    //使能相关端口时钟
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA|...|RCC_APB2Periph_AFIO, ENABLE);
    //设置PA8为输入上拉
	GPIO_InitStructure.GPIO_Pin  = GPIO_Pin_8;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
 	GPIO_Init(GPIOA, &GPIO_InitStructure);
	GPIO_SetBits(GPIOA,GPIO_Pin_8);
	//设置PB3和PB4为推挽输出
 	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3|GPIO_Pin_4;
 	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; 
 	GPIO_Init(GPIOB, &GPIO_InitStructure);
 	GPIO_SetBits(GPIOB,GPIO_Pin_3|GPIO_Pin_4);	
	//设置PC0~7为输入上拉
	GPIO_InitStructure.GPIO_Pin  = 0xff; 
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
 	GPIO_Init(GPIOC, &GPIO_InitStructure);
	//设置PD6、PG14和PG15为推挽输出
	GPIO_InitStructure.GPIO_Pin  = GPIO_Pin_6;  
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
 	GPIO_Init(GPIOD, &GPIO_InitStructure);
	GPIO_SetBits(GPIOD,GPIO_Pin_6);
	GPIO_InitStructure.GPIO_Pin  = GPIO_Pin_14|GPIO_Pin_15;  
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
 	GPIO_Init(GPIOG, &GPIO_InitStructure);
	GPIO_SetBits(GPIOG,GPIO_Pin_14|GPIO_Pin_15);
	//禁用JTAG
	GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable,ENABLE);
 
    //初始化SCCB的IO口	
	SCCB_Init();      
    //软复位
 	if(SCCB_WR_Reg(0x12,0x80))
        return 1;
	delay_ms(50); 
    //判断制造商ID：MID
	reg=SCCB_RD_Reg(0X1c);
	reg<<=8;
	reg|=SCCB_RD_Reg(0X1d);
	if(reg!=CAMERA_MID)
	{
		printf("MID:%d\r\n",reg);
		return 1;
	}
    //判断产品ID：PID
	reg=SCCB_RD_Reg(0X0a);
	reg<<=8;
	reg|=SCCB_RD_Reg(0X0b);
	if(reg!=CAMERA_PID)
	{
		printf("HID:%d\r\n",reg);
		return 2;
	}   
 	//初始化摄像头
	for(i=0;i<sizeof(camera_init_reg_tb1)/sizeof(camera_init_reg_tb1[0]);i++)
	{								
	   	SCCB_WR_Reg(camera_init_reg_tb1[i][0],camera_init_reg_tb1[i][1]);
 	} 
  	return 0x00;
} 
```

补充：

```
1. JTAG|SWD是调试嵌入式系统的接口协议：
	a. JTAG（Joint Test Action Group）需要至少四根信号线：TMS（模式选择）、TCK（时钟）、TDI（数据输入）和TDO（数据输出）。JTAG接口的优点是兼容性好，支持多种设备和协议，缺点是占用的引脚较多，速度较慢。
	b. SWD（Serial Wire Debugging）是为了简化和优化JTAG接口而设计的，只需要两根信号线：SWDIO（数据输入输出）和SWCLK（时钟）。
2. SCCB是一种串行通信协议，用于控制摄像头的寄存器。
3. 寄存器版本与HAL库版本：
	a. 直接操作STM32的寄存器，需要对寄存器的地址、位域、功能等有较深入的了解，编程难度较高，但是性能较好，代码量较小，可移植性较差。
	b. HAL库版本是使用ST提供的硬件抽象层库，对寄存器进行了封装，提供了统一的API接口，编程难度较低，但是性能较差，代码量较大，可移植性较好。
```

## 0x03 摄像头相关函数

#### 设置Camera，以VGA模式输出图像：

&emsp;设置摄像头的窗口大小和位置，并以VGA模式（640x480）输出图像。函数逻辑为：（1）设置摄像头为VGA模式，并设置水平和垂直的起始位置和尺寸为640x480；（2）根据width和height的值，计算窗口的水平和垂直的偏移量sx和sy；（3）修改摄像头的寄存器HSTART、HSIZE、VSTRT、VSIZE、HREF、HOutSize、VOutSize、EXHCH，将窗口的位置和尺寸写入寄存器中。相关代码如下：

```c++
// camera.c
void camera_Window_Set(u16 width,u16 height)
{
	...
    sx=(640-width)/2;
    sy=(480-height)/2;
    SCCB_WR_Reg(COM7,0x06);		//设置为VGA模式
    SCCB_WR_Reg(HSTART,0x23); 	//水平起始位置
    SCCB_WR_Reg(HSIZE,0xA0); 	//水平尺寸
    SCCB_WR_Reg(VSTRT,0x07); 	//垂直起始位置
    SCCB_WR_Reg(VSIZE,0xF0); 	//垂直尺寸
    SCCB_WR_Reg(HREF,0x00);
    SCCB_WR_Reg(HOutSize,0xA0); //输出尺寸
    SCCB_WR_Reg(VOutSize,0xF0); //输出尺寸
	//设置HSTART、VSTRT、HREF、HOutSize、VOutSize、EXHCH寄存器
	raw=SCCB_RD_Reg(HSTART);
	temp=raw+(sx>>2);
	SCCB_WR_Reg(HSTART,temp);
	SCCB_WR_Reg(HSIZE,width>>2);
	raw=SCCB_RD_Reg(VSTRT);
	temp=raw+(sy>>1);
	SCCB_WR_Reg(VSTRT,temp);
	SCCB_WR_Reg(VSIZE,height>>1);
	raw=SCCB_RD_Reg(HREF);
	temp=((sy&0x01)<<6)|((sx&0x03)<<4)|((height&0x01)<<2)|(width&0x03)|raw;
	SCCB_WR_Reg(HREF,temp);
	SCCB_WR_Reg(HOutSize,width>>2);
	SCCB_WR_Reg(VOutSize,height>>1);
	SCCB_RD_Reg(EXHCH);	
	temp = (raw|(width&0x03)|((height&0x01)<<2));	
	SCCB_WR_Reg(EXHCH,temp);	
}
// sccb.c
// 写寄存器
u8 SCCB_WR_Reg(u8 reg,u8 data)
{
	u8 res=0;
	SCCB_Start(); 				
	if(SCCB_WR_Byte(SCCB_ID))res=1;
	delay_us(100);
  	if(SCCB_WR_Byte(reg))res=1;		  
	delay_us(100);
  	if(SCCB_WR_Byte(data))res=1; 	 
  	SCCB_Stop();	  
  	return	res;
}
// 读寄存器
u8 SCCB_RD_Reg(u8 reg)
{
	u8 val=0;
	SCCB_Start(); 		
	SCCB_WR_Byte(SCCB_ID);	 
	delay_us(100);	 
  	SCCB_WR_Byte(reg);	  
	delay_us(100);	  
	SCCB_Stop();   
	delay_us(100);	   

	SCCB_Start();
	SCCB_WR_Byte(SCCB_ID|0X01);
	delay_us(100);
  	val=SCCB_RD_Byte();	
  	SCCB_No_Ack();
  	SCCB_Stop();
  	return val;
}
```

#### 设置Camera画面风格

```cpp
void camera_Light_Mode(u8 mode)
{
	switch(mode)
	{
		case 0:	//Auto，自动模式
			SCCB_WR_Reg(0x13, 0xff);
			SCCB_WR_Reg(0x0e, 0x65);
			SCCB_WR_Reg(0x2d, 0x00);
			SCCB_WR_Reg(0x2e, 0x00);
			break;
		case 1://sunny，晴天
			SCCB_WR_Reg(0x13, 0xfd);
			SCCB_WR_Reg(0x01, 0x5a);
			SCCB_WR_Reg(0x02, 0x5c);
			SCCB_WR_Reg(0x0e, 0x65);
			SCCB_WR_Reg(0x2d, 0x00);
			SCCB_WR_Reg(0x2e, 0x00);
			break;	
		case 2://cloudy，多云
			SCCB_WR_Reg(0x13, 0xfd);
			SCCB_WR_Reg(0x01, 0x58);
			SCCB_WR_Reg(0x02, 0x60);
			SCCB_WR_Reg(0x0e, 0x65);
			SCCB_WR_Reg(0x2d, 0x00);
			SCCB_WR_Reg(0x2e, 0x00);
			break;	
		case 3://office，办公室
			SCCB_WR_Reg(0x13, 0xfd);
			SCCB_WR_Reg(0x01, 0x84);
			SCCB_WR_Reg(0x02, 0x4c);
			SCCB_WR_Reg(0x0e, 0x65);
			SCCB_WR_Reg(0x2d, 0x00);
			SCCB_WR_Reg(0x2e, 0x00);
			break;	
		case 4://home，家里
			SCCB_WR_Reg(0x13, 0xfd);
			SCCB_WR_Reg(0x01, 0x96);
			SCCB_WR_Reg(0x02, 0x40);
			SCCB_WR_Reg(0x0e, 0x65);
			SCCB_WR_Reg(0x2d, 0x00);
			SCCB_WR_Reg(0x2e, 0x00);
			break;	

		case 5://night，夜晚
			SCCB_WR_Reg(0x13, 0xff);
			SCCB_WR_Reg(0x0e, 0xe5);
			break;
	}
}
```



## 0x04 LCD相关函数

#### LCD输出字符串：

```c++
// lcd.c
// 参数介绍：
// （1）x,y:起点坐标
// （2）width,height:区域大小
// （3）size:字体大小
// （4）*p:字符串起始地址
void LCD_ShowString(u16 x, u16 y, u16 width, u16 height, u8 size, u8 *p)
{
    u8 x0 = x;
    width += x;
    height += y;
    while True:
    {
        if (x >= width)
        {
            x = x0;
            y += size;
        }
        if (y >= height)
            break;
        LCD_ShowChar(x, y, *p, size, 0);
        x += size / 2;
        p++;
    }
}
// 画字符
void LCD_ShowChar(u16 x, u16 y, u8 num, u8 size, u8 mode)
{
    u8 temp, t1, t;
    u16 y0 = y;
    u8 csize = (size / 8 + ((size % 8) ? 1 : 0)) * (size / 2);  //得到字体一个字符对应点阵集所占的字节数
    num = num - ' ';    //得到偏移后的值（ASCII字库是从空格开始取模，所以-' '就是对应字符的字库）

    for (t = 0; t < csize; t++)
    {
        if (size == 16)
            temp = asc2_1608[num][t];   //调用1608字体
        else 
            return;

        for (t1 = 0; t1 < 8; t1++)
        {
            if (temp & 0x80)
                LCD_Fast_DrawPoint(x, y, POINT_COLOR);
            else if (mode == 0)
                LCD_Fast_DrawPoint(x, y, BACK_COLOR);
            temp <<= 1;
            y++;
            if ((y - y0) == size)
            {
                y = y0;
                x++;
                break;
            }
        }
    }
}
// 画点
void LCD_Fast_DrawPoint(u16 x, u16 y, u16 color)
{
    LCD_WR_REG(lcddev.setxcmd);
    LCD_WR_DATA(x >> 8);
    LCD_WR_DATA(x & 0XFF);
    LCD_WR_REG(lcddev.setycmd);
    LCD_WR_DATA(y >> 8);
    LCD_WR_DATA(y & 0XFF);

    LCD->LCD_REG=lcddev.wramcmd; 
    LCD->LCD_RAM=color; 
}
```

#### LCD屏幕填充颜色：

```c++
//lcd.c
// 参数介绍：
//（1）区域大小:(ex-sx+1)*(ey-sy+1)
//（2）color:要填充的颜色
void LCD_Fill(u16 sx, u16 sy, u16 ex, u16 ey, u16 color)
{
    u16 i, j;
    u16 xlen = 0;

    xlen = ex - sx + 1;

    for (i = sy; i <= ey; i++)
    {
        LCD_SetCursor(sx, i);       //设置光标位置
        LCD_WriteRAM_Prepare();     //开始写入GRAM

        for (j = 0; j < xlen; j++)
        {
            LCD->LCD_RAM=color;     //设置光标位置
        }
    }
}
```

## 0x05 按键相关函数

```cpp
//key.c
//按键处理函数
//0.没有任何按键按下
//1.KEY0按下
//2.KEY1按下
u8 KEY_Scan(u8 mode)
{	 
	static u8 key_up=1;
	if(mode)
        key_up=1;	  
	if(key_up&&(KEY0==0||KEY1==0)){
		key_up=0;
		if(KEY0==0)
            return KEY0_PRES;
		else if(KEY1==0)
            return KEY1_PRES;
	}else if(KEY0==1&&KEY1==1)
        key_up=1;
 	return 0;
}
```

## 0x06 main函数

&emsp;其中，camera_refresh的逻辑为：（1）判断ov_sta变量是否为真，如果为真，表示有帧中断发生，即摄像头采集到了一帧图像；（2）设置LCD的扫描方向为从上到下，从左到右；（3）设置LCD的显示窗口为屏幕中央的一块区域，大小为CAMARA_WINDOW_WIDTH x CAMARA_WINDOW_HEIGHT；（4）准备写入LCD的GRAM（图形存储器）；（5）复位摄像头的读指针，使其指向第一个像素数据；（6）用双层循环遍历摄像头的每个像素，每次读取两个字节（16位）的数据，即一个像素的颜色值；（7）将读取到的颜色值写入LCD的GRAM中，显示在对应的位置上；（8）清零ov_sta变量，表示帧中断已经处理完毕；（9）恢复LCD的默认扫描方向。

&emsp;最终实验的展示效果为：

（1）屏幕首先输出"wd2711's camera exp"。

（2）如果摄像头正常进行初始化，那么输出"camera init success"，并闪烁输出"press key1 to start"。

（3）按下key1按键，之后进入摄像头画面，画面为VGA格式。

（4）按下key0，可以改变画面风格，一共有6种风格。

```cpp
#include "delay.h"
#include "key.h"
#include "sys.h"
#include "lcd.h"
#include "usart.h"	 
#include "string.h"
#include "camera.h"
#include "timer.h"
#include "exti.h"
#include "usmart.h"

#define  CAMERA 1

// 定义高度与宽度
#define  CAMARA_WINDOW_WIDTH		320 // <=320
#define  CAMARA_WINDOW_HEIGHT		240 // <=240


u8 *LMODE_TBL[6]={"Auto","Sunny","Cloudy","Office","Home","Night"};//6种画面风格    
extern u8 ov_sta;	//在exit.c里定义

//更新LCD显示
void camera_refresh(void)
{
	u32 i,j;
 	u16 color;	 
	if(ov_sta) //有帧中断
	{
		LCD_Scan_Dir(U2D_L2R);		//从上到下,从左到右
		LCD_Set_Window((lcddev.width-CAMARA_WINDOW_WIDTH)/2,(lcddev.height-CAMARA_WINDOW_HEIGHT)/2,CAMARA_WINDOW_WIDTH,CAMARA_WINDOW_HEIGHT);//将显示区域设置到屏幕中央
		LCD_WriteRAM_Prepare();      //开始写入GRAM	
		CAMERA_RRST=0;				//开始复位读指针 
		CAMERA_RCK_L;
		CAMERA_RCK_H;
		CAMERA_RCK_L;
		CAMERA_RRST=1;				//复位读指针结束 
		CAMERA_RCK_H; 
		for(i=0;i<CAMARA_WINDOW_HEIGHT;i++)
		{
			for(j=0;j<CAMARA_WINDOW_WIDTH;j++)
			{
				CAMERA_RCK_L;
				color=GPIOC->IDR&0XFF;	//读数据
				CAMERA_RCK_H; 
				color<<=8;  
				CAMERA_RCK_L;
				color|=GPIOC->IDR&0XFF;	//读数据
				CAMERA_RCK_H; 
				LCD->LCD_RAM=color;  
			}
		}
 		ov_sta=0;				   //清零帧中断标记
		LCD_Scan_Dir(DFT_SCAN_DIR);	//恢复默认扫描方向 
	} 
}


int main(void)
{
	u8 key;  
	u8 i=0;
	u8 lightmode=0;

	delay_init();	    	 //延时函数初始化	  
	uart_init(115200);	 	 //串口初始化
	KEY_Init();				//按键初始化
	LCD_Init();			   	//LCD初始化  
	LCD_ShowString(30,50,200,16,16,"wd2711's camera exp");	

	while(1)
	{
		if(CAMERA_Init()==0)//摄像头初始化
		{
			LCD_ShowString(30,210,200,16,16,"camera init success");
			while(1)
			{
				key=KEY_Scan(0);
				if(key==KEY1_PRES) // 按下key1就进入画面
				{
					CAMERA_Window_Set(OV7725_WINDOW_WIDTH,OV7725_WINDOW_HEIGHT,1);
					break;
				}
				i++;
				if(i==100)
                    LCD_ShowString(30,230,210,16,16,"press key1 to start"); //闪烁显示提示信息
				if(i==200)
				{	
					LCD_Fill(30,230,210,250+16,WHITE);
					i=0; 
				}
				delay_ms(5);
			}				
			CAMERA_CS=0; // 用来表示摄像头的片选信号（CS）的引脚。它的意思是将PG15引脚作为输出，并用PGout(15)来控制它的高低电平。当PGout(15)为0时，表示选中摄像头，当为1时，表示不选中摄像头。
			break;
		}
	}

	EXTI8_Init(); //使能外部中断，捕获帧中断			
 	while(1)
	{	
		LCD_ShowString(30,250,210,16,16,"press key0 to change mode");
		LCD_ShowString(30,270,210,16,16,"now mode is ");
		LCD_ShowString(30,290,210,16,16,LMODE_TBL[lightmode]);
		if(key)
		{
			switch(key)
			{				    
				case KEY0_PRES:
					lightmode++;
					if(lightmode>5)
						lightmode=0;
					camera_Light_Mode(lightmode);
					break;
			}
		}	 
		camera_refresh();//更新显示
	}	   
}
```

## 0x07 最终展示效果

&emsp;初始界面：

<img src="/images/image-20230603175046004.png" alt="image-20230603175046004" style="zoom:67%;" />

&emsp;进入摄像头画面：

<img src="/images/image-20230603175211044.png" alt="image-20230603175211044" style="zoom:67%;" />

&emsp;更改画面风格：

<img src="/images/image-20230603175347165.png" alt="image-20230603175347165" style="zoom:67%;" />