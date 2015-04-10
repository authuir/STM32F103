#include "stm32f10x.h"
#include "stm32f10x_usart.h"
#include <stdio.h>	//下面strlen函数需要此头文件
#include "USART.h"

/**********************************************************
** 函数名:u32tostr
** 功能描述: 将一个32位的变量dat转为字符串，比如把1234转为"1234"
** 输入参数: dat:带转的long型的变量
             str:指向字符数组的指针，转换后的字节串放在其中 
** 输出参数: 无
***********************************************************/
void u32tostr(unsigned long dat,char *str) 
{
	char temp[20];
	unsigned char i=0,j=0;
	i=0;
	while(dat)
	{
		temp[i]=dat%10+0x30;
		i++;
		dat/=10;
	}
	j=i;
	for(i=0;i<j;i++)
	{
	  	str[i]=temp[j-i-1];
	}
	if(!i) {str[i++]='0';}
	str[i]=0;
}
/**********************************************************
** 函数名: strtou32
** 功能描述: 将一个字符串转为32位的变量，比如"1234"转为1234
** 输入参数: str:指向待转换的字符串     
** 输出参数: 无
** 返回：转换后的数值  
***********************************************************/
unsigned long strtou32(char *str) 
{
	unsigned long temp=0;
	unsigned long fact=1;
	unsigned char len=strlen(str);
	unsigned char i;
	for(i=len;i>0;i--)
	{
		temp+=((str[i-1]-0x30)*fact);
		fact*=10;
	}
	return temp;
}
/**********************************************************
** 函数名: Usart_Configuration
** 功能描述: 串口1配置, 包括串口时钟，GPIO配置
** 输入参数: 无
** 输出参数: 无
***********************************************************/
void Usart_Configuration(void)
{
	 GPIO_InitTypeDef GPIO_InitStructure; //GPIO库函数结构体
	 USART_InitTypeDef USART_InitStructure;//USART库函数结构体
	 USART_ClockInitTypeDef USART_ClockInitStructure;
	 //使能串口1，GPIOA，AFIO总线
	 RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_AFIO|RCC_APB2Periph_USART1,ENABLE);
	 /* Configure USART1 Tx (PA9) as alternate function push-pull */
	 GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
	 GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;//PA9时钟速度50MHz
	 GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP; //复用输出
	 GPIO_Init(GPIOA, &GPIO_InitStructure);
	 /* Configure USART1 Rx (PA10) as input floating */
	 GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
	 GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;	//上拉输入
	 GPIO_Init(GPIOA, &GPIO_InitStructure);
	 
	 USART_InitStructure.USART_BaudRate =115200; //波特率115200
	 USART_InitStructure.USART_WordLength = USART_WordLength_8b; //8位数据
	 USART_InitStructure.USART_StopBits = USART_StopBits_1; //1个停止位
	 USART_InitStructure.USART_Parity = USART_Parity_No; //奇偶失能
	 USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None; //硬件流控制失能
	 USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx; //发送、接收使能
	
	 USART_ClockInitStructure.USART_Clock = USART_Clock_Disable;
	 USART_ClockInitStructure.USART_CPOL = USART_CPOL_Low;//空闲时钟为低电平
	 USART_ClockInitStructure.USART_CPHA = USART_CPHA_2Edge;//时钟第二个边沿进行数据捕获
	 USART_ClockInitStructure.USART_LastBit = USART_LastBit_Disable;//最后一位数据的时钟脉冲不从SCLK输出
	
	 USART_ClockInit(USART1, &USART_ClockInitStructure);
	 USART_Init(USART1, &USART_InitStructure);	//初始化结构体
	 USART_Cmd(USART1, ENABLE); //使能串口1	
}

//加入以下代码,支持printf函数,而不需要选择use MicroLIB	  
/***************************START*********************/
#if 1
#pragma import(__use_no_semihosting)             
//标准库需要的支持函数                 
struct __FILE 
{ 
	int handle; 
	/* Whatever you require here. If the only file you are using is */ 
	/* standard output using printf() for debugging, no file handling */ 
	/* is required. */ 
}; 
/* FILE is typedef’ d in stdio.h. */ 
FILE __stdout;       
//定义_sys_exit()以避免使用半主机模式    
_sys_exit(int x) 
{ 
	x = x; 
} 
//重定义fputc函数 
int fputc(int ch, FILE *f)
{      
	USART1->DR = (u8) ch;      
	while((USART1->SR&0X40)==0);//循环发送,直到发送完毕   
	return ch;
}
#endif 
/***************************END*********************/

/**********************************************************
** 函数名:USART1_Putc
** 功能描述: 串口1发送一字符
** 输入参数: c
** 输出参数: 无
***********************************************************/
void USART1_Putc(unsigned char c)
{
    USART1->DR = (u8)c; //要发送的字符赋给串口数据寄存器  
	while((USART1->SR&0X40)==0); //等待发送完成  
}
/**********************************************************
** 函数名:USART1_Puts
** 功能描述: 串口1发送一字符串
** 输入参数: 指针str
** 输出参数: 无
***********************************************************/
void USART1_Puts(char * str)
{
    while(*str)
    {
        USART1->DR= *str++;
		while((USART1->SR&0X40)==0);//等待发送完成  
    }
}
/**********************************************************
** 函数名:UART_Send_Enter
** 功能描述: 串口1发送一换行符
** 输入参数: 无
** 输出参数: 无
***********************************************************/
void UART_Send_Enter(void)
{
	USART1_Putc(0x0d);
	USART1_Putc(0x0a);
}
/**********************************************************
** 函数名:UART_Send_Str
** 功能描述: 串口1发送一字符串，带回车换行功能
** 输入参数: 指针s
** 输出参数: 无
***********************************************************/
void UART_Send_Str(char *s)
{
 
 	for(;*s;s++)
 	{
	 	if(*s=='\n') 
	  		UART_Send_Enter();
	 	else
	  		USART1_Putc(*s);
 	}
}
/**********************************************************
** 函数名: UART_Put_Num
** 功能描述: STM32F的USART发送数值
** 输入参数: dat:要发送的数值
** 输出参数: 无
** 说明：函数中会将数值转为相应的字符串，发送出去。比如 4567 转为 "4567" 
***********************************************************/
void UART_Put_Num(unsigned long dat)
{
	char temp[20];
	u32tostr(dat,temp);
	UART_Send_Str(temp);
}
/**********************************************************
** 函数名: UART_Put_Inf
** 功能描述: STM32F的USART发送调试信息
** 输入参数: inf:指向提示信息字符串的指针
             dat:一个数值，前面的提示信息就是在说明这个数值的意义
** 输出参数: 无 
***********************************************************/
void UART_Put_Inf(char *inf,unsigned long dat)
{
	UART_Send_Str(inf);
	UART_Put_Num(dat);
	UART_Send_Str("\n");  
}
