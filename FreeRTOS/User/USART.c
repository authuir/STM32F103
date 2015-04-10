#include "stm32f10x.h"
#include "stm32f10x_usart.h"
#include <stdio.h>	//����strlen������Ҫ��ͷ�ļ�
#include "USART.h"

/**********************************************************
** ������:u32tostr
** ��������: ��һ��32λ�ı���datתΪ�ַ����������1234תΪ"1234"
** �������: dat:��ת��long�͵ı���
             str:ָ���ַ������ָ�룬ת������ֽڴ��������� 
** �������: ��
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
** ������: strtou32
** ��������: ��һ���ַ���תΪ32λ�ı���������"1234"תΪ1234
** �������: str:ָ���ת�����ַ���     
** �������: ��
** ���أ�ת�������ֵ  
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
** ������: Usart_Configuration
** ��������: ����1����, ��������ʱ�ӣ�GPIO����
** �������: ��
** �������: ��
***********************************************************/
void Usart_Configuration(void)
{
	 GPIO_InitTypeDef GPIO_InitStructure; //GPIO�⺯���ṹ��
	 USART_InitTypeDef USART_InitStructure;//USART�⺯���ṹ��
	 USART_ClockInitTypeDef USART_ClockInitStructure;
	 //ʹ�ܴ���1��GPIOA��AFIO����
	 RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_AFIO|RCC_APB2Periph_USART1,ENABLE);
	 /* Configure USART1 Tx (PA9) as alternate function push-pull */
	 GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
	 GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;//PA9ʱ���ٶ�50MHz
	 GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP; //�������
	 GPIO_Init(GPIOA, &GPIO_InitStructure);
	 /* Configure USART1 Rx (PA10) as input floating */
	 GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
	 GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;	//��������
	 GPIO_Init(GPIOA, &GPIO_InitStructure);
	 
	 USART_InitStructure.USART_BaudRate =115200; //������115200
	 USART_InitStructure.USART_WordLength = USART_WordLength_8b; //8λ����
	 USART_InitStructure.USART_StopBits = USART_StopBits_1; //1��ֹͣλ
	 USART_InitStructure.USART_Parity = USART_Parity_No; //��żʧ��
	 USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None; //Ӳ��������ʧ��
	 USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx; //���͡�����ʹ��
	
	 USART_ClockInitStructure.USART_Clock = USART_Clock_Disable;
	 USART_ClockInitStructure.USART_CPOL = USART_CPOL_Low;//����ʱ��Ϊ�͵�ƽ
	 USART_ClockInitStructure.USART_CPHA = USART_CPHA_2Edge;//ʱ�ӵڶ������ؽ������ݲ���
	 USART_ClockInitStructure.USART_LastBit = USART_LastBit_Disable;//���һλ���ݵ�ʱ�����岻��SCLK���
	
	 USART_ClockInit(USART1, &USART_ClockInitStructure);
	 USART_Init(USART1, &USART_InitStructure);	//��ʼ���ṹ��
	 USART_Cmd(USART1, ENABLE); //ʹ�ܴ���1	
}

//�������´���,֧��printf����,������Ҫѡ��use MicroLIB	  
/***************************START*********************/
#if 1
#pragma import(__use_no_semihosting)             
//��׼����Ҫ��֧�ֺ���                 
struct __FILE 
{ 
	int handle; 
	/* Whatever you require here. If the only file you are using is */ 
	/* standard output using printf() for debugging, no file handling */ 
	/* is required. */ 
}; 
/* FILE is typedef�� d in stdio.h. */ 
FILE __stdout;       
//����_sys_exit()�Ա���ʹ�ð�����ģʽ    
_sys_exit(int x) 
{ 
	x = x; 
} 
//�ض���fputc���� 
int fputc(int ch, FILE *f)
{      
	USART1->DR = (u8) ch;      
	while((USART1->SR&0X40)==0);//ѭ������,ֱ���������   
	return ch;
}
#endif 
/***************************END*********************/

/**********************************************************
** ������:USART1_Putc
** ��������: ����1����һ�ַ�
** �������: c
** �������: ��
***********************************************************/
void USART1_Putc(unsigned char c)
{
    USART1->DR = (u8)c; //Ҫ���͵��ַ������������ݼĴ���  
	while((USART1->SR&0X40)==0); //�ȴ��������  
}
/**********************************************************
** ������:USART1_Puts
** ��������: ����1����һ�ַ���
** �������: ָ��str
** �������: ��
***********************************************************/
void USART1_Puts(char * str)
{
    while(*str)
    {
        USART1->DR= *str++;
		while((USART1->SR&0X40)==0);//�ȴ��������  
    }
}
/**********************************************************
** ������:UART_Send_Enter
** ��������: ����1����һ���з�
** �������: ��
** �������: ��
***********************************************************/
void UART_Send_Enter(void)
{
	USART1_Putc(0x0d);
	USART1_Putc(0x0a);
}
/**********************************************************
** ������:UART_Send_Str
** ��������: ����1����һ�ַ��������س����й���
** �������: ָ��s
** �������: ��
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
** ������: UART_Put_Num
** ��������: STM32F��USART������ֵ
** �������: dat:Ҫ���͵���ֵ
** �������: ��
** ˵���������лὫ��ֵתΪ��Ӧ���ַ��������ͳ�ȥ������ 4567 תΪ "4567" 
***********************************************************/
void UART_Put_Num(unsigned long dat)
{
	char temp[20];
	u32tostr(dat,temp);
	UART_Send_Str(temp);
}
/**********************************************************
** ������: UART_Put_Inf
** ��������: STM32F��USART���͵�����Ϣ
** �������: inf:ָ����ʾ��Ϣ�ַ�����ָ��
             dat:һ����ֵ��ǰ�����ʾ��Ϣ������˵�������ֵ������
** �������: �� 
***********************************************************/
void UART_Put_Inf(char *inf,unsigned long dat)
{
	UART_Send_Str(inf);
	UART_Put_Num(dat);
	UART_Send_Str("\n");  
}
