/* Standard includes. */
#include <stdio.h>

/* Scheduler includes. */
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

/* Library includes. */
#include "stm32f10x_it.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_tim.h"
#include "stm32f10x.h"

/* USB Library includes. */
#include "usb_lib.h"
#include "hw_config.h"
#include "usb_pwr.h"

#include "app.h"

#define KEY_SEL    0x01
#define KEY_RIGHT  0x02
#define KEY_LEFT   0x04
#define KEY_DOWN   0x10
#define KEY_UP     0x08
#define KEY_2      0x20
#define KEY_3      0x40
#define KSP				 vTaskDelay( 300/portTICK_RATE_MS );Joystick_Send(0)
#define Dltime		 1000000
#define Go_Left vTaskDelay( 300/portTICK_RATE_MS );Joystick_Send(KEY_LEFT);KSP
#define Go_Right vTaskDelay( 300/portTICK_RATE_MS );Joystick_Send(KEY_RIGHT);KSP

#define State_00 (ADC_Left<ADC_MID&&ADC_Right<ADC_MID)
#define State_10 (ADC_Left>ADC_MID&&ADC_Right<ADC_MID)
#define State_01 (ADC_Left<ADC_MID&&ADC_Right>ADC_MID)
#define State_11 (ADC_Left>ADC_MID&&ADC_Right>ADC_MID)

#define ADC_Left ADCConvertedValue[0]
#define ADC_Right ADCConvertedValue[1]
#define ADC_MIDs 250

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
unsigned int ADC_MID;
/* Extern variables ----------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
void Delay(__IO uint32_t nCount);

static void prvSetupHardware( void );

static void vLED1( void *pvParameters );

static void vLED2( void *pvParameters );

static void vUSB( void *pvParameters );

static void vUart( void *pvParameters );

/* Private functions ---------------------------------------------------------*/

/*******************************************************************************
* Function Name  : main.
* Description    : main routine.
* Input          : None.
* Output         : None.
* Return         : None.
*******************************************************************************/
int main(void)
{
	prvSetupHardware();

	xTaskCreate( vLED1, "LED1 Shrink", configMINIMAL_STACK_SIZE, NULL, tskIDLE_PRIORITY+3, NULL );
	
	xTaskCreate( vLED2, "LED2 Shrink", configMINIMAL_STACK_SIZE, NULL, tskIDLE_PRIORITY+3, NULL );
	
	xTaskCreate( vUSB, "USB HID Device", configMINIMAL_STACK_SIZE, NULL, tskIDLE_PRIORITY+4, NULL );
	
	xTaskCreate( vUart, "UART", configMINIMAL_STACK_SIZE, NULL, tskIDLE_PRIORITY+5, NULL );

	vTaskStartScheduler();

	return 0;	
}

/*******************************************************************************
* Function Name  : Delay
* Description    : Inserts a delay time.
* Input          : nCount: specifies the delay time length.
* Output         : None
* Return         : None
*******************************************************************************/
void Delay(__IO uint32_t nCount)
{
  for (; nCount != 0;nCount--);
}

static void vLED1( void *pvParameters )
{
	while(1)
	{	
		GPIO_WriteBit(GPIOC,GPIO_Pin_13,Bit_SET);
		vTaskDelay( 100/portTICK_RATE_MS );  
		GPIO_WriteBit(GPIOC,GPIO_Pin_13,Bit_RESET);
		vTaskDelay( 100/portTICK_RATE_MS );
	}
}

static void vLED2( void *pvParameters )
{
	while(1)
	{	
		GPIO_WriteBit(GPIOB,GPIO_Pin_0,Bit_SET);
		vTaskDelay( 200/portTICK_RATE_MS );  
		GPIO_WriteBit(GPIOB,GPIO_Pin_0,Bit_RESET);
		vTaskDelay( 300/portTICK_RATE_MS );
	}
}

static void vUSB( void *pvParameters )
{
  while (1)
  {
		ADC_MID = (ADC_Left+ADC_Right)/2 + 50;
		if(State_10)
			if(State_10)
			{
				while (!(State_01));
				vTaskDelay( 300/portTICK_RATE_MS );Joystick_Send(KEY_RIGHT);KSP;
			}
		if(State_01)
			if(State_01)
			{
				while (!(State_10));
				vTaskDelay( 300/portTICK_RATE_MS );Joystick_Send(KEY_LEFT);KSP;
			}

		//vTaskDelay( 300/portTICK_RATE_MS );Joystick_Send(KEY_LEFT);KSP;
		
  }	
}

static void vUart( void *pvParameters )
{
  while (1)
  {
		printf(">>");
		printf("|%d   %d\n\r",ADCConvertedValue[0],ADCConvertedValue[1]);		//PA3,PA2
		vTaskDelay( 500/portTICK_RATE_MS );
	}	
}

static void prvSetupHardware( void )
{
	GPIO_InitTypeDef  GPIO_InitStructure; 
	
	Set_System();
  USB_Interrupts_Config();
  Set_USBClock();
  USB_Init();
	NVIC_Config();							//中断分组及优先级配置
	Usart_Configuration();			//串口初始化，波特率115200
	ADC1_DMA_Config(); 					//ADC1的DMA初始化
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE );
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE );
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13 ;
	GPIO_Init(GPIOC, &GPIO_InitStructure);
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 ;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
}

#ifdef  USE_FULL_ASSERT
/*******************************************************************************
* Function Name  : assert_failed
* Description    : Reports the name of the source file and the source line number
*                  where the assert_param error has occurred.
* Input          : - file: pointer to the source file name
*                  - line: assert_param error line source number
* Output         : None
* Return         : None
*******************************************************************************/
void assert_failed(uint8_t* file, uint32_t line)
{
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

  /* Infinite loop */
  while (1)
  {}
}
#endif

/******************* (C) COPYRIGHT 2009 STMicroelectronics *****END OF FILE****/
