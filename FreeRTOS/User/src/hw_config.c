/******************** (C) COPYRIGHT 2009 STMicroelectronics ********************
* File Name          : hw_config.c
* Author             : MCD Application Team
* Version            : V3.0.1
* Date               : 04/27/2009
* Description        : Hardware Configuration & Setup
********************************************************************************
* THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
* WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE TIME.
* AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY DIRECT,
* INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING FROM THE
* CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE CODING
* INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
*******************************************************************************/

/* Includes ------------------------------------------------------------------*/
#include "stm32f10x.h"
#include "stm32f10x_exti.h"
#include "stm32f10x_pwr.h"
#include "hw_config.h"
#include "usb_lib.h"
#include "usb_desc.h"
#include "platform_config.h"
#include "usb_pwr.h"

#define KEY_SEL    0x01
#define KEY_RIGHT  0x02
#define KEY_LEFT   0x04
#define KEY_DOWN   0x10
#define KEY_UP     0x08
#define KEY_2      0x20
#define KEY_3      0x40

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
ErrorStatus HSEStartUpStatus;
EXTI_InitTypeDef EXTI_InitStructure;

/* Extern variables ----------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/*******************************************************************************
* Function Name  : Set_System
* Description    : Configures Main system clocks & power.
* Input          : None.
* Return         : None.
*******************************************************************************/
void Set_System(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;

  /* Setup the microcontroller system. Initialize the Embedded Flash Interface,  
     initialize the PLL and update the SystemFrequency variable. */
  SystemInit();
  
  /* enable the PWR clock */
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_PWR, ENABLE);

  /* Set all the GPIOs to AIN */
  GPIO_AINConfig();

  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIO_DISCONNECT, ENABLE);


  /* USB_DISCONNECT used as USB pull-up */
  GPIO_InitStructure.GPIO_Pin = USB_DISCONNECT_PIN;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_OD;
  GPIO_Init(USB_DISCONNECT, &GPIO_InitStructure);


  /* Enable Joystick GPIOs clock */
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIO_JOY_SET1, ENABLE);
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIO_JOY_SET2, ENABLE);

  /* Configure the JoyStick IOs */
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_UP;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
  GPIO_Init(GPIO_UP, &GPIO_InitStructure);

  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_DOWN;
  GPIO_Init(GPIO_DOWN, &GPIO_InitStructure);

  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_LEFT;
  GPIO_Init(GPIO_LEFT, &GPIO_InitStructure);

  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_RIGHT;
  GPIO_Init(GPIO_RIGHT, &GPIO_InitStructure);

  /* Enable GPIOB & AFIO clocks */
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIO_KEY | RCC_APB2Periph_AFIO, ENABLE);

  /* Configure the Key pin as input floating */
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_KEY;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
  GPIO_Init(GPIO_KEY, &GPIO_InitStructure);

  /* Connect EXTI Line9 */
  GPIO_EXTILineConfig(GPIO_KEY_PORTSOURCE, GPIO_KEY_PINSOURCE);

  /* Configure EXTI Line9 to generate an interrupt on falling edge */
  EXTI_InitStructure.EXTI_Line = GPIO_KEY_EXTI_Line;
  EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
  EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;
  EXTI_InitStructure.EXTI_LineCmd = ENABLE;
  EXTI_Init(&EXTI_InitStructure);

  EXTI_ClearITPendingBit(EXTI_Line18);
  EXTI_InitStructure.EXTI_Line = EXTI_Line18; // USB resume from suspend mode
  EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;
  EXTI_InitStructure.EXTI_LineCmd = ENABLE;
  EXTI_Init(&EXTI_InitStructure);

  EXTI_ClearITPendingBit(GPIO_KEY_EXTI_Line);
}

/*******************************************************************************
* Function Name  : Set_USBClock
* Description    : Configures USB Clock input (48MHz).
* Input          : None.
* Output         : None.
* Return         : None.
*******************************************************************************/
void Set_USBClock(void)
{
  /* Select USBCLK source */
  RCC_USBCLKConfig(RCC_USBCLKSource_PLLCLK_1Div5);

  /* Enable USB clock */
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_USB, ENABLE);
}

/*******************************************************************************
* Function Name  : GPIO_AINConfig
* Description    : Configures all IOs as AIN to reduce the power consumption.
* Input          : None.
* Output         : None.
* Return         : None.
*******************************************************************************/
void GPIO_AINConfig(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;

  /* Enable all GPIOs Clock*/
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_ALLGPIO, ENABLE);

  /* Configure all GPIO port pins in Analog Input mode (floating input trigger OFF) */
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_All;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
  GPIO_Init(GPIOA, &GPIO_InitStructure);
  GPIO_Init(GPIOB, &GPIO_InitStructure);
  GPIO_Init(GPIOC, &GPIO_InitStructure);
  GPIO_Init(GPIOD, &GPIO_InitStructure);
  GPIO_Init(GPIOE, &GPIO_InitStructure);


#ifdef USE_STM3210E_EVAL
  GPIO_Init(GPIOF, &GPIO_InitStructure);
  GPIO_Init(GPIOG, &GPIO_InitStructure);
#endif /* USE_STM3210E_EVAL */

  /* Disable all GPIOs Clock*/
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_ALLGPIO, DISABLE);
}
/*******************************************************************************
* Function Name  : Enter_LowPowerMode.
* Description    : Power-off system clocks and power while entering suspend mode.
* Input          : None.
* Output         : None.
* Return         : None.
*******************************************************************************/
void Enter_LowPowerMode(void)
{
  /* Set the device state to suspend */
  bDeviceState = SUSPENDED;

  /* Clear EXTI Line18 pending bit */
  EXTI_ClearITPendingBit(GPIO_KEY_EXTI_Line);

  /* Request to enter STOP mode with regulator in low power mode */
  PWR_EnterSTOPMode(PWR_Regulator_LowPower, PWR_STOPEntry_WFI);
}

/*******************************************************************************
* Function Name  : Leave_LowPowerMode.
* Description    : Restores system clocks and power while exiting suspend mode.
* Input          : None.
* Output         : None.
* Return         : None.
*******************************************************************************/
void Leave_LowPowerMode(void)
{
  DEVICE_INFO *pInfo = &Device_Info;

  /* Enable HSE */
  RCC_HSEConfig(RCC_HSE_ON);

  /* Wait till HSE is ready */
  HSEStartUpStatus = RCC_WaitForHSEStartUp();

  /* Enable PLL */
  RCC_PLLCmd(ENABLE);

  /* Wait till PLL is ready */
  while (RCC_GetFlagStatus(RCC_FLAG_PLLRDY) == RESET)
  {}

  /* Select PLL as system clock source */
  RCC_SYSCLKConfig(RCC_SYSCLKSource_PLLCLK);

  /* Wait till PLL is used as system clock source */
  while (RCC_GetSYSCLKSource() != 0x08)
  {}

  /* Set the device state to the correct state */
  if (pInfo->Current_Configuration != 0)
  {
    /* Device configured */
    bDeviceState = CONFIGURED;
  }
  else
  {
    bDeviceState = ATTACHED;
  }
}

/*******************************************************************************
* Function Name  : USB_Interrupts_Config.
* Description    : Configures the USB interrupts.
* Input          : None.
* Output         : None.
* Return         : None.
*******************************************************************************/
void USB_Interrupts_Config(void)
{
  NVIC_InitTypeDef NVIC_InitStructure;

  /* 2 bit for pre-emption priority, 2 bits for subpriority */
  NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);

  /* Enable the USB interrupt */
  NVIC_InitStructure.NVIC_IRQChannel = USB_LP_CAN1_RX0_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);

  /* Enable the USB Wake-up interrupt */
  NVIC_InitStructure.NVIC_IRQChannel = USBWakeUp_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);

  /* Enable the Key EXTI line Interrupt */
  NVIC_InitStructure.NVIC_IRQChannel = EXTI_KEY_IRQChannel;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
}

/*******************************************************************************
* Function Name  : USB_Cable_Config.
* Description    : Software Connection/Disconnection of USB Cable.
* Input          : NewState: new state.
* Output         : None.
* Return         : None
*******************************************************************************/
void USB_Cable_Config (FunctionalState NewState)
{
  if (NewState != DISABLE)
  {
    GPIO_ResetBits(USB_DISCONNECT, USB_DISCONNECT_PIN);
  }
  else
  {
    GPIO_SetBits(USB_DISCONNECT, USB_DISCONNECT_PIN);
  }
}

/*******************************************************************************
* Function Name : JoyState.
* Description   : Decodes the Joystick direction.
* Input         : None.
* Output        : None.
* Return value  : The direction value.
*******************************************************************************/
uint8_t JoyState(void)
{
  /* "right" key is pressed */
  if (!GPIO_ReadInputDataBit(GPIO_RIGHT, GPIO_Pin_RIGHT))
  {
    return RIGHT;
  }
  /* "left" key is pressed */
  if (!GPIO_ReadInputDataBit(GPIO_LEFT, GPIO_Pin_LEFT))
  {
    return LEFT;
  }
  /* "up" key is pressed */
  if (!GPIO_ReadInputDataBit(GPIO_UP, GPIO_Pin_UP))
  {
    return UP;
  }
  /* "down" key is pressed */
  if (!GPIO_ReadInputDataBit(GPIO_DOWN, GPIO_Pin_DOWN))
  {
    return DOWN;
  }
  /* No key is pressed */
  else
  {
    return 0;
  }
}

/*******************************************************************************
* Function Name : Joystick_Send.
* Description   : prepares buffer to be sent containing Joystick event infos.
* Input         : Keys: keys received from terminal.
* Output        : None.
* Return value  : None.
*******************************************************************************/
void Joystick_Send(uint8_t Keys)
{
  u8 Buffer[8] = {0, 0, 0, 0, 0, 0, 0, 0};
  u8 i;
  i=2;
  

  if(Keys&KEY_UP)
  {
   Buffer[i]=0x52; //Keyboard UpArrow
   i++;
  }
  if(Keys&KEY_DOWN)
  {
   Buffer[i]=0x51; //Keyboard DownArrow
   i++;
  }
  if(Keys&KEY_LEFT)
  {
   Buffer[i]=0x50; //Keyboard LeftArrow
   i++;
  }
  if(Keys&KEY_RIGHT)
  {
   Buffer[i]=0x4F; //Keyboard RightArrow
   i++;
  }
  if(Keys&KEY_2)
  {
   Buffer[i]=0x39; //Keyboard Caps Lock
   i++;
  }
  if(Keys&KEY_3)
  {
   Buffer[i]=0x53; //Keypad Num Lock and Clear
   i++;
  }
  if(Keys&KEY_SEL)
  {
   Buffer[i]=0x28; //Keyboard Return (ENTER)
  }
  /*copy mouse position info in ENDP1 Tx Packet Memory Area*/
  UserToPMABufferCopy(Buffer, GetEPTxAddr(ENDP1), 8);   //?8???
  /* enable endpoint for transmission */
  SetEPTxValid(ENDP1);
}

/*******************************************************************************
* Function Name  : Get_SerialNum.
* Description    : Create the serial number string descriptor.
* Input          : None.
* Output         : None.
* Return         : None.
*******************************************************************************/
void Get_SerialNum(void)
{
  uint32_t Device_Serial0, Device_Serial1, Device_Serial2;

  Device_Serial0 = *(uint32_t*)(0x1FFFF7E8);
  Device_Serial1 = *(uint32_t*)(0x1FFFF7EC);
  Device_Serial2 = *(uint32_t*)(0x1FFFF7F0);

  if (Device_Serial0 != 0)
  {
    Joystick_StringSerial[2] = (uint8_t)(Device_Serial0 & 0x000000FF);
    Joystick_StringSerial[4] = (uint8_t)((Device_Serial0 & 0x0000FF00) >> 8);
    Joystick_StringSerial[6] = (uint8_t)((Device_Serial0 & 0x00FF0000) >> 16);
    Joystick_StringSerial[8] = (uint8_t)((Device_Serial0 & 0xFF000000) >> 24);

    Joystick_StringSerial[10] = (uint8_t)(Device_Serial1 & 0x000000FF);
    Joystick_StringSerial[12] = (uint8_t)((Device_Serial1 & 0x0000FF00) >> 8);
    Joystick_StringSerial[14] = (uint8_t)((Device_Serial1 & 0x00FF0000) >> 16);
    Joystick_StringSerial[16] = (uint8_t)((Device_Serial1 & 0xFF000000) >> 24);

    Joystick_StringSerial[18] = (uint8_t)(Device_Serial2 & 0x000000FF);
    Joystick_StringSerial[20] = (uint8_t)((Device_Serial2 & 0x0000FF00) >> 8);
    Joystick_StringSerial[22] = (uint8_t)((Device_Serial2 & 0x00FF0000) >> 16);
    Joystick_StringSerial[24] = (uint8_t)((Device_Serial2 & 0xFF000000) >> 24);
  }
}

/******************* (C) COPYRIGHT 2009 STMicroelectronics *****END OF FILE****/
