#include "app.h"

#define ADC1_DR_Address ((uint32_t)0x4001244C)//定义硬件ADC1的物理地址
__IO uint16_t ADCConvertedValue[6];//转换的6通道AD值


/**********************************************************
** 函数名: ADC1_DMA_Config
** 功能描述: ADC1的DMA方式配置
** 输入参数: 无
** 输出参数: 无
***********************************************************/
void ADC1_DMA_Config(void)
{
  ADC_InitTypeDef ADC_InitStructure;//定义ADC结构体
  DMA_InitTypeDef DMA_InitStructure;//定义DMA结构体  
  GPIO_InitTypeDef GPIO_InitStructure;

  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);//使能DMA1时钟
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1| RCC_APB2Periph_GPIOA, ENABLE ); //使能ADC1及GPIOA时钟
  /*作为ADC1的6通道模拟输入的GPIO初始化配置*/
  //PA2,3,4,5,6,7配置为模拟输入
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2|GPIO_Pin_3|GPIO_Pin_4|GPIO_Pin_5|GPIO_Pin_6|GPIO_Pin_7;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;//模拟输入
  GPIO_Init(GPIOA, &GPIO_InitStructure);

  /*DMA1的通道1配置*/
  DMA_DeInit(DMA1_Channel1);
  DMA_InitStructure.DMA_PeripheralBaseAddr = ADC1_DR_Address;//传输的源头地址
  DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)&ADCConvertedValue;//目标地址
  DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC; //外设作源头
  DMA_InitStructure.DMA_BufferSize = 6;//数据长度为6
  DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;//外设地址寄存器不递增
  DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;//内存地址递增
  DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;//外设传输以字节为单位
  DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;//内存以字节为单位
  DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;//循环模式
  DMA_InitStructure.DMA_Priority = DMA_Priority_High;//4优先级之一的(高优先)
  DMA_InitStructure.DMA_M2M = DMA_M2M_Disable; //非内存到内存
  DMA_Init(DMA1_Channel1, &DMA_InitStructure);//根据以上参数初始化DMA_InitStructure

  DMA_ITConfig(DMA1_Channel1, DMA_IT_TC, ENABLE);//配置DMA1通道1传输完成中断 
  DMA_Cmd(DMA1_Channel1, ENABLE);//使能DMA1
  
  /*下面为ADC1的配置*/
  ADC_InitStructure.ADC_Mode = ADC_Mode_Independent;//ADC1工作在独立模式
  ADC_InitStructure.ADC_ScanConvMode = ENABLE;//模数转换工作在扫描模式（多通道）
  ADC_InitStructure.ADC_ContinuousConvMode = ENABLE;//模数转换工作在连续模式
  ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;//转换由软件而不是外部触发启动
  ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;//ADC数据右对齐
  ADC_InitStructure.ADC_NbrOfChannel = 6;//转换的ADC通道的数目为6
  ADC_Init(ADC1, &ADC_InitStructure);//要把以下参数初始化ADC_InitStructure

  /* 设置ADC1的6个规则组通道，设置它们的转化顺序和采样时间*/ 
  //转换时间Tconv=采样时间+12.5个周期
  ADC_RegularChannelConfig(ADC1, ADC_Channel_2, 1, ADC_SampleTime_7Cycles5); //ADC1通道2转换顺序为1，采样时间为7.5个周期 
  ADC_RegularChannelConfig(ADC1, ADC_Channel_3, 2, ADC_SampleTime_55Cycles5);//ADC1通道3转换顺序为2，采样时间为55.5个周期 
  ADC_RegularChannelConfig(ADC1, ADC_Channel_4, 3, ADC_SampleTime_55Cycles5);//ADC1通道4  
  ADC_RegularChannelConfig(ADC1, ADC_Channel_5, 4, ADC_SampleTime_55Cycles5);//ADC1通道5 
  ADC_RegularChannelConfig(ADC1, ADC_Channel_6, 5, ADC_SampleTime_55Cycles5);//ADC1通道6 
  ADC_RegularChannelConfig(ADC1, ADC_Channel_7, 6, ADC_SampleTime_55Cycles5);//ADC1通道7 
  /*使能ADC1的DMA传输方式*/
  ADC_DMACmd(ADC1, ENABLE);
  /*使能ADC1 */
  ADC_Cmd(ADC1, ENABLE);
  /*重置ADC1的校准寄存器 */   
  ADC_ResetCalibration(ADC1);
  /*获取ADC重置校准寄存器的状态*/
  while(ADC_GetResetCalibrationStatus(ADC1));
  ADC_StartCalibration(ADC1); /*开始校准ADC1*/
  while(ADC_GetCalibrationStatus(ADC1)); //等待校准完成
  ADC_SoftwareStartConvCmd(ADC1, ENABLE);//使能ADC1软件转换
}

/**********************************************************
** 函数名: void NVIC_Config(void)
** 功能描述:  中断分组及优先级配置
** 输入参数: 无
** 输出参数: 无
***********************************************************/
void NVIC_Config(void)
{
	NVIC_InitTypeDef NVIC_InitStructure;
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);  //采用组别2
	/*DMA1的通道1中断配置*/
	NVIC_InitStructure.NVIC_IRQChannel = DMA1_Channel1_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;//占先式优先级设置为0
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;//副优先级设置为1
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;//中断使能
	NVIC_Init(&NVIC_InitStructure);//按指定参数初始化中断	 
}

/**********************************************************
** 函数名: u8 All_Init(void)
** 功能描述: 系统的所有初始化配置放在这里
** 输入参数: 无
** 输出参数: 初始化成功返回1，否则返回0
***********************************************************/
u8 All_Init(void)
{
//	SystemInit(); //系统时钟初始化
	delay_init(72);//初始化SysTick延时
	NVIC_Config();//中断分组及优先级配置
	Usart_Configuration();//串口初始化，波特率115200
	ADC1_DMA_Config(); //ADC1的DMA初始化
	return 1;
}
