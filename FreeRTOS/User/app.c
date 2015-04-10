#include "app.h"

#define ADC1_DR_Address ((uint32_t)0x4001244C)//����Ӳ��ADC1�������ַ
__IO uint16_t ADCConvertedValue[6];//ת����6ͨ��ADֵ


/**********************************************************
** ������: ADC1_DMA_Config
** ��������: ADC1��DMA��ʽ����
** �������: ��
** �������: ��
***********************************************************/
void ADC1_DMA_Config(void)
{
  ADC_InitTypeDef ADC_InitStructure;//����ADC�ṹ��
  DMA_InitTypeDef DMA_InitStructure;//����DMA�ṹ��  
  GPIO_InitTypeDef GPIO_InitStructure;

  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);//ʹ��DMA1ʱ��
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1| RCC_APB2Periph_GPIOA, ENABLE ); //ʹ��ADC1��GPIOAʱ��
  /*��ΪADC1��6ͨ��ģ�������GPIO��ʼ������*/
  //PA2,3,4,5,6,7����Ϊģ������
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2|GPIO_Pin_3|GPIO_Pin_4|GPIO_Pin_5|GPIO_Pin_6|GPIO_Pin_7;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;//ģ������
  GPIO_Init(GPIOA, &GPIO_InitStructure);

  /*DMA1��ͨ��1����*/
  DMA_DeInit(DMA1_Channel1);
  DMA_InitStructure.DMA_PeripheralBaseAddr = ADC1_DR_Address;//�����Դͷ��ַ
  DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)&ADCConvertedValue;//Ŀ���ַ
  DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC; //������Դͷ
  DMA_InitStructure.DMA_BufferSize = 6;//���ݳ���Ϊ6
  DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;//�����ַ�Ĵ���������
  DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;//�ڴ��ַ����
  DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;//���贫�����ֽ�Ϊ��λ
  DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;//�ڴ����ֽ�Ϊ��λ
  DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;//ѭ��ģʽ
  DMA_InitStructure.DMA_Priority = DMA_Priority_High;//4���ȼ�֮һ��(������)
  DMA_InitStructure.DMA_M2M = DMA_M2M_Disable; //���ڴ浽�ڴ�
  DMA_Init(DMA1_Channel1, &DMA_InitStructure);//�������ϲ�����ʼ��DMA_InitStructure

  DMA_ITConfig(DMA1_Channel1, DMA_IT_TC, ENABLE);//����DMA1ͨ��1��������ж� 
  DMA_Cmd(DMA1_Channel1, ENABLE);//ʹ��DMA1
  
  /*����ΪADC1������*/
  ADC_InitStructure.ADC_Mode = ADC_Mode_Independent;//ADC1�����ڶ���ģʽ
  ADC_InitStructure.ADC_ScanConvMode = ENABLE;//ģ��ת��������ɨ��ģʽ����ͨ����
  ADC_InitStructure.ADC_ContinuousConvMode = ENABLE;//ģ��ת������������ģʽ
  ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;//ת��������������ⲿ��������
  ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;//ADC�����Ҷ���
  ADC_InitStructure.ADC_NbrOfChannel = 6;//ת����ADCͨ������ĿΪ6
  ADC_Init(ADC1, &ADC_InitStructure);//Ҫ�����²�����ʼ��ADC_InitStructure

  /* ����ADC1��6��������ͨ�����������ǵ�ת��˳��Ͳ���ʱ��*/ 
  //ת��ʱ��Tconv=����ʱ��+12.5������
  ADC_RegularChannelConfig(ADC1, ADC_Channel_2, 1, ADC_SampleTime_7Cycles5); //ADC1ͨ��2ת��˳��Ϊ1������ʱ��Ϊ7.5������ 
  ADC_RegularChannelConfig(ADC1, ADC_Channel_3, 2, ADC_SampleTime_55Cycles5);//ADC1ͨ��3ת��˳��Ϊ2������ʱ��Ϊ55.5������ 
  ADC_RegularChannelConfig(ADC1, ADC_Channel_4, 3, ADC_SampleTime_55Cycles5);//ADC1ͨ��4  
  ADC_RegularChannelConfig(ADC1, ADC_Channel_5, 4, ADC_SampleTime_55Cycles5);//ADC1ͨ��5 
  ADC_RegularChannelConfig(ADC1, ADC_Channel_6, 5, ADC_SampleTime_55Cycles5);//ADC1ͨ��6 
  ADC_RegularChannelConfig(ADC1, ADC_Channel_7, 6, ADC_SampleTime_55Cycles5);//ADC1ͨ��7 
  /*ʹ��ADC1��DMA���䷽ʽ*/
  ADC_DMACmd(ADC1, ENABLE);
  /*ʹ��ADC1 */
  ADC_Cmd(ADC1, ENABLE);
  /*����ADC1��У׼�Ĵ��� */   
  ADC_ResetCalibration(ADC1);
  /*��ȡADC����У׼�Ĵ�����״̬*/
  while(ADC_GetResetCalibrationStatus(ADC1));
  ADC_StartCalibration(ADC1); /*��ʼУ׼ADC1*/
  while(ADC_GetCalibrationStatus(ADC1)); //�ȴ�У׼���
  ADC_SoftwareStartConvCmd(ADC1, ENABLE);//ʹ��ADC1���ת��
}

/**********************************************************
** ������: void NVIC_Config(void)
** ��������:  �жϷ��鼰���ȼ�����
** �������: ��
** �������: ��
***********************************************************/
void NVIC_Config(void)
{
	NVIC_InitTypeDef NVIC_InitStructure;
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);  //�������2
	/*DMA1��ͨ��1�ж�����*/
	NVIC_InitStructure.NVIC_IRQChannel = DMA1_Channel1_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;//ռ��ʽ���ȼ�����Ϊ0
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;//�����ȼ�����Ϊ1
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;//�ж�ʹ��
	NVIC_Init(&NVIC_InitStructure);//��ָ��������ʼ���ж�	 
}

/**********************************************************
** ������: u8 All_Init(void)
** ��������: ϵͳ�����г�ʼ�����÷�������
** �������: ��
** �������: ��ʼ���ɹ�����1�����򷵻�0
***********************************************************/
u8 All_Init(void)
{
//	SystemInit(); //ϵͳʱ�ӳ�ʼ��
	delay_init(72);//��ʼ��SysTick��ʱ
	NVIC_Config();//�жϷ��鼰���ȼ�����
	Usart_Configuration();//���ڳ�ʼ����������115200
	ADC1_DMA_Config(); //ADC1��DMA��ʼ��
	return 1;
}
