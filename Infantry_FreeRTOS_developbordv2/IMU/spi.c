#include "spi.h"
#include "InfantryConfig.h" 

void SPI5_Gpio_Config(void)
{
	GPIO_InitTypeDef  GPIO_InitStructure;
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOF, ENABLE);
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7|GPIO_Pin_9|GPIO_Pin_8;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOF, &GPIO_InitStructure);

	GPIO_PinAFConfig(GPIOF,GPIO_PinSource7,GPIO_AF_SPI5); 
	GPIO_PinAFConfig(GPIOF,GPIO_PinSource9,GPIO_AF_SPI5); 
	GPIO_PinAFConfig(GPIOF,GPIO_PinSource8,GPIO_AF_SPI5); 
}

void SPI5_Config(void)
{
	SPI_InitTypeDef  SPI_InitStructure;
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SPI5, ENABLE);//ʹ��SPI5ʱ��
	
	RCC_APB2PeriphResetCmd(RCC_APB2Periph_SPI5,ENABLE);//��λSPI5
	RCC_APB2PeriphResetCmd(RCC_APB2Periph_SPI5,DISABLE);//ֹͣ��λSPI5
	
	SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;  //����SPI�������˫�������ģʽ:SPI����Ϊ˫��˫��ȫ˫��
	SPI_InitStructure.SPI_Mode = SPI_Mode_Master;		//����SPI����ģʽ:����Ϊ��SPI
	SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;		//����SPI�����ݴ�С:SPI���ͽ���8λ֡�ṹ
	SPI_InitStructure.SPI_CPOL = SPI_CPOL_High;		//����ͬ��ʱ�ӵĿ���״̬Ϊ�ߵ�ƽ
	SPI_InitStructure.SPI_CPHA = SPI_CPHA_2Edge;	//����ͬ��ʱ�ӵĵڶ��������أ��������½������ݱ�����
	SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;		//NSS�ź���Ӳ����NSS�ܽţ����������ʹ��SSIλ������:�ڲ�NSS�ź���SSIλ����
	SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_256;		//���岨����Ԥ��Ƶ��ֵ:������Ԥ��ƵֵΪ256
	SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;	//ָ�����ݴ����MSBλ����LSBλ��ʼ:���ݴ����MSBλ��ʼ
	SPI_InitStructure.SPI_CRCPolynomial = 7;	//stm32f4xx.h 7881 7882
	SPI_Init(SPI5, &SPI_InitStructure);  //����SPI_InitStruct��ָ���Ĳ�����ʼ������SPIx�Ĵ���
 
	SPI_Cmd(SPI5, ENABLE); //ʹ��SPI����		 
}

void SPIConfig(void)
{
	SPI5_Gpio_Config();
	SPI5_Config();
}
  
void SPI5SetSpeedAndDataSize(uint16_t Speed, uint16_t DataSize)
{
    SPI5->CR1 &= 0xF7C7;
    SPI5->CR1 |= Speed;
    SPI5->CR1 |= DataSize;
    SPI5->CR1 |= 1 << 6; //ʹ��spi
} 


void SPI5_DMA_Init(uint32_t tx_buf, uint32_t rx_buf, uint16_t num)
{

    NVIC_InitTypeDef NVIC_InitStructure;
    DMA_InitTypeDef DMA_InitStructure;
    /* -------------- Enable Module Clock Source ----------------------------*/
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA2, ENABLE);

    NVIC_InitStructure.NVIC_IRQChannel = DMA2_Stream3_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = SPI5_RX_NVIC;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    DMA_DeInit(DMA2_Stream3);

    while (DMA_GetCmdStatus(DMA2_Stream3) != DISABLE)
    {
        ;
    }
    DMA_InitStructure.DMA_Channel = DMA_Channel_2;
    DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t) & (SPI5->DR);
    DMA_InitStructure.DMA_Memory0BaseAddr = rx_buf;
    DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory;
    DMA_InitStructure.DMA_BufferSize = num;
    DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
    DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
    DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
    DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
    DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
    DMA_InitStructure.DMA_Priority = DMA_Priority_VeryHigh;
    DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;
    DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_1QuarterFull;
    DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;
    DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
    DMA_Init(DMA2_Stream3, &DMA_InitStructure);

    DMA_ITConfig(DMA2_Stream3, DMA_IT_TC, ENABLE);

    DMA_Cmd(DMA2_Stream3, DISABLE); //Add a disable

    DMA_DeInit(DMA2_Stream4);

    while (DMA_GetCmdStatus(DMA2_Stream4) != DISABLE)
    {
        ;
    }
    DMA_InitStructure.DMA_Channel = DMA_Channel_2;
    DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t) & (SPI5->DR);
    DMA_InitStructure.DMA_Memory0BaseAddr = tx_buf;
    DMA_InitStructure.DMA_DIR = DMA_DIR_MemoryToPeripheral;
    DMA_InitStructure.DMA_BufferSize = num;
    DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
    DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
    DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
    DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
    DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
    DMA_InitStructure.DMA_Priority = DMA_Priority_Medium;
    DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;
    DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_1QuarterFull;
    DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;
    DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
    DMA_Init(DMA2_Stream4, &DMA_InitStructure);

    DMA_Cmd(DMA2_Stream4, DISABLE); //Add a disable
}

void SPI5_DMA_Enable(uint16_t ndtr)
{
    DMA_Cmd(DMA2_Stream3, DISABLE);
    DMA_Cmd(DMA2_Stream4, DISABLE);
    while (DMA_GetCmdStatus(DMA2_Stream4) != DISABLE)
    {
        ;
    }
    while (DMA_GetCmdStatus(DMA2_Stream3) != DISABLE)
    {
        ;
    }

    DMA_ClearFlag(DMA2_Stream3, DMA_FLAG_TCIF3 | DMA_FLAG_HTIF3);
    DMA_ClearFlag(DMA2_Stream4, DMA_FLAG_TCIF4 | DMA_FLAG_HTIF4);

    DMA_SetCurrDataCounter(DMA2_Stream4, ndtr);
    DMA_SetCurrDataCounter(DMA2_Stream3, ndtr);
    DMA_Cmd(DMA2_Stream3, ENABLE);
    DMA_Cmd(DMA2_Stream4, ENABLE);
}








