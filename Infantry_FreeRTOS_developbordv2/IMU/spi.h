#ifndef __SPI_H
#define __SPI_H

#include <stm32f4xx.h>
 	    													  
void SPIConfig(void);
void SPI5SetSpeedAndDataSize(uint16_t Speed, uint16_t DataSize);
void SPI5_DMA_Init(uint32_t tx_buf, uint32_t rx_buf, uint16_t num);
void SPI5_DMA_Enable(uint16_t ndtr);
		 
#endif

