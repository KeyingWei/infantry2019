#ifndef __SPI_H
#define __SPI_H

#include <stm32f4xx.h>
 	    													  
void SPIConfig(void);
void SPI5_SetSpeed(u8 SpeedSet); //����SPI1�ٶ�   
u8 SPI5_ReadWriteByte(u8 TxData);
		 
#endif

