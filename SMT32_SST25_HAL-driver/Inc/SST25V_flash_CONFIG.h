#ifndef __SST25V_FLASH_CONFIG_H
#define __SST25V_FLASH_CONFIG_H

#include "stm32f4xx_hal.h"

#define _SST25_USE_FREERTOS          0
#define _SST25_DEBUG                 0

extern SPI_HandleTypeDef hspi1;
#define SST25_hspi				hspi1

#define	FLASH_SS_Pin			GPIO_PIN_6
#define	FLASH_SS_GPIO_Port		GPIOB

#define FLASH_MISO_GPIO_Port	GPIOA
#define FLASH_MISO_Pin			GPIO_PIN_6
#endif
