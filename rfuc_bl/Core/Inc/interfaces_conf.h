/**
  ******************************************************************************
  * @file    interfaces_conf.h
  * @brief   Contains Interfaces configuration
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef INTERFACES_CONF_H
#define INTERFACES_CONF_H

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

/* Includes ------------------------------------------------------------------*/
//#include "stm32h5xx_ll_spi.h"
//#include "stm32h5xx_ll_gpio.h"
//#include "stm32h5xx_ll_rcc.h"
#include "stm32l4xx_hal.h"

//#define MEMORIES_SUPPORTED                7U
#define MEMORIES_SUPPORTED                6U // until I figure out where engi bytes are located on MCU

/*--------------------------- Definitions for SPI ----------------------------*/
#define SPIx                              SPI1
#define SPIx_CLK_ENABLE()                 __HAL_RCC_SPI1_CLK_ENABLE()
#define SPIx_CLK_DISABLE()                __HAL_RCC_SPI1_CLK_DISABLE()
#define SPIx_GPIO_CLK_SCK_ENABLE()        __HAL_RCC_GPIOH_CLK_ENABLE()
#define SPIx_GPIO_CLK_MISO_ENABLE()       __HAL_RCC_GPIOH_CLK_ENABLE()
#define SPIx_GPIO_CLK_MOSI_ENABLE()       __HAL_RCC_GPIOH_CLK_ENABLE()
#define SPIx_GPIO_CLK_NSS_ENABLE()        __HAL_RCC_GPIOH_CLK_ENABLE()
#define SPIx_IRQ                          SPI1_IRQn

#define SPIx_MOSI_PIN                     SPI1MOSI_SOM_Pin
#define SPIx_MOSI_PIN_PORT                SPI1MOSI_SOM_GPIO_Port
#define SPIx_MISO_PIN                     SPI1MISO_SOM_Pin
#define SPIx_MISO_PIN_PORT                SPI1MISO_SOM_GPIO_Port
#define SPIx_SCK_PIN                      SPI1SCK_SOM_Pin
#define SPIx_SCK_PIN_PORT                 SPI1SCK_SOM_GPIO_Port
#define SPIx_NSS_PIN                      SPI1CS_SOM_Pin
#define SPIx_NSS_PIN_PORT                 SPI1CS_SOM_GPIO_Port
#define SPIx_ALTERNATE                    GPIO_AF5_SPI1


#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* INTERFACES_CONF_H */
