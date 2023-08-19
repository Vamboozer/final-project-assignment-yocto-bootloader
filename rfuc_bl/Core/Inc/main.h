/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32l4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include "cmsis_os.h"

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

typedef struct {
	char msg[255];
} UartTxStruct;

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

extern osThreadId_t CommandProcessTaskHandle;
extern osMessageQueueId_t UartTxQueueHandle;
extern UART_HandleTypeDef huart1;
extern SPI_HandleTypeDef hspi1;

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

void System_DeInit(void);

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define GPO_TP17_Pin GPIO_PIN_2
#define GPO_TP17_GPIO_Port GPIOH
#define GPO_SBANDRX_Pin GPIO_PIN_0
#define GPO_SBANDRX_GPIO_Port GPIOE
#define SPI1MISO_SOM_Pin GPIO_PIN_4
#define SPI1MISO_SOM_GPIO_Port GPIOB
#define SPI1SCK_SOM_Pin GPIO_PIN_3
#define SPI1SCK_SOM_GPIO_Port GPIOB
#define SPI1CS_SOM_Pin GPIO_PIN_15
#define SPI1CS_SOM_GPIO_Port GPIOA
#define SWDCLK_Pin GPIO_PIN_14
#define SWDCLK_GPIO_Port GPIOA
#define SWDIO_Pin GPIO_PIN_13
#define SWDIO_GPIO_Port GPIOA
#define SPI2_NSS_IqDemod_Pin GPIO_PIN_0
#define SPI2_NSS_IqDemod_GPIO_Port GPIOI
#define GPIO2_Pin GPIO_PIN_14
#define GPIO2_GPIO_Port GPIOH
#define GPO_TP14_Pin GPIO_PIN_9
#define GPO_TP14_GPIO_Port GPIOI
#define GPO_TP15_Pin GPIO_PIN_7
#define GPO_TP15_GPIO_Port GPIOI
#define GPO_DEN_Pin GPIO_PIN_1
#define GPO_DEN_GPIO_Port GPIOE
#define SPI1MOSI_SOM_Pin GPIO_PIN_5
#define SPI1MOSI_SOM_GPIO_Port GPIOB
#define SPI3SCK_AR_Pin GPIO_PIN_9
#define SPI3SCK_AR_GPIO_Port GPIOG
#define SPI2MISO_RxIq_Pin GPIO_PIN_2
#define SPI2MISO_RxIq_GPIO_Port GPIOI
#define SPI2SCK_RxIq_Pin GPIO_PIN_1
#define SPI2SCK_RxIq_GPIO_Port GPIOI
#define GPIO3_Pin GPIO_PIN_15
#define GPIO3_GPIO_Port GPIOH
#define GPI_MPAEN_Pin GPIO_PIN_12
#define GPI_MPAEN_GPIO_Port GPIOH
#define GPI_MPAEN_EXTI_IRQn EXTI15_10_IRQn
#define GPO_MEN_Pin GPIO_PIN_6
#define GPO_MEN_GPIO_Port GPIOB
#define SPI2MOSI_RxIq_Pin GPIO_PIN_4
#define SPI2MOSI_RxIq_GPIO_Port GPIOD
#define GPO_RXLOEN_Pin GPIO_PIN_4
#define GPO_RXLOEN_GPIO_Port GPIOE
#define SPI2_NSS_RxSynth_Pin GPIO_PIN_3
#define SPI2_NSS_RxSynth_GPIO_Port GPIOE
#define SPI2_NSS_RxDAC_Pin GPIO_PIN_2
#define SPI2_NSS_RxDAC_GPIO_Port GPIOE
#define GPO_SBANDTX_Pin GPIO_PIN_7
#define GPO_SBANDTX_GPIO_Port GPIOB
#define SPI3MISO_AR_Pin GPIO_PIN_10
#define SPI3MISO_AR_GPIO_Port GPIOG
#define SPI3_NSS_TxDAC_Pin GPIO_PIN_10
#define SPI3_NSS_TxDAC_GPIO_Port GPIOC
#define PURX_Pin GPIO_PIN_10
#define PURX_GPIO_Port GPIOA
#define GPIO1_Pin GPIO_PIN_12
#define GPIO1_GPIO_Port GPIOA
#define GPI_DO_NOT_USE_Pin GPIO_PIN_13
#define GPI_DO_NOT_USE_GPIO_Port GPIOC
#define GPO_RXPDEN_Pin GPIO_PIN_6
#define GPO_RXPDEN_GPIO_Port GPIOE
#define GPO_RXLNASD_Pin GPIO_PIN_5
#define GPO_RXLNASD_GPIO_Port GPIOE
#define SPI3MOSI_AR_Pin GPIO_PIN_11
#define SPI3MOSI_AR_GPIO_Port GPIOG
#define SPI3_NSS_TxSynth_Pin GPIO_PIN_3
#define SPI3_NSS_TxSynth_GPIO_Port GPIOD
#define PUTX_Pin GPIO_PIN_9
#define PUTX_GPIO_Port GPIOA
#define GPIO0_Pin GPIO_PIN_11
#define GPIO0_GPIO_Port GPIOA
#define SPI3_NSS_AR_Pin GPIO_PIN_12
#define SPI3_NSS_AR_GPIO_Port GPIOG
#define I2C3SDA_EEPROM_Pin GPIO_PIN_8
#define I2C3SDA_EEPROM_GPIO_Port GPIOG
#define GPO_TXLOEN_Pin GPIO_PIN_6
#define GPO_TXLOEN_GPIO_Port GPIOC
#define GPO_TPX_Pin GPIO_PIN_5
#define GPO_TPX_GPIO_Port GPIOG
#define GPO_STXDEN_Pin GPIO_PIN_7
#define GPO_STXDEN_GPIO_Port GPIOC
#define GPO_TP16_Pin GPIO_PIN_6
#define GPO_TP16_GPIO_Port GPIOG
#define GPO_CTXDEN_Pin GPIO_PIN_9
#define GPO_CTXDEN_GPIO_Port GPIOC
#define GPO_PHVRXEN_Pin GPIO_PIN_10
#define GPO_PHVRXEN_GPIO_Port GPIOE
#define DMDTMP_Pin GPIO_PIN_0
#define DMDTMP_GPIO_Port GPIOC
#define P28VPA_Pin GPIO_PIN_2
#define P28VPA_GPIO_Port GPIOC
#define GPO_HEATON_Pin GPIO_PIN_9
#define GPO_HEATON_GPIO_Port GPIOE
#define GPO_RSEL2n_Pin GPIO_PIN_15
#define GPO_RSEL2n_GPIO_Port GPIOE
#define TXRPWR_Pin GPIO_PIN_0
#define TXRPWR_GPIO_Port GPIOA
#define GPO_RSEL3n_Pin GPIO_PIN_8
#define GPO_RSEL3n_GPIO_Port GPIOE
#define GPO_RSEL1n_Pin GPIO_PIN_14
#define GPO_RSEL1n_GPIO_Port GPIOE
#define TVCA_Pin GPIO_PIN_5
#define TVCA_GPIO_Port GPIOA
#define GPO_RSEL0n_Pin GPIO_PIN_7
#define GPO_RSEL0n_GPIO_Port GPIOE
#define GPO_PHVTXEN_Pin GPIO_PIN_15
#define GPO_PHVTXEN_GPIO_Port GPIOB
#define GPO_LPBK1_Pin GPIO_PIN_12
#define GPO_LPBK1_GPIO_Port GPIOE
#define GPO_TSEL0n_Pin GPIO_PIN_14
#define GPO_TSEL0n_GPIO_Port GPIOB
#define CRTNSENSE_Pin GPIO_PIN_2
#define CRTNSENSE_GPIO_Port GPIOA
#define RVCA0_Pin GPIO_PIN_4
#define RVCA0_GPIO_Port GPIOA
#define GPO_LPBK0_Pin GPIO_PIN_11
#define GPO_LPBK0_GPIO_Port GPIOE
#define GPO_TXPDEN_Pin GPIO_PIN_10
#define GPO_TXPDEN_GPIO_Port GPIOB
#define GPO_TSEL2n_Pin GPIO_PIN_12
#define GPO_TSEL2n_GPIO_Port GPIOB
#define GPO_TSEL1n_Pin GPIO_PIN_13
#define GPO_TSEL1n_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
