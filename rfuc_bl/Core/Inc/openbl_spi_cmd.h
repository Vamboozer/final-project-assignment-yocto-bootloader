/**
  ******************************************************************************
  * @file    openbl_spi_cmd.h
  * @brief   Header for openbl_spi_cmd.c module
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef OPENBL_SPI_CMD_H
#define OPENBL_SPI_CMD_H

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

/* Includes ------------------------------------------------------------------*/
#include "openbl_core.h"

/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
// See table 3 of AN4286 for explaination of SPI Protocol Versions
//#define OPENBL_SPI_VERSION                 0x11U  /* Open Bootloader SPI protocol V1.1 */
#define OPENBL_SPI_VERSION                 0x20U  /* Open Bootloader SPI protocol V2.0 */

/* Exported macro ------------------------------------------------------------*/
/* Exported variables --------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */
OPENBL_CommandsTypeDef *OPENBL_SPI_GetCommandsList(void);
void OPENBL_SPI_SetCommandsList(OPENBL_CommandsTypeDef *pSpiCmd);
void OPENBL_SPI_GetCommand(void);
void OPENBL_SPI_GetVersion(void);
void OPENBL_SPI_GetID(void);
void OPENBL_SPI_ReadMemory(void);
void OPENBL_SPI_WriteMemory(void);
void OPENBL_SPI_Go(void);
void OPENBL_SPI_ReadoutProtect(void);
void OPENBL_SPI_ReadoutUnprotect(void);
void OPENBL_SPI_EraseMemory(void);
void OPENBL_SPI_WriteProtect(void);
void OPENBL_SPI_WriteUnprotect(void);
void OPENBL_SPI_SpecialCommand(void);
void OPENBL_SPI_ExtendedSpecialCommand(void);

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* OPENBL_SPI_CMD_H */
