/**
  ******************************************************************************
  * @file    ram_interface.h
  * @brief   Header for ram_interface.c module
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef RAM_INTERFACE_H
#define RAM_INTERFACE_H

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

/* Includes ------------------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
/* Exported functions --------------------------------------------------------*/
void OPENBL_RAM_JumpToAddress(uint32_t Address);
uint8_t OPENBL_RAM_Read(uint32_t Address);
void OPENBL_RAM_Write(uint32_t Address, uint8_t *pData, uint32_t DataLength);

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* RAM_INTERFACE_H */
