/**
  ******************************************************************************
  * @file    openbl_mem.h
  * @brief   Header for openbl_mem.c module
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef OPENBL_MEM_H
#define OPENBL_MEM_H

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

/* Includes ------------------------------------------------------------------*/
#include "openbootloader_conf.h"

/* Exported types ------------------------------------------------------------*/
typedef struct
{
  uint32_t StartAddress;
  uint32_t EndAddress;
  uint32_t Size;
  uint32_t Type;
  uint8_t (*Read)(uint32_t Address);
  void (*Write)(uint32_t Address, uint8_t *Data, uint32_t DataLength);
  void (*SetReadoutProtect)(uint32_t State);
  ErrorStatus(*SetWriteProtect)(FunctionalState State, uint32_t OB_WrpArea);
  void (*JumpToAddress)(uint32_t Address);
  ErrorStatus(*MassErase)(uint8_t *p_Data, uint32_t DataLength);
  ErrorStatus(*Erase)(uint8_t *p_Data, uint32_t DataLength);
} OPENBL_MemoryTypeDef;

/* Exported constants --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */
void OPENBL_MEM_JumpToAddress(uint32_t Address);
void OPENBL_MEM_SetReadOutProtection(uint32_t Address, FunctionalState State);
void OPENBL_MEM_Write(uint32_t Address, uint8_t *Data, uint32_t DataLength);

uint8_t OPENBL_MEM_Read(uint32_t Address, uint32_t MemoryIndex);
uint32_t OPENBL_MEM_GetAddressArea(uint32_t Address);
uint32_t OPENBL_MEM_GetMemoryIndex(uint32_t Address);
uint8_t OPENBL_MEM_CheckJumpAddress(uint32_t Address);

ErrorStatus OPENBL_MEM_Erase(uint32_t Address, uint8_t *p_Data, uint32_t DataLength);
ErrorStatus OPENBL_MEM_MassErase(uint32_t Address, uint8_t *p_Data, uint32_t DataLength);
ErrorStatus OPENBL_MEM_RegisterMemory(OPENBL_MemoryTypeDef *Memory);
ErrorStatus OPENBL_MEM_SetWriteProtection(FunctionalState State, uint32_t Address, uint32_t OB_WrpArea);

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* OPENBL_MEM_H */
