/**
  ******************************************************************************
  * @file    app_openbootloader.h
  * @brief   Header for app_openbootloader.c module
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef APP_OPENBOOTLOADER_H
#define APP_OPENBOOTLOADER_H

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

/* Includes ------------------------------------------------------------------*/
#include "openbl_mem.h"

/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
#define SPECIAL_CMD_MAX_NUMBER            0x03U    /* Special command max length array */
#define EXTENDED_SPECIAL_CMD_MAX_NUMBER   0x01U    /* Extended special command max length array */
#define SPECIAL_CMD_DEFAULT               0x0102U  /* Default special command */
#define SPECIAL_CMD_GET_RFUC_VERSION	  0x0001U
#define SPECIAL_CMD_RFUC_VERSION_SYNC	  0x0002U

/* Exported macro ------------------------------------------------------------*/
/* Exported functions --------------------------------------------------------*/
void OpenBootloader_Init(void);
void OpenBootloader_ProtocolDetection(void);

/* External variables --------------------------------------------------------*/
extern OPENBL_MemoryTypeDef FLASH_Descriptor;
extern OPENBL_MemoryTypeDef RAM_Descriptor;
extern OPENBL_MemoryTypeDef OB_Descriptor;
extern OPENBL_MemoryTypeDef OTP_Descriptor;
extern OPENBL_MemoryTypeDef ICP_Descriptor;
//extern OPENBL_MemoryTypeDef EB_Descriptor;

extern uint16_t SpecialCmdList[SPECIAL_CMD_MAX_NUMBER];
extern uint16_t ExtendedSpecialCmdList[EXTENDED_SPECIAL_CMD_MAX_NUMBER];

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* APP_OPENBOOTLOADER_H */
