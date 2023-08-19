/**
  ******************************************************************************
  * @file    app_openbootloader.c
  * @author  MCD Application Team
  * @brief   OpenBootloader application entry point
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "main.h"

#include "openbl_core.h"
#include "openbl_mem.h"

#include "openbl_spi_cmd.h"

#include "app_openbootloader.h"
#include "spi_interface.h"
#include "flash_interface.h"
#include "ram_interface.h"
#include "optionbytes_interface.h"
#include "otp_interface.h"
//#include "engibytes_interface.h"
#include "systemmemory_interface.h"

#include "USART1Terminal.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
static OPENBL_HandleTypeDef SPI_Handle;

static OPENBL_OpsTypeDef SPI_Ops =
{
  NULL,	// OPENBL_SPI_Configuration,
  NULL,	// OPENBL_SPI_DeInit,
  OPENBL_SPI_ProtocolDetection,
  OPENBL_SPI_GetCommandOpcode,
  OPENBL_SPI_SendAcknowledgeByte
};

/* Exported variables --------------------------------------------------------*/
uint16_t SpecialCmdList[SPECIAL_CMD_MAX_NUMBER] =
{
  //SPECIAL_CMD_DEFAULT
  SPECIAL_CMD_0
};

uint16_t ExtendedSpecialCmdList[EXTENDED_SPECIAL_CMD_MAX_NUMBER] =
{
  //SPECIAL_CMD_DEFAULT
  SPECIAL_CMD_0
};

OPENBL_CommandsTypeDef SPI_Cmd =
{
  OPENBL_SPI_GetCommand,
  OPENBL_SPI_GetVersion,
  OPENBL_SPI_GetID,
  OPENBL_SPI_ReadMemory,
  OPENBL_SPI_WriteMemory,
  OPENBL_SPI_Go,
  NULL,
  NULL,
  OPENBL_SPI_EraseMemory,
  OPENBL_SPI_WriteProtect,
  OPENBL_SPI_WriteUnprotect,
  NULL,
  NULL,
  NULL,
  NULL,
  NULL,
  NULL,
  NULL,
  OPENBL_SPI_SpecialCommand,
  OPENBL_SPI_ExtendedSpecialCommand
};

/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/**
  * @brief  Initialize open Bootloader.
  * @param  None.
  * @retval None.
  */
void OpenBootloader_Init(void)
{
  /* Register SPI interfaces */
  SPI_Handle.p_Ops = &SPI_Ops;
  SPI_Handle.p_Cmd = OPENBL_SPI_GetCommandsList();
  OPENBL_SPI_SetCommandsList(&SPI_Cmd);

  OPENBL_RegisterInterface(&SPI_Handle);

  /* Initialize interfaces */
  OPENBL_Init();

  /* Initialize memories */
  OPENBL_MEM_RegisterMemory(&FLASH_Descriptor);
  OPENBL_MEM_RegisterMemory(&RAM_Descriptor);
  OPENBL_MEM_RegisterMemory(&OB_Descriptor);
  OPENBL_MEM_RegisterMemory(&OTP_Descriptor);
  OPENBL_MEM_RegisterMemory(&ICP_Descriptor);
  //OPENBL_MEM_RegisterMemory(&EB_Descriptor);
}

/**
  * @brief  DeInitialize open Bootloader.
  * @param  None.
  * @retval None.
  */
void OpenBootloader_DeInit(void)
{
  System_DeInit();
}

/**
  * @brief  This function is used to select which protocol will be used when communicating with the host.
  * @param  None.
  * @retval None.
  */
void OpenBootloader_ProtocolDetection(void)
{
  static uint32_t interface_detected = 0U;

  if (interface_detected == 0U)
  {
    interface_detected = OPENBL_InterfaceDetection();

    /* De-initialize the interfaces that are not detected */
    if (interface_detected == 1U)
    {
      OPENBL_InterfacesDeInit();
      PRINTF("ERROR: Interfaces not detected\n\r");
    }
  }

  if (interface_detected == 1U)
  {
    OPENBL_CommandProcess();
    PRINTF("Interface detected\n\r");
  }
}
