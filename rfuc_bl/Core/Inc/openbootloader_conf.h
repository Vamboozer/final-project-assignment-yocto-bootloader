/**
  ******************************************************************************
  * @file    openbootloader_conf.h
  * @brief   Contains Open Bootloader configuration
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef OPENBOOTLOADER_CONF_H
#define OPENBOOTLOADER_CONF_H

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

/* Includes ------------------------------------------------------------------*/
#include "platform.h"

/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/

/* -------------------------------- Device ID ------------------------------- */
// The device ID for the STM32L4Rxxx and STM32L4Sxxx devices is: 0x470
#define DEVICE_ID_MSB                     0x04U  /* MSB byte of device ID */
#define DEVICE_ID_LSB                     0x70U  /* LSB byte of device ID */

/*--------------------------- Definitions for Memories ---------------------- */
#define FLASH_MEM_SIZE                    (2048U * 1024U)                 /* Size of Flash 2 MByte */
#define FLASH_START_ADDRESS               0x08000000U                     /* Flash start address */
#define FLASH_END_ADDRESS                 (FLASH_BASE + FLASH_MEM_SIZE)   /* Flash end address */

#define RAM_SIZE                          (640U * 1024U)                  /* Size of RAM 640 kByte */
#define RAM_START_ADDRESS                 0x20000000U                     /* SRAM start address  */
#define RAM_END_ADDRESS                   (RAM_START_ADDRESS + RAM_SIZE)  /* SRAM end address */

#define OB_SIZE                           4111U                           /* Size of OB 4111 Byte */
#define OB_START_ADDRESS                  0x1FF00000U                     /* Option bytes registers address */
#define OB1_START_ADDRESS                 OB_START_ADDRESS                /* Option Bytes 1 start address */
#define OB2_START_ADDRESS                 0x1FF01000U                     /* Option Bytes 2 start address */
#define OB_END_ADDRESS                    (OB_START_ADDRESS + OB_SIZE)    /* Option bytes end address */

#define OTP_SIZE                          (1U * 1024U)                    /* Size of OTP 1 kByte */
#define OTP_START_ADDRESS                 0x1FFF7000U                     /* OTP start address */
#define OTP_END_ADDRESS                   (OTP_START_ADDRESS + OTP_SIZE)  /* OTP end address */

// System Memory - Bank 1
#define ICP_SIZE                          (28U * 1024U)                   /* Size of ICP 28 kByte */
#define ICP_START_ADDRESS                 0x1FFF0000U                     /* System memory start address */
#define ICP_END_ADDRESS                   (ICP_START_ADDRESS + ICP_SIZE)  /* System memory end address */

//#define EB_SIZE                           156U                            /* Size of Engi bytes 156 Byte */
//#define EB_START_ADDRESS                  0x40022400U                     /* Engi bytes start address */
//#define EB_END_ADDRESS                    (EB_START_ADDRESS + EB_SIZE)    /* Engi bytes end address */

#define OPENBL_RAM_SIZE                   0x16800U             /* RAM used by the Open Bootloader 88 kBytes */

#define OPENBL_DEFAULT_MEM                FLASH_START_ADDRESS  /* Used for Erase and Write protect CMDs */

#define RDP_LEVEL_0                       0xEEEEEEEEU
#define RDP_LEVEL_1                       0xEEEEEEEEU
#define RDP_LEVEL_2                       0xEEEEEEEEU

#define AREA_ERROR                        0x0U  /* Error Address Area */
#define FLASH_AREA                        0x1U  /* Flash Address Area */
#define RAM_AREA                          0x2U  /* RAM Address area */
#define OB_AREA                           0x3U  /* Option bytes Address area */
#define OTP_AREA                          0x4U  /* OTP Address area */
#define ICP_AREA                          0x5U  /* System memory area */
#define EB_AREA                           0x6U  /* Engi bytes Address area */

#define FLASH_MASS_ERASE                  0xFFFFU
#define FLASH_BANK1_ERASE                 0xFFFEU
#define FLASH_BANK2_ERASE                 0xFFFDU

#define INTERFACES_SUPPORTED              1U

/* Exported macro ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* OPENBOOTLOADER_CONF_H */
