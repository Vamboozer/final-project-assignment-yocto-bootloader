/**
  ******************************************************************************
  * @file    systemmemory_interface.h
  * @brief   Header for systemmemory_interface.c module
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef SYSTEMMEMORY_INTERFACE_H
#define SYSTEMMEMORY_INTERFACE_H

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

/* Includes ------------------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
/* Exported functions --------------------------------------------------------*/
uint8_t OPENBL_ICP_Read(uint32_t Address);

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* SYSTEMMEMORY_INTERFACE_H */
