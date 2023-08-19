/**
  ******************************************************************************
  * @file    otp_interface.h
  * @brief   Header for otp_interface.c module
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef OTP_INTERFACE_H
#define OTP_INTERFACE_H

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

/* Includes ------------------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
/* Exported functions --------------------------------------------------------*/
uint8_t OPENBL_OTP_Read(uint32_t Address);
void OPENBL_OTP_Write(uint32_t Address, uint8_t *pData, uint32_t DataLength);

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* OPTIONBYTES_INTERFACE_H */
