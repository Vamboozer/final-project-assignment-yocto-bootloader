/**
  ******************************************************************************
  * @file    flash_interface.c
  * @author  MCD Application Team
  * @brief   Contains FLASH access functions
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
#include "platform.h"
#include "common_interface.h"
#include "openbl_mem.h"
#include "app_openbootloader.h"
#include "flash_interface.h"
#include "optionbytes_interface.h"
#include "main.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
#define FLASH_PAGE_NUMBER              ((uint16_t)256U)
#define OPENBL_FLASH_TIMEOUT_VALUE       0x00000FFFU
#define FLASH_PROG_STEP_SIZE             ((uint8_t)1U)

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
static uint32_t Flash_BusyState          = FLASH_BUSY_STATE_DISABLED;
/*static FLASH_ProcessTypeDef FlashProcess = {.Lock = HAL_UNLOCKED, \
                                            .ErrorCode = HAL_FLASH_ERROR_NONE, \
                                            .ProcedureOnGoing = 0U, \
                                            .Address = 0U, \
                                            .Bank = FLASH_BANK_1, \
                                            .Page = 0U, \
                                            .NbPagesToErase = 0U, \
											.CacheToReactivate = FLASH_CACHE_DISABLED
                                           };*/

/* Private function prototypes -----------------------------------------------*/
//static void OPENBL_FLASH_Program(uint32_t FlashAddress, uint32_t DataAddress);
static ErrorStatus OPENBL_FLASH_EnableWriteProtection(uint32_t OB_WrpArea);
static ErrorStatus OPENBL_FLASH_DisableWriteProtection(void);

/* Exported variables --------------------------------------------------------*/
OPENBL_MemoryTypeDef FLASH_Descriptor =
{
  FLASH_START_ADDRESS,
  FLASH_END_ADDRESS,
  FLASH_MEM_SIZE,
  FLASH_AREA,
  OPENBL_FLASH_Read,
  OPENBL_FLASH_Write,
  NULL,
  OPENBL_FLASH_SetWriteProtection,
  OPENBL_FLASH_JumpToAddress,
  OPENBL_FLASH_MassErase,
  OPENBL_FLASH_Erase
};

extern TIM_HandleTypeDef htim3;

/* Exported functions --------------------------------------------------------*/

/**
  * @brief  Unlock the FLASH control register access.
  * @retval None.
  */
void OPENBL_FLASH_Unlock(void)
{
  HAL_FLASH_Unlock();
}

/**
  * @brief  Lock the FLASH control register access.
  * @retval None.
  */
void OPENBL_FLASH_Lock(void)
{
  HAL_FLASH_Lock();
}

/**
  * @brief  Unlock the FLASH Option Bytes Registers access.
  * @retval None.
  */
void OPENBL_FLASH_OB_Unlock(void)
{
  HAL_FLASH_Unlock();

  HAL_FLASH_OB_Unlock();
}

/**
  * @brief  This function is used to read data from a given address.
  * @param  Address The address to be read.
  * @retval Returns the read value.
  */
uint8_t OPENBL_FLASH_Read(uint32_t Address)
{
  return (*(uint8_t *)(Address));
}

/**
  * @brief  This function is used to write data in FLASH memory.
  * @param  Address The address where that data will be written.
  * @param  pData The data to be written.
  * @param  DataLength The length of the data to be written.
  * @retval None.
  */
void OPENBL_FLASH_Write(uint32_t Address, uint8_t *pData, uint32_t DataLength)
{
  uint32_t index;
  uint64_t data;

  if ((pData != NULL) && (DataLength != 0U))
  {
    /* Unlock the flash memory for write operation */
    OPENBL_FLASH_Unlock();

    /* Clear error programming flags */
    __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_ALL_ERRORS);

    /* Program 64 bits at a time */
    while (DataLength >= 8U)
    {
      data = 0;
      for (index = 0U; index < 8U; index++)
      {
        data |= (uint64_t)pData[index] << (8 * index); // Packing 8-bit values into 64-bit word
      }

      /* Clear all FLASH errors flags before starting write operation */
      __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_ALL_ERRORS);

      HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, Address, data);

      Address    += 8U;
      pData      += 8U;
      DataLength -= 8U;
    }

    /* If remaining count, pack the rest and fill with 0xFF */
    if (DataLength > 0U)
    {
      data = 0xFFFFFFFFFFFFFFFFU;
      for (index = 0U; index < DataLength; index++)
      {
        data &= ~(0xFFULL << (8 * index));
        data |= (uint64_t)pData[index] << (8 * index);
      }

      /* Clear all FLASH errors flags before starting write operation */
      __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_ALL_ERRORS);

      HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, Address, data);
    }

    /* Lock the Flash to disable the flash control register access */
    OPENBL_FLASH_Lock();
  }
}

/**
  * @brief  This function is used to jump to a given address.
  * @param  Address The address where the function will jump.
  * @retval None.
  */
void OPENBL_FLASH_JumpToAddress(uint32_t Address)
{
  // === De-initialize all HW resources to their reset values ===
  // Disable interfaces
  HAL_SPI_DeInit(&hspi1);
  HAL_SPI_DeInit(&hspi2);
  HAL_SPI_DeInit(&hspi3);
  HAL_UART_DeInit(&huart1);

  // Peripherals to intentionally keep initialized:
  //HAL_GPIO_DeInit(); // All GPIO

  HAL_GPIO_DeInit(GPIOH, GPO_TP17_Pin|GPIO2_Pin|GPIO3_Pin);
  HAL_GPIO_DeInit(GPIOE, GPO_SBANDRX_Pin|GPO_DEN_Pin|GPO_RXLOEN_Pin|SPI2_NSS_RxSynth_Pin
					  	  |SPI2_NSS_RxDAC_Pin|GPO_RXPDEN_Pin|GPO_RXLNASD_Pin|GPO_PHVRXEN_Pin
						  |GPO_HEATON_Pin|GPO_RSEL2n_Pin|GPO_RSEL3n_Pin|GPO_RSEL1n_Pin
						  |GPO_RSEL0n_Pin|GPO_LPBK1_Pin|GPO_LPBK0_Pin);
  HAL_GPIO_DeInit(GPIOI, SPI2_NSS_IqDemod_Pin|GPO_TP14_Pin|GPO_TP15_Pin);
  HAL_GPIO_DeInit(GPI_MPAEN_GPIO_Port, GPI_MPAEN_Pin);
  HAL_GPIO_DeInit(GPIOB, GPO_MEN_Pin|GPO_SBANDTX_Pin|GPO_PHVTXEN_Pin|GPO_TSEL0n_Pin
          	  	  	  	  |GPO_TXPDEN_Pin|GPO_TSEL2n_Pin|GPO_TSEL1n_Pin);
  HAL_GPIO_DeInit(GPIOC, SPI3_NSS_TxDAC_Pin|GPO_TXLOEN_Pin|GPO_STXDEN_Pin|GPO_CTXDEN_Pin);
  HAL_GPIO_DeInit(GPIO1_GPIO_Port, GPIO1_Pin);
  HAL_GPIO_DeInit(GPI_DO_NOT_USE_GPIO_Port, GPI_DO_NOT_USE_Pin);
  HAL_GPIO_DeInit(SPI3_NSS_TxSynth_GPIO_Port, SPI3_NSS_TxSynth_Pin);
  HAL_GPIO_DeInit(GPIO0_GPIO_Port, GPIO0_Pin);
  HAL_GPIO_DeInit(GPIOG, SPI3_NSS_AR_Pin|GPO_TPX_Pin|GPO_TP16_Pin);
  HAL_GPIO_DeInit(I2C3SDA_EEPROM_GPIO_Port, I2C3SDA_EEPROM_Pin);
  HAL_GPIO_DeInit(GPIOC, DMDTMP_Pin|P28VPA_Pin);
  HAL_GPIO_DeInit(GPIOA, TXRPWR_Pin|CRTNSENSE_Pin);

  __HAL_RCC_GPIOH_CLK_DISABLE();
  __HAL_RCC_GPIOE_CLK_DISABLE();
  __HAL_RCC_GPIOB_CLK_DISABLE();
  __HAL_RCC_GPIOA_CLK_DISABLE();
  __HAL_RCC_GPIOI_CLK_DISABLE();
  __HAL_RCC_GPIOG_CLK_DISABLE();
  __HAL_RCC_GPIOD_CLK_DISABLE();
  __HAL_RCC_GPIOC_CLK_DISABLE();
  HAL_DAC_DeInit(&hdac1); // Both MCU DACs

  // Disable timer
  HAL_TIM_Base_DeInit(&htim3);
  HAL_TIM_Base_Stop_IT(&htim3);
  __HAL_RCC_TIM3_CLK_DISABLE();
  //HAL_NVIC_DisableIRQ(TIM3_IRQn);
  HAL_RCC_DeInit();

  // Disable systick timer and reset it to default values
  SysTick->CTRL = 0;
  SysTick->LOAD = 0;
  SysTick->VAL = 0;
  HAL_DeInit(); // reset all peripherals

  // Disable ALL IRQs at the peripheral level
  for(int i = -14; i < 94; i++)
  {
	  HAL_NVIC_DisableIRQ(i);
  }

  // Disable all the interrupts at the CPU level
  __disable_irq(); // If there are any IRQs left.

  uint32_t stack_pointer = *(__IO uint32_t *) Address;
  uint32_t program_counter = *(__IO uint32_t *)(Address + 4U);

  // Update vector table location
  SCB->VTOR = Address;

  // Initialize user application's stack pointer
  __set_MSP(stack_pointer);

  // Create pointer to the beginning of the application code and run
  void (*jump_to_application)(void) = (void(*)(void))program_counter; // AKA: reset_handler
  //NVIC_SystemReset(); // reset & bootup in jump_to_application();

  // Enable all the interrupts at the CPU level
  __enable_irq();

  jump_to_application(); // Run the RFuC Application
  while(1){}; // Never get here
}

/**
  * @brief  Get the the product state.
  * @retval ProductState returns the product state.
  *         This returned value can be one of @ref FLASH_OB_Product_State
  */
uint32_t OPENBL_FLASH_GetProductState(void)
{
  FLASH_OBProgramInitTypeDef flash_ob;

  /* Get the Option bytes configuration */
  HAL_FLASHEx_OBGetConfig(&flash_ob);

  return flash_ob.RDPLevel;
}

/**
  * @brief  This function is used to enable or disable write protection of the specified FLASH areas.
  * @param  State Can be one of these values:
  *         @arg DISABLE: Disable FLASH write protection
  *         @arg ENABLE: Enable FLASH write protection
  * @param  pListOfSectors Contains the list of sectors to be protected.
  * @param  Length The length of the list of sectors to be protected.
  * @retval An ErrorStatus enumeration value:
  *          - SUCCESS: Enable or disable of the write protection is done
  *          - ERROR:   Enable or disable of the write protection is not done
  */
ErrorStatus OPENBL_FLASH_SetWriteProtection(FunctionalState State, uint32_t OB_WrpArea)
{
  ErrorStatus status = SUCCESS;

  if (State == ENABLE)
  {
    OPENBL_FLASH_EnableWriteProtection(OB_WrpArea);

    /* Register system reset callback */
    Common_SetPostProcessingCallback(OPENBL_OB_Launch);
  }
  else if (State == DISABLE)
  {
    OPENBL_FLASH_DisableWriteProtection();

    /* Register system reset callback */
    Common_SetPostProcessingCallback(OPENBL_OB_Launch);
  }
  else
  {
    status = ERROR;
  }

  return status;
}

/**
  * @brief  This function is used to start FLASH mass erase operation.
  * @param  *pData Pointer to the buffer that contains mass erase operation options.
  * @param  DataLength Size of the Data buffer.
  * @retval An ErrorStatus enumeration value:
  *          - SUCCESS: Mass erase operation done
  *          - ERROR:   Mass erase operation failed or the value of one parameter is not OK
  */
ErrorStatus OPENBL_FLASH_MassErase(uint8_t *pData, uint32_t DataLength)
{
  uint32_t sector_error;
  uint16_t bank_option;
  ErrorStatus status = SUCCESS;
  FLASH_EraseInitTypeDef erase_init_struct;

  // Unlock the flash memory for erase operation
  OPENBL_FLASH_Unlock();

  // Clear all FLASH errors flags before starting write operation
  __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_ALL_ERRORS);

  erase_init_struct.TypeErase = FLASH_TYPEERASE_MASSERASE;

  if (DataLength >= 2U)
  {
    bank_option = *(uint16_t *)(pData);

    if (bank_option == FLASH_MASS_ERASE)
    {
      erase_init_struct.Banks = 0U;
    }
    else if (bank_option == FLASH_BANK1_ERASE)
    {
      erase_init_struct.Banks = FLASH_BANK_1;
    }
    else if (bank_option == FLASH_BANK2_ERASE)
    {
      erase_init_struct.Banks = FLASH_BANK_2;
    }
    else
    {
      status = ERROR;
    }

    if (status == SUCCESS)
    {
      if (HAL_FLASHEx_Erase(&erase_init_struct, &sector_error) != HAL_OK)
      {
        status = ERROR;
      }
      else
      {
        status = SUCCESS;
      }
    }
  }
  else
  {
    status = ERROR;
  }

  // Lock the Flash to disable the flash control register access
  OPENBL_FLASH_Lock();

  return status;
}

/**
  * @brief  This function is used to erase the specified FLASH pages.
  * @param  *pData Pointer to the buffer that contains erase operation options.
  * @param  DataLength Size of the Data buffer.
  * @retval An ErrorStatus enumeration value:
  *          - SUCCESS: Erase operation done
  *          - ERROR:   Erase operation failed or the value of one parameter is not OK
  */
ErrorStatus OPENBL_FLASH_Erase(uint8_t *pData, uint32_t DataLength)
{
  uint32_t counter;
  uint32_t pages_number;
  uint32_t page_error = 0U;
  uint32_t errors       = 0U;
  ErrorStatus status    = SUCCESS;
  FLASH_EraseInitTypeDef erase_init_struct;

  /* Unlock the flash memory for erase operation */
  OPENBL_FLASH_Unlock();

  /* Clear error programming flags */
  __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_ALL_ERRORS);

  pages_number = (uint32_t)(*(uint16_t *)(pData));

  /* The sector number size is 2 bytes */
  pData += 2U;

  erase_init_struct.TypeErase = FLASH_TYPEERASE_PAGES;
  erase_init_struct.NbPages = 1U;

  for (counter = 0U; ((counter < pages_number) && (counter < (DataLength / 2U))); counter++)
  {
    erase_init_struct.Page = ((uint32_t)(*(uint16_t *)(pData)));

    if (erase_init_struct.Page <= ((FLASH_PAGE_NUMBER / 2U) - 1U))
    {
      erase_init_struct.Banks = FLASH_BANK_1;
    }
    else if (erase_init_struct.Page <= (FLASH_PAGE_NUMBER - 1U))
    {
      erase_init_struct.Banks = FLASH_BANK_2;
    }
    else
    {
      status = ERROR;
    }

    if (status != ERROR)
    {
      if (HAL_FLASHEx_Erase(&erase_init_struct, &page_error) != HAL_OK)
      {
        errors++;
      }
    }
    else
    {
      /* Reset the status for next erase operation */
      status = SUCCESS;
    }

    /* The sector number size is 2 bytes */
    pData += 2U;
  }

  /* Lock the Flash to disable the flash control register access */
  OPENBL_FLASH_Lock();

  if (errors > 0U)
  {
    status = ERROR;
  }
  else
  {
    status = SUCCESS;
  }

  return status;
}

/**
  * @brief  This function is used to Set Flash busy state variable to activate busy state sending
  *         during flash operations
  * @retval None.
  */
void OPENBL_Enable_BusyState_Flag(void)
{
  /* Enable Flash busy state sending */
  Flash_BusyState = FLASH_BUSY_STATE_ENABLED;
}

/**
  * @brief  This function is used to disable the send of busy state in I2C non stretch mode.
  * @retval None.
  */
void OPENBL_Disable_BusyState_Flag(void)
{
  /* Disable Flash busy state sending */
  Flash_BusyState = FLASH_BUSY_STATE_DISABLED;
}

/* Private functions ---------------------------------------------------------*/

/**
  * @brief  Program double word at a specified FLASH address.
  * @param  FlashAddress specifies the address to be programmed.
  * @param  DataAddress specifies the address of data to be programmed.
  * @retval None.
  */
/*static void OPENBL_FLASH_Program(uint32_t FlashAddress, uint32_t DataAddress)
{
  // Clear all FLASH errors flags before starting write operation
  __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_ALL_ERRORS);

  HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, FlashAddress, DataAddress);
}*/

/**
  * @brief  This function is used to enable write protection of the specified FLASH areas.
  * @param  OB_WrpArea Contains the area to be protected.
  * @retval An ErrorStatus enumeration value:
  *          - SUCCESS: Enable or disable of the write protection is done
  *          - ERROR:   Enable or disable of the write protection is not done
  */
static ErrorStatus OPENBL_FLASH_EnableWriteProtection(uint32_t OB_WrpArea)
{
  ErrorStatus status = SUCCESS;
  FLASH_OBProgramInitTypeDef flash_ob;

  /* Unlock the FLASH registers & Option Bytes registers access */
  OPENBL_FLASH_OB_Unlock();

  /* Clear error programming flags */
  __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_ALL_ERRORS);

  /* Enable FLASH_WRP_SECTORS write protection */
  flash_ob.OptionType = OPTIONBYTE_WRP;
  //flash_ob.WRPState   = OB_WRPSTATE_ENABLE;

  if (    (OB_WrpArea == OB_WRPAREA_BANK1_AREAA) ||
		  (OB_WrpArea == OB_WRPAREA_BANK1_AREAB) ||
		  (OB_WrpArea == OB_WRPAREA_BANK2_AREAA) ||
		  (OB_WrpArea == OB_WRPAREA_BANK2_AREAB) )
  {
	flash_ob.WRPArea  = OB_WrpArea;
    status = (HAL_FLASHEx_OBProgram(&flash_ob) != HAL_OK) ? ERROR : SUCCESS;
  }

  return status;
}

/**
  * @brief  This function is used to disable write protection.
  * @retval An ErrorStatus enumeration value:
  *          - SUCCESS: Enable or disable of the write protection is done
  *          - ERROR:   Enable or disable of the write protection is not done
  */
static ErrorStatus OPENBL_FLASH_DisableWriteProtection(void)
{
  ErrorStatus status = SUCCESS;
  //FLASH_OBProgramInitTypeDef flash_ob;

  /* Unlock the FLASH registers & Option Bytes registers access */
  OPENBL_FLASH_OB_Unlock();

  /* Clear all FLASH errors flags before starting write operation */
  __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_ALL_ERRORS);

  //flash_ob.OptionType = OPTIONBYTE_WRP;

  /* Disable write protection of bank 1 area */
  //flash_ob.WRPState  = OB_WRPSTATE_DISABLE;
  //flash_ob.WRPArea = OB_WRP_SECTOR_ALL;

  //HAL_FLASHEx_OBProgram(&flash_ob);

  return status;
}

/**
  * @brief  Wait for a FLASH operation to complete.
  * @param  Timeout maximum flash operation timeout.
  * @retval HAL_Status
  */
#if defined (__ICCARM__)
__ramfunc HAL_StatusTypeDef OPENBL_FLASH_WaitForLastOperation(uint32_t Timeout)
#else
__attribute__((section(".ramfunc"))) HAL_StatusTypeDef OPENBL_FLASH_WaitForLastOperation(uint32_t Timeout)
#endif /* (__ICCARM__) */
{
  return FLASH_WaitForLastOperation(Timeout);
/*
  uint32_t error;
  __IO uint32_t *reg_sr;
  uint32_t counter         = 0U;
  HAL_StatusTypeDef status = HAL_OK;

  // While the FLASH is in busy state, send busy byte to the host
  while (__HAL_FLASH_GET_FLAG(FLASH_FLAG_BSY))
  {
    // Check if we need to send a busy byte
       NOTE: this can be removed if I2C protocol is not used
    //if (Flash_BusyState == FLASH_BUSY_STATE_ENABLED)
    //{
    //  OPENBL_I2C_SendBusyByte();
    //}

    if ((counter++) > Timeout)
    {
      status = HAL_TIMEOUT;
      break;
    }
  }

  // Access to SECSR or NSSR registers depends on operation type
  //reg_sr = IS_FLASH_SECURE_OPERATION() ? &(FLASH->SECSR) : &(FLASH_NS->NSSR);
  //FLASH_TypeDef

  // Check FLASH operation error flags
  error = ((*reg_sr) & FLASH_FLAG_SR_ERRORS);

#if defined (__ARM_FEATURE_CMSE) && (__ARM_FEATURE_CMSE == 3U)
  error |= (FLASH->NSSR & FLASH_FLAG_OPTWERR);
#endif // __ARM_FEATURE_CMSE

  if (error != 0U)
  {
    // Save the error code
    FlashProcess.ErrorCode |= error;

    // Clear error programming flags
    (*reg_sr) = error;

#if defined (__ARM_FEATURE_CMSE) && (__ARM_FEATURE_CMSE == 3U)
    if ((error & FLASH_FLAG_OPTWERR) != 0U)
    {
      FLASH->NSSR = FLASH_FLAG_OPTWERR;
    }
#endif // __ARM_FEATURE_CMSE

    status = HAL_ERROR;
  }

  // Check FLASH End of Operation flag
  if (((*reg_sr) & FLASH_FLAG_EOP) != 0U)
  {
    // Clear FLASH End of Operation pending bit
    (*reg_sr) = FLASH_FLAG_EOP;
  }

  return status;
  */
}

/**
  * @brief  Perform a mass erase or erase the specified FLASH memory sectors.
  * @param[in]  pEraseInit pointer to an FLASH_EraseInitTypeDef structure that
  *         contains the configuration information for the erasing.
  * @param[out]  SectorError pointer to variable that contains the configuration
  *         information on faulty sector in case of error (0xFFFFFFFF means that all
  *         the sectors have been correctly erased).
  * @retval HAL_Status
  */
/*
#if defined (__ICCARM__)
__ramfunc HAL_StatusTypeDef OPENBL_FLASH_ExtendedErase(FLASH_EraseInitTypeDef *pEraseInit, uint32_t *SectorError)
#else
__attribute__((section(".ramfunc"))) HAL_StatusTypeDef OPENBL_FLASH_ExtendedErase(
  FLASH_EraseInitTypeDef *pEraseInit, uint32_t *SectorError)
#endif // (__ICCARM__)
{
  uint32_t errors = 0U;
  __IO uint32_t *reg_cr;
  HAL_StatusTypeDef status;

  // Process Locked
  __HAL_LOCK(&FlashProcess);

  // Reset error code
  FlashProcess.ErrorCode = HAL_FLASH_ERROR_NONE;

  // Wait for last operation to be completed on Bank1
  status = OPENBL_FLASH_WaitForLastOperation((uint32_t)OPENBL_FLASH_TIMEOUT_VALUE);

  if (status == HAL_OK)
  {
    // Initialization of SectorError variable
    *SectorError = 0xFFFFFFFFU;

    // Access to SECCR or NSCR registers depends on operation type
#if defined (FLASH_OPTSR2_TZEN)
    reg_cr = IS_FLASH_SECURE_OPERATION() ? &(FLASH->SECCR) : &(FLASH_NS->NSCR);
#else
    reg_cr = &(FLASH_NS->NSCR);
#endif // FLASH_OPTSR2_TZEN

    if (((pEraseInit->Banks) & FLASH_BANK_1) == FLASH_BANK_1)
    {
      // Reset Sector Number for Bank1
      CLEAR_BIT((*reg_cr), (FLASH_CR_BKSEL | FLASH_CR_SNB));
    }

    if (((pEraseInit->Banks) & FLASH_BANK_2) == FLASH_BANK_2)
    {
      // Reset Sector Number for Bank2
      CLEAR_BIT((*reg_cr), FLASH_CR_SNB);
      SET_BIT((*reg_cr), FLASH_CR_BKSEL);

      pEraseInit->Sector = pEraseInit->Sector - (FLASH_SECTOR_127 + 1U);
    }

    // Proceed to erase the sector
    SET_BIT((*reg_cr), (FLASH_CR_SER | (pEraseInit->Sector << FLASH_CR_SNB_Pos) | FLASH_CR_START));

    // Wait for last operation to be completed
    if (OPENBL_FLASH_WaitForLastOperation(PROGRAM_TIMEOUT) != HAL_OK)
    {
      errors++;
    }

    // If the erase operation is completed, disable the SER Bit
    CLEAR_BIT(FLASH->NSCR, (FLASH_CR_SER));

    if (status != HAL_OK)
    {
      // In case of error, stop erase procedure and return the faulty sector
      *SectorError = pEraseInit->Sector;
    }
  }

  // Process Unlocked
  __HAL_UNLOCK(&FlashProcess);

  if (errors > 0U)
  {
    status = HAL_ERROR;
  }
  else
  {
    status = HAL_OK;
  }

  return status;
}
*/
