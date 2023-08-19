/**
  ******************************************************************************
  * NOTICE:  This source code and its derivative thereof was prepared by/for
  * Battelle Energy Alliance, LLC and Idaho National Laboratory, hereinafter the
  * Contractor, under Contract DE-AC07-05ID14517 with the United States (U.S.)
  * Department of Energy (DOE).  All rights in the source code and its
  * derivative thereof are reserved by DOE on behalf of the United States
  * Government and the Contractor as provided in the Contract. This source code
  * and its derivative thereof is not to be released or distributed to the
  * public without written permission of the Contractor. NEITHER THE GOVERNMENT
  * NOR THE CONTRACTOR MAKES ANY WARRANTY, EXPRESS OR IMPLIED, OR ASSUMES ANY
  * LIABILITY FOR THE USE OF THIS SOFTWARE. This notice including this sentence
  * must appear on any copies of this source code and its derivative thereof.
  *
  * EXPORT RESTRICTIONS. The provider of this source code and its employees and
  * its agents are subject to U.S. export control laws that prohibit or restrict
  * (i) transactions with certain parties, and (ii) the type and level of
  * technologies and services that may be exported.  You agree to comply fully
  * with all laws and regulations of the United States and other countries
  * (Export Laws) to assure that neither this source code, nor any direct
  * products thereof are (1) exported, directly or indirectly, in violation of
  * Export Laws, or (2) are used for any purpose prohibited by Export Laws,
  * including, without limitation, nuclear, chemical, or biological weapons
  * proliferation.
  *
  * None of this source code or underlying information or technology may be
  * downloaded or otherwise exported or re-exported (i) into (or to a national
  * or resident of) Cuba, North Korea, Iran, Sudan, Syria or any other country
  * to which the U.S. has embargoed goods; or (ii) to anyone on the U.S.
  * Treasury Department's List of Specially Designated Nationals or the U.S.
  * Commerce Department's Denied Persons List, Unverified List, Entity List,
  * Nonproliferation Sanctions or General Orders.  By downloading or using this
  * source code, you are agreeing to the foregoing and you are representing and
  * warranting that you are not located in, under the control of, or a national
  * or resident of any such country or on any such list, and that you
  * acknowledge you are responsible to obtain any necessary U.S. government
  * authorization to ensure compliance with U.S. law.
  ******************************************************************************
  * File          : USART1Terminal.c
  * Classification: UNCLASSIFIED
  * Purpose       : PRINTF to UART Terminal
  ******************************************************************************
  */

///============================================================================
/// Includes
///============================================================================
#include "USART1Terminal.h"
#include "cmsis_os.h"
#include "stdio.h"
#include "stdarg.h"
#include "string.h"

///============================================================================
/// Public Definitions
///============================================================================


///============================================================================
/// Static Declarations
///============================================================================
void Uart_LocalPrint(void);

//#define RING_BUFFER_SIZE 10

///============================================================================
/// Methods
///============================================================================

///----------------------------------------------------------------------------
/// Method Name: Uart_LocalPrint
/// Description: Pops next PRINTF message off queue and sends it out through
///							 the Uart1Tx peripheral.
///----------------------------------------------------------------------------
void Uart_LocalPrint(void){
	static char bufferLocal[255];
    for (int i = 0; i < sizeof(bufferLocal); i++) {
        bufferLocal[i] = 0;
    }

	if(osMessageQueueGetCount(UartTxQueueHandle)){
		osMessageQueueGet(UartTxQueueHandle, &bufferLocal, NULL, 0);
		HAL_UART_Transmit_IT(&huart1, (uint8_t *)bufferLocal, strlen(bufferLocal));
	}
}

///----------------------------------------------------------------------------
/// Method Name: PRINTF
/// Description: Pushes char* (character string) to atomic queue. When the
///							 USART1TermTask gets to it, it will pop the char* from the
///							 queue and print the string through the UART1 TX port. Up to
///							 254 ascii characters can transmitted per PRINTF statement. If
///							 the queue buffer is overloaded, it will print an error that
///							 the queue is overloaded. This function is disable while in
///							 CLI Mode as to not overload the terminal while the user is
///							 typing.
///----------------------------------------------------------------------------
void PRINTF(const char *format, ...){
	static char bufferTxEr[] = "\n\n\nERROR! UART Tx BUFFER OVERFLOW!\n\n\n\r";
	va_list ap;
	char buffer1[255]={0}; // print up to 254 characters per PRINTF

	va_start(ap, format);
	vsnprintf((char *)buffer1, 255, format, ap);
	va_end(ap);

	if(osMessageQueueGetSpace(UartTxQueueHandle) > 0){ // Push to queue
		osMessageQueuePut(UartTxQueueHandle, &buffer1, osPriorityNormal, 0);
	}
	else{ // Out of memory in queue!
		osMessageQueuePut(UartTxQueueHandle, &bufferTxEr, osPriorityNormal, osWaitForever);
	}
}

///----------------------------------------------------------------------------
/// Method Name: USART1TerminalTaskUserCode
/// Description: UART Terminal Task for printf statements, developer input
///              commands, and automated testing interface
///----------------------------------------------------------------------------
void USART1TerminalTaskUserCode(void){
	// ========= Startup Code =========

	PRINTF("\033[2J"); // Clear the terminal
	PRINTF("\033[s\033[1;1H"); // Save cursor position, return to top left corner
	//PRINTF("\033[5B"); // Move down 5 lines
	PRINTF("Running RFuC Bootloader...\n\r");

	// ================================

	// transmit next message off of PRINTF queue if not empty
	Uart_LocalPrint();

	// PRINTF("Running USART1TermTask\n\r"); // print to UART Console
	for(uint8_t i=0; i<2; i++){
		osDelay(25); // typical max length printf takes about 22ms
		//Main_UpdateLocalHeartBeat(eUSART1TermTask); // Tell task manager I'm alive & healthy

		// transmit next message off of PRINTF queue if not empty
		Uart_LocalPrint();
	}

	vTaskDelete(NULL);
}
