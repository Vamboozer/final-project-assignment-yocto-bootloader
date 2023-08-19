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
  * File          : USART1Terminal.h
  * Classification: UNCLASSIFIED
  * Purpose       : PRINTF to UART Terminal
  ******************************************************************************
  */

#ifndef INC_USART1TERMINAL_H_
#define INC_USART1TERMINAL_H_

#include "main.h"

void USART1TerminalTaskUserCode(void);
void PRINTF(const char *format, ...);

#endif /* INC_USART1TERMINAL_H_ */
