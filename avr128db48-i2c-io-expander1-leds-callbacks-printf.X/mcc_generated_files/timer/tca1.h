/**
 * TCA1 Generated Timer Driver API Header File
 * 
 * @file tca1.h
 * 
 * @ingroup timerdriver
 * 
 * @brief This file contains API prototypes and other data types for the TCA1 Timer Driver.
 *
 * @version TCA1 Timer Driver Version 3.0.0
 *
 * @version Package Version 7.1.0
*/
/*
© [2025] Microchip Technology Inc. and its subsidiaries.

    Subject to your compliance with these terms, you may use Microchip 
    software and any derivatives exclusively with Microchip products. 
    You are responsible for complying with 3rd party license terms  
    applicable to your use of 3rd party software (including open source  
    software) that may accompany Microchip software. SOFTWARE IS ?AS IS.? 
    NO WARRANTIES, WHETHER EXPRESS, IMPLIED OR STATUTORY, APPLY TO THIS 
    SOFTWARE, INCLUDING ANY IMPLIED WARRANTIES OF NON-INFRINGEMENT,  
    MERCHANTABILITY, OR FITNESS FOR A PARTICULAR PURPOSE. IN NO EVENT 
    WILL MICROCHIP BE LIABLE FOR ANY INDIRECT, SPECIAL, PUNITIVE, 
    INCIDENTAL OR CONSEQUENTIAL LOSS, DAMAGE, COST OR EXPENSE OF ANY 
    KIND WHATSOEVER RELATED TO THE SOFTWARE, HOWEVER CAUSED, EVEN IF 
    MICROCHIP HAS BEEN ADVISED OF THE POSSIBILITY OR THE DAMAGES ARE 
    FORESEEABLE. TO THE FULLEST EXTENT ALLOWED BY LAW, MICROCHIP?S 
    TOTAL LIABILITY ON ALL CLAIMS RELATED TO THE SOFTWARE WILL NOT 
    EXCEED AMOUNT OF FEES, IF ANY, YOU PAID DIRECTLY TO MICROCHIP FOR 
    THIS SOFTWARE.
*/

#ifndef TCA1_H
#define TCA1_H

#include <stdint.h>
#include <stdbool.h>
#include "../system/utils/compiler.h"
#include "timer_interface.h"

/**
 * @ingroup timerdriver
 * @brief Defines the maximum count of the timer.
 */
#define TCA1_MAX_COUNT (65535U)

/**
 * @ingroup timerdriver
 * @brief Defines the timer prescaled clock frequency in hertz.
 */
#define TCA1_CLOCK_FREQ (15625UL)


/**
 * @ingroup timerdriver
 * @brief Defines the Custom Name for the \ref TCA1_MAX_COUNT.
 */
#define TIMER1_MAX_COUNT TCA1_MAX_COUNT

/**
 * @ingroup timerdriver
 * @brief Defines the Custom Name for the \ref TCA1_CLOCK_FREQ.
 */
#define TIMER1_CLOCK_FREQ TCA1_CLOCK_FREQ

/**
 * @ingroup timerdriver
 * @brief Defines the Custom Name for the \ref TCA1_Initialize API.
 */
#define Timer1_Initialize TCA1_Initialize

/**
 * @ingroup timerdriver
 * @brief Defines the Custom Name for the \ref TCA1_Deinitialize API.
 */
#define Timer1_Deinitialize TCA1_Deinitialize

/**
 * @ingroup timerdriver
 * @brief Defines the Custom Name for the \ref TCA1_Start API.
 */
#define Timer1_Start TCA1_Start

/**
 * @ingroup timerdriver
 * @brief Defines the Custom Name for the \ref TCA1_Stop API.
 */
#define Timer1_Stop TCA1_Stop

/**
 * @ingroup timerdriver
 * @brief Defines the Custom Name for the \ref TCA1_CounterGet API.
 */
#define Timer1_CounterGet TCA1_CounterGet

/**
 * @ingroup timerdriver
 * @brief Defines the Custom Name for the \ref TCA1_CounterSet API.
 */
#define Timer1_CounterSet TCA1_CounterSet

/**
 * @ingroup timerdriver
 * @brief Defines the Custom Name for the \ref TCA1_PeriodSet API.
 */
#define Timer1_PeriodSet TCA1_PeriodSet

/**
 * @ingroup timerdriver
 * @brief Defines the Custom Name for the \ref TCA1_PeriodGet API.
 */
#define Timer1_PeriodGet TCA1_PeriodGet

/**
 * @ingroup timerdriver
 * @brief Defines the Custom Name for the \ref TCA1_MaxCountGet API.
 */
#define Timer1_MaxCountGet TCA1_MaxCountGet

/**
 * @ingroup timerdriver
 * @brief Defines the Custom Name for the \ref TCA1_OverflowCallbackRegister API.
 */
#define Timer1_OverflowCallbackRegister TCA1_OverflowCallbackRegister

/**
 @ingroup timerdriver
 @struct TIMER_INTERFACE
 @brief Declares an instance of TIMER_INTERFACE for the TCA1 module.
 */
extern const struct TIMER_INTERFACE Timer1;

/**
 * @ingroup timerdriver
 * @brief Initializes the TCA1 module.
 *        This routine must be called before any other TCA1 routines.
 * @param None.
 * @return None.
 */
void TCA1_Initialize(void);

/**
 * @ingroup timerdriver
 * @brief Deinitializes the TCA1 module.
 * @param None.
 * @return None.
 */
void TCA1_Deinitialize(void);

/**
 * @ingroup timerdriver
 * @brief Starts TCA1.
 * @pre Initialize TCA1 with TCA1_Initialize() before calling this API.
 * @param None.
 * @return None.
 */
void TCA1_Start(void);

/**
 * @ingroup timerdriver
 * @brief Stops TCA1.
 * @pre Initialize TCA1 with TCA1_Initialize() before calling this API.
 * @param None.
 * @return None.
 */
void TCA1_Stop(void);

/**
 * @ingroup timerdriver
 * @brief Returns the current counter value.
 * @pre Initialize TCA1 with TCA1_Initialize() before calling this API.
 * @param None.
 * @return Counter value
 */
uint32_t TCA1_CounterGet(void);

/**
 * @ingroup timerdriver
 * @brief Sets the counter value.
 * @pre Initialize TCA1 with TCA1_Initialize() before calling this API.
 * @param counterValue - Counter value to be written to the CNT register
 * @return None.
 */
void TCA1_CounterSet(uint32_t counterValue);

/**
 * @ingroup timerdriver
 * @brief Sets the period count value.
 * @pre Initialize TCA1 with TCA1_Initialize() before calling this API.
 * @param periodCount - Period count value to be written to the PER register
 * @return None.
 */
void TCA1_PeriodSet(uint32_t periodCount);

/**
 * @ingroup timerdriver
 * @brief Returns the current period value.
 * @pre Initialize TCA1 with TCA1_Initialize() before calling this API.
 * @param None.
 * @return Period count value
 */
uint32_t TCA1_PeriodGet(void);

/**
 * @ingroup timerdriver
 * @brief Returns the maximum count value.
 * @param None.
 * @return Maximum count value
 */
uint32_t TCA1_MaxCountGet(void);

/**
 * @ingroup timerdriver
 * @brief Registers a callback function for the TCA1 overflow event.
 * @param CallbackHandler - Address of the custom callback function
 * @return None.
 */
 void TCA1_OverflowCallbackRegister(void (* CallbackHandler)(void));


#endif //TCA1_H