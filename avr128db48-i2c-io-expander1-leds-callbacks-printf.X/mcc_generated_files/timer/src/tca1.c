/**
 * TCA1 Generated Timer Driver File
 *
 * @file tca1.c
 *
 * @ingroup timerdriver
 *
 * @brief Timer Driver implementation for the TCA1 module.
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

#include "../tca1.h"

const struct TIMER_INTERFACE Timer1 = {
    .Initialize = TCA1_Initialize,
    .Deinitialize = TCA1_Deinitialize,
    .Start = TCA1_Start,
    .Stop = TCA1_Stop,
    .PeriodSet = TCA1_PeriodSet,
    .PeriodGet = TCA1_PeriodGet,
    .CounterSet = TCA1_CounterSet,
    .CounterGet = TCA1_CounterGet,
    .MaxCountGet =  TCA1_MaxCountGet,
    .TimeoutCallbackRegister = TCA1_OverflowCallbackRegister,
    .Tasks = NULL
};

static void (*TCA1_OverflowCallback)(void) = NULL;

void TCA1_Initialize(void)
{
    TCA1.SINGLE.CMP0 = 0x0;  // CMP0 0x0

    TCA1.SINGLE.CMP1 = 0x0;  // CMP1 0x0

    TCA1.SINGLE.CMP2 = 0x0;  // CMP2 0x0

    TCA1.SINGLE.CNT = 0x0;  // CNT 0x0

    TCA1.SINGLE.CTRLB = (0 << TCA_SINGLE_ALUPD_bp)   // ALUPD disabled
        | (0 << TCA_SINGLE_CMP0EN_bp)   // CMP0EN disabled
        | (0 << TCA_SINGLE_CMP1EN_bp)   // CMP1EN disabled
        | (0 << TCA_SINGLE_CMP2EN_bp)   // CMP2EN disabled
        | (TCA_SINGLE_WGMODE_NORMAL_gc);  // WGMODE NORMAL

    TCA1.SINGLE.CTRLC = (0 << TCA_SINGLE_CMP0OV_bp)   // CMP0OV disabled
        | (0 << TCA_SINGLE_CMP1OV_bp)   // CMP1OV disabled
        | (0 << TCA_SINGLE_CMP2OV_bp);  // CMP2OV disabled

    TCA1.SINGLE.CTRLD = (0 << TCA_SINGLE_SPLITM_bp);  // SPLITM disabled

    TCA1.SINGLE.CTRLECLR = (TCA_SINGLE_CMD_NONE_gc)   // CMD NONE
        | (0 << TCA_SINGLE_DIR_bp)   // DIR disabled
        | (0 << TCA_SINGLE_LUPD_bp);  // LUPD disabled

    TCA1.SINGLE.CTRLESET = (TCA_SINGLE_CMD_NONE_gc)   // CMD NONE
        | (TCA_SINGLE_DIR_UP_gc)   // DIR UP
        | (0 << TCA_SINGLE_LUPD_bp);  // LUPD disabled

    TCA1.SINGLE.CTRLFCLR = (0 << TCA_SINGLE_CMP0BV_bp)   // CMP0BV disabled
        | (0 << TCA_SINGLE_CMP1BV_bp)   // CMP1BV disabled
        | (0 << TCA_SINGLE_CMP2BV_bp)   // CMP2BV disabled
        | (0 << TCA_SINGLE_PERBV_bp);  // PERBV disabled

    TCA1.SINGLE.CTRLFSET = (0 << TCA_SINGLE_CMP0BV_bp)   // CMP0BV disabled
        | (0 << TCA_SINGLE_CMP1BV_bp)   // CMP1BV disabled
        | (0 << TCA_SINGLE_CMP2BV_bp)   // CMP2BV disabled
        | (0 << TCA_SINGLE_PERBV_bp);  // PERBV disabled

    TCA1.SINGLE.DBGCTRL = (0 << TCA_SINGLE_DBGRUN_bp);  // DBGRUN disabled

    TCA1.SINGLE.EVCTRL = (0 << TCA_SINGLE_CNTAEI_bp)   // CNTAEI disabled
        | (0 << TCA_SINGLE_CNTBEI_bp)   // CNTBEI disabled
        | (TCA_SINGLE_EVACTA_CNT_POSEDGE_gc)   // EVACTA CNT_POSEDGE
        | (TCA_SINGLE_EVACTB_NONE_gc);  // EVACTB NONE

    TCA1.SINGLE.INTCTRL = (0 << TCA_SINGLE_CMP0_bp)   // CMP0 disabled
        | (0 << TCA_SINGLE_CMP1_bp)   // CMP1 disabled
        | (0 << TCA_SINGLE_CMP2_bp)   // CMP2 disabled
        | (1 << TCA_SINGLE_OVF_bp);  // OVF enabled

    TCA1.SINGLE.INTFLAGS = (0 << TCA_SINGLE_CMP0_bp)   // CMP0 disabled
        | (0 << TCA_SINGLE_CMP1_bp)   // CMP1 disabled
        | (0 << TCA_SINGLE_CMP2_bp)   // CMP2 disabled
        | (0 << TCA_SINGLE_OVF_bp);  // OVF disabled

    TCA1.SINGLE.PER = 0x619U;

    TCA1.SINGLE.TEMP = 0x0;  // TEMP 0x0

    TCA1.SINGLE.CTRLA = (TCA_SINGLE_CLKSEL_DIV256_gc)   // CLKSEL DIV256
        | (1 << TCA_SINGLE_ENABLE_bp)   // ENABLE enabled
        | (0 << TCA_SINGLE_RUNSTDBY_bp);  // RUNSTDBY disabled
}

void TCA1_Deinitialize(void)
{
    TCA1.SINGLE.CTRLA &= ~TCA_SINGLE_ENABLE_bm;

    TCA1.SINGLE.CTRLA = 0x0;
    TCA1.SINGLE.CTRLB = 0x0;
    TCA1.SINGLE.CTRLC = 0x0;
    TCA1.SINGLE.CTRLD = 0x0;
    
    TCA1.SINGLE.CNT = 0x0;
    TCA1.SINGLE.PER = 0xffffU;
    TCA1.SINGLE.CMP0 = 0x0;
    TCA1.SINGLE.CMP1 = 0x0;
    TCA1.SINGLE.CMP2 = 0x0;
    TCA1.SINGLE.EVCTRL = 0x0;
    TCA1.SINGLE.CTRLESET = 0x0;
    TCA1.SINGLE.CTRLECLR = 0x0;
    TCA1.SINGLE.CTRLFSET = 0x0;
    TCA1.SINGLE.CTRLFCLR = 0x0;

    TCA1.SINGLE.INTCTRL = 0x0;
    TCA1.SINGLE.INTFLAGS = ~0x0;
}

void TCA1_Start(void)
{
    TCA1.SINGLE.CTRLA |= TCA_SINGLE_ENABLE_bm;
}

void TCA1_Stop(void)
{
    TCA1.SINGLE.CTRLA &= ~TCA_SINGLE_ENABLE_bm;
}

void TCA1_CounterSet(uint32_t counterValue)
{
    TCA1.SINGLE.CNT = (uint16_t)counterValue;
}

uint32_t TCA1_CounterGet(void)
{
    return (uint32_t)TCA1.SINGLE.CNT;
}

void TCA1_PeriodSet(uint32_t periodCount)
{
    TCA1.SINGLE.PER = (uint16_t)periodCount;
}

uint32_t TCA1_PeriodGet(void)
{
    return (uint32_t)TCA1.SINGLE.PER;
}

uint32_t TCA1_MaxCountGet(void)
{
    return (uint32_t)TCA1_MAX_COUNT;
}

/* cppcheck-suppress misra-c2012-2.7 */
/* cppcheck-suppress misra-c2012-8.2 */
/* cppcheck-suppress misra-c2012-8.4 */
ISR(TCA1_OVF_vect)
{
    if(NULL != TCA1_OverflowCallback)
    {
        (*TCA1_OverflowCallback)();
    }
    TCA1.SINGLE.INTFLAGS = TCA_SINGLE_OVF_bm;
}

void TCA1_OverflowCallbackRegister(void (* CallbackHandler)(void))
{
    TCA1_OverflowCallback = CallbackHandler;
}
