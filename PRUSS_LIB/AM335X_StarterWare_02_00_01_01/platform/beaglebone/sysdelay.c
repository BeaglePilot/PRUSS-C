/**
 * \file   sysdelay.c
 *
 * \brief  This file contains functions that configures a DMTimer instance
 *         for operation and to generate a requested amount of delay.
 *
*/

/*
* Copyright (C) 2010 Texas Instruments Incorporated - http://www.ti.com/
*/
/*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*    Redistributions of source code must retain the above copyright
*    notice, this list of conditions and the following disclaimer.
*
*    Redistributions in binary form must reproduce the above copyright
*    notice, this list of conditions and the following disclaimer in the
*    documentation and/or other materials provided with the
*    distribution.
*
*    Neither the name of Texas Instruments Incorporated nor the names of
*    its contributors may be used to endorse or promote products derived
*    from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
*  A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
*  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
*  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
*  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
*  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
*  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
*  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
*  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*
*/

#include "soc_AM335x.h"
#include "beaglebone.h"
#include "interrupt.h"
#include "dmtimer.h"
#include "delay.h"

/******************************************************************************
**                      INTERNAL MACRO DEFINITIONS
*******************************************************************************/
#define TIMER_INITIAL_COUNT             (0xFFFFA23Fu)
/* The Input clock is selected as 24MHz. So for 1ms set the count to 0x5DC0. 
 *If the input clock is changed to different source this value has to be updated 
 *accordingly.	
*/
#define TIMER_1MS_COUNT                 (0x5DC0u) 
#define TIMER_OVERFLOW                  (0xFFFFFFFFu)

/* If delay using interrupts is desire define this. If polling is desired,
   undefine this */
#define DELAY_USE_INTERRUPTS            1  

/******************************************************************************
**                      INTERNAL FUNCTION PROTOTYPES
*******************************************************************************/
static void DMTimerIsr(void);

/******************************************************************************
**                      INTERNAL VARIABLE DEFINITIONS
*******************************************************************************/
static volatile unsigned int flagIsr = 1;

/******************************************************************************
**                      FUNCTION DEFINITIONS
*******************************************************************************/

/**
 * This function configures the DMTimer7 instance. It also registers the
 * DMTimer Interrupt Service Routine in the Interrupt Controller, sets the
 * priority and enables the respective system interrupt.
 */

void SysDelayTimerSetup(void)
{   

#ifdef DELAY_USE_INTERRUPTS
    /* This function will enable clocks for the DMTimer7 instance */
    DMTimer7ModuleClkConfig();

    /* Registering DMTimerIsr */
    IntRegister(SYS_INT_TINT7, DMTimerIsr);

    /* Set the priority */
    IntPrioritySet(SYS_INT_TINT7, 0, AINTC_HOSTINT_ROUTE_IRQ);

    /* Enable the system interrupt */
    IntSystemEnable(SYS_INT_TINT7);

    DMTimerCounterSet(SOC_DMTIMER_7_REGS, 0);

    /* Configure the DMTimer for Auto-reload and compare mode */
    DMTimerModeConfigure(SOC_DMTIMER_7_REGS, DMTIMER_ONESHOT_NOCMP_ENABLE);
#else
    DMTimer7ModuleClkConfig();

    DMTimerModeConfigure(SOC_DMTIMER_7_REGS, DMTIMER_ONESHOT_NOCMP_ENABLE);
#endif

}

/**
 * This function generates the requested amount of delay in milliseconds.
 */

void Sysdelay(unsigned int milliSec)
{
#ifdef DELAY_USE_INTERRUPTS
    unsigned int countVal = TIMER_OVERFLOW - (milliSec * TIMER_1MS_COUNT);

    DMTimerCounterSet(SOC_DMTIMER_7_REGS, countVal);

    flagIsr = FALSE;

    /* Enable the DMTimer interrupts */
    DMTimerIntEnable(SOC_DMTIMER_7_REGS, DMTIMER_INT_OVF_EN_FLAG);

    /* Start the DMTimer */
    DMTimerEnable(SOC_DMTIMER_7_REGS);

    while(FALSE == flagIsr) ;

    /* Disable the DMTimer interrupts */
    DMTimerIntDisable(SOC_DMTIMER_7_REGS, DMTIMER_INT_OVF_EN_FLAG);

#else
    while(milliSec != 0)
    {
        DMTimerCounterSet(SOC_DMTIMER_7_REGS, 0);
        DMTimerEnable(SOC_DMTIMER_7_REGS);
        while(DMTimerCounterGet(SOC_DMTIMER_7_REGS) < TIMER_1MS_COUNT);
        DMTimerDisable(SOC_DMTIMER_7_REGS);
        milliSec--;
    }
 
#endif
}

/**
 * /brief This function starts the timer.
 *
 * /milliSec Maximum value = TIMER_OVERFLOW/TIMER_1MS_COUNT.
 *
 * /NOTE  SysDelay functionality cannot be used till SysStopTimer is called.
 */
void SysStartTimer(unsigned int milliSec)
{
#ifdef DELAY_USE_INTERRUPTS
    unsigned int countVal = TIMER_OVERFLOW - (milliSec * TIMER_1MS_COUNT);

    DMTimerCounterSet(SOC_DMTIMER_7_REGS, countVal);

    flagIsr = FALSE;

    /* Enable the DMTimer interrupts */
    DMTimerIntEnable(SOC_DMTIMER_7_REGS, DMTIMER_INT_OVF_EN_FLAG);

    /* Start the DMTimer */
    DMTimerEnable(SOC_DMTIMER_7_REGS);
#else
    DMTimerCounterSet(SOC_DMTIMER_7_REGS, 0);
    DMTimerEnable(SOC_DMTIMER_7_REGS);
    flagIsr = milliSec;
#endif
}

/**
 * This function stops the timer.
 */
void SysStopTimer(void)
{

#ifdef DELAY_USE_INTERRUPTS
    DMTimerDisable(SOC_DMTIMER_7_REGS);
    /* Disable the DMTimer interrupts */
    DMTimerIntDisable(SOC_DMTIMER_7_REGS, DMTIMER_INT_OVF_EN_FLAG);
#else
    DMTimerDisable(SOC_DMTIMER_7_REGS);
#endif
}

/**
 * This function checks whether 'count' milli secs are elapsed or not.
 * SysStartTimer has to be called prior to checking status.
 * NOTE: SysDelay functionality cannot be used till SysStopTimer is called.
 */
unsigned int SysIsTimerElapsed(void)
{

#ifdef DELAY_USE_INTERRUPTS

    return flagIsr;

#else
    if(DMTimerCounterGet(SOC_DMTIMER_7_REGS) < (flagIsr * TIMER_1MS_COUNT))
    {
        return 0;
    }
    else
    {
        return 1;
    }
#endif
}
#ifdef DELAY_USE_INTERRUPTS
/*
** DMTimer Interrupt Service Routine.
*/

static void DMTimerIsr(void)
{
    /* Clear the status of the interrupt flags */
    DMTimerIntStatusClear(SOC_DMTIMER_7_REGS, DMTIMER_INT_OVF_EN_FLAG);

    DMTimerDisable(SOC_DMTIMER_7_REGS);

    flagIsr = TRUE;
}
#endif

