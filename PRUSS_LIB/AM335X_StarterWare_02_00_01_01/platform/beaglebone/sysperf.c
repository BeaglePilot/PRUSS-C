/**
 * \file   sysperf.c
 *
 * \brief  This file contains functions that configures a DMTimer instance
 *         for performance measurement.
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
#include "perf.h"

/******************************************************************************
**                      INTERNAL MACRO DEFINITIONS
*******************************************************************************/
#define TIMER_INITIAL_COUNT             (0xFFFFA23Fu)
#define TIMER_PERF_BASE                 (SOC_DMTIMER_7_REGS)

/******************************************************************************
**                      INTERNAL VARIABLE DEFINITIONS
*******************************************************************************/
static volatile unsigned int flagIsr = 1;

/******************************************************************************
**                      FUNCTION DEFINITIONS
*******************************************************************************/
/*
** This function sets up timer for performance measurement
*/
void SysPerfTimerSetup(void)
{   
    /* This function will enable clocks for the DMTimer7 instance */
    DMTimer7ModuleClkConfig();

    DMTimerCounterSet(TIMER_PERF_BASE, 0);

}

/*
** Configures the performance timer to start or stop timer 
** @param  flag   '0', stop the timer and read the value
**                non-zero value to start timer
** /NOTE  This function shouldnot be called when SysStartTimer, SysStopTimer,
**        SysIsTimerElapsed or Sysdelay functionality is in use and vice Versa.
**             Maximim Duration is 171 Sec.
** 
*/
unsigned int SysPerfTimerConfig(unsigned int flag)
{
    unsigned int timeInTicks = 0;

    if(flag)
    {
        DMTimerCounterSet(TIMER_PERF_BASE, 0);
        DMTimerEnable(TIMER_PERF_BASE);
    }
    else
    {
        DMTimerDisable(TIMER_PERF_BASE);
        timeInTicks = DMTimerCounterGet(TIMER_PERF_BASE);
    }

    return timeInTicks;
}


