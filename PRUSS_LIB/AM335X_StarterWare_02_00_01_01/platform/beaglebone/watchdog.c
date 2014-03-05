/**
 * \file   watchdog.c
 *
 * \brief  This file contains functions which configure clock for watchdog 
 *         timer.
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
#include "hw_types.h"
#include "beaglebone.h"
#include "hw_cm_per.h"
#include "hw_cm_wkup.h"
#include "hw_cm_dpll.h"

/*
** No pin muxing is required for WDT
*/

/*
** Enable clocks for WDT.
*/
void WatchdogTimer1ModuleClkConfig(void)
{
    /* Select 32kHz clock from 32K clock divider for WDT1 */
    HWREG(SOC_CM_DPLL_REGS + CM_DPLL_CLKSEL_WDT1_CLK) = 
                                    CM_DPLL_CLKSEL_WDT1_CLK_CLKSEL_SEL2;
   
    while((HWREG(SOC_CM_DPLL_REGS + CM_DPLL_CLKSEL_WDT1_CLK) & 
    CM_DPLL_CLKSEL_WDT1_CLK_CLKSEL) != CM_DPLL_CLKSEL_WDT1_CLK_CLKSEL_SEL2);

    /* Configuration of L4_PER bus */
    HWREG(SOC_CM_PER_REGS + CM_PER_L3S_CLKSTCTRL) =
                             CM_PER_L3S_CLKSTCTRL_CLKTRCTRL_SW_WKUP;

    while((HWREG(SOC_CM_PER_REGS + CM_PER_L3S_CLKSTCTRL) &
    CM_PER_L3S_CLKSTCTRL_CLKTRCTRL) != CM_PER_L3S_CLKSTCTRL_CLKTRCTRL_SW_WKUP);

    HWREG(SOC_CM_PER_REGS + CM_PER_L3_CLKSTCTRL) =
                             CM_PER_L3_CLKSTCTRL_CLKTRCTRL_SW_WKUP;

    while((HWREG(SOC_CM_PER_REGS + CM_PER_L3_CLKSTCTRL) &
     CM_PER_L3_CLKSTCTRL_CLKTRCTRL) != CM_PER_L3_CLKSTCTRL_CLKTRCTRL_SW_WKUP);

    HWREG(SOC_CM_PER_REGS + CM_PER_L3_INSTR_CLKCTRL) =
                             CM_PER_L3_INSTR_CLKCTRL_MODULEMODE_ENABLE;

    while((HWREG(SOC_CM_PER_REGS + CM_PER_L3_INSTR_CLKCTRL) &
                               CM_PER_L3_INSTR_CLKCTRL_MODULEMODE) !=
                                   CM_PER_L3_INSTR_CLKCTRL_MODULEMODE_ENABLE);

    HWREG(SOC_CM_PER_REGS + CM_PER_L3_CLKCTRL) =
                             CM_PER_L3_CLKCTRL_MODULEMODE_ENABLE;

    while((HWREG(SOC_CM_PER_REGS + CM_PER_L3_CLKCTRL) &
        CM_PER_L3_CLKCTRL_MODULEMODE) != CM_PER_L3_CLKCTRL_MODULEMODE_ENABLE);

    HWREG(SOC_CM_PER_REGS + CM_PER_OCPWP_L3_CLKSTCTRL) =
                             CM_PER_OCPWP_L3_CLKSTCTRL_CLKTRCTRL_SW_WKUP;

    while((HWREG(SOC_CM_PER_REGS + CM_PER_OCPWP_L3_CLKSTCTRL) &
                              CM_PER_OCPWP_L3_CLKSTCTRL_CLKTRCTRL) !=
                                CM_PER_OCPWP_L3_CLKSTCTRL_CLKTRCTRL_SW_WKUP);

    /* Configuration for L4_WKUP bus */
    HWREG(SOC_CM_WKUP_REGS + CM_WKUP_CLKSTCTRL) = 
                             CM_WKUP_CLKSTCTRL_CLKTRCTRL_SW_WKUP;        

    while((HWREG(SOC_CM_WKUP_REGS + CM_WKUP_CLKSTCTRL) & 
           CM_WKUP_CLKSTCTRL_CLKTRCTRL) != CM_WKUP_CLKSTCTRL_CLKTRCTRL_SW_WKUP);

    HWREG(SOC_CM_WKUP_REGS + CM_WKUP_CONTROL_CLKCTRL) = 
                                      CM_WKUP_CONTROL_CLKCTRL_MODULEMODE_ENABLE;

    while((HWREG(SOC_CM_WKUP_REGS + CM_WKUP_CONTROL_CLKCTRL) & 
     CM_WKUP_CONTROL_CLKCTRL_MODULEMODE) != CM_WKUP_CONTROL_CLKCTRL_MODULEMODE_ENABLE);

    HWREG(SOC_CM_WKUP_REGS + CM_WKUP_DEBUGSS_CLKCTRL) |= 
                                    CM_WKUP_DEBUGSS_CLKCTRL_MODULEMODE_ENABLE;
    
    while((HWREG(SOC_CM_WKUP_REGS + CM_WKUP_DEBUGSS_CLKCTRL) & 
      CM_WKUP_DEBUGSS_CLKCTRL_MODULEMODE) != CM_WKUP_DEBUGSS_CLKCTRL_MODULEMODE_ENABLE);

    HWREG(SOC_CM_WKUP_REGS + CM_WKUP_WDT1_CLKCTRL) = 
                                   CM_WKUP_WDT1_CLKCTRL_MODULEMODE_ENABLE;

    while((HWREG(SOC_CM_WKUP_REGS + CM_WKUP_WDT1_CLKCTRL) & 
           CM_WKUP_WDT1_CLKCTRL_MODULEMODE) != CM_WKUP_WDT1_CLKCTRL_MODULEMODE_ENABLE);

    while(!(HWREG(SOC_CM_WKUP_REGS + CM_WKUP_CLKSTCTRL) & 
           (CM_WKUP_CLKSTCTRL_CLKACTIVITY_L4_WKUP_GCLK | 
            CM_WKUP_CLKSTCTRL_CLKACTIVITY_WDT1_GCLK)));

    while(!(HWREG(SOC_CM_WKUP_REGS + CM_WKUP_L4WKUP_CLKCTRL) & 0x02u));

    while(!(HWREG(SOC_CM_WKUP_REGS + CM_WKUP_WKUP_M3_CLKCTRL) & 0x02u));

    while(!(HWREG(SOC_CM_WKUP_REGS + CM_WKUP_CM_L4_WKUP_AON_CLKSTCTRL) & 0x02u));
}

