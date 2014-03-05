/**
 * \file   gpio.c
 *
 * \brief  This file contains functions which performs the platform specific
 *         configurations of GPIO.
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


#include "hw_control_AM335x.h"
#include "soc_AM335x.h"
#include "hw_cm_wkup.h"
#include "beaglebone.h"
#include "hw_cm_per.h"
#include "hw_types.h"

/**
 * \brief  This function does the Pin Multiplexing and selects GPIO pin
 *         GPIO1[23] for use. GPIO1[23] means 23rd pin of GPIO1 instance.
 *         This pin can be used to control the Audio Buzzer.
 *
 * \param  None
 *
 * \return None
 *
 * \note   Either of GPIO1[23] or GPIO1[30] pins could be used to control the
 *         Audio Buzzer.
 */
void GPIO1Pin23PinMuxSetup(void)
{
    HWREG(SOC_CONTROL_REGS + CONTROL_CONF_GPMC_A(7)) = CONTROL_CONF_MUXMODE(7);
}

/*
** This function enables the L3 and L4_WKUP interface clocks.
** This also enables the functional clock for GPIO0 instance.
*/

void GPIO0ModuleClkConfig(void)
{
    /* Writing to MODULEMODE field of CM_WKUP_GPIO0_CLKCTRL register. */
    HWREG(SOC_CM_WKUP_REGS + CM_WKUP_GPIO0_CLKCTRL) |=
        CM_WKUP_GPIO0_CLKCTRL_MODULEMODE_ENABLE;

    /* Waiting for MODULEMODE field to reflect the written value. */
    while(CM_WKUP_GPIO0_CLKCTRL_MODULEMODE_ENABLE !=
          (HWREG(SOC_CM_WKUP_REGS + CM_WKUP_GPIO0_CLKCTRL) &
           CM_WKUP_GPIO0_CLKCTRL_MODULEMODE));

    /*
    ** Writing to OPTFCLKEN_GPIO0_GDBCLK field of CM_WKUP_GPIO0_CLKCTRL
    ** register.
    */
    HWREG(SOC_CM_WKUP_REGS + CM_WKUP_GPIO0_CLKCTRL) |=
        CM_WKUP_GPIO0_CLKCTRL_OPTFCLKEN_GPIO0_GDBCLK;

    /* Waiting for OPTFCLKEN_GPIO0_GDBCLK field to reflect the written value. */
    while(CM_WKUP_GPIO0_CLKCTRL_OPTFCLKEN_GPIO0_GDBCLK !=
          (HWREG(SOC_CM_WKUP_REGS + CM_WKUP_GPIO0_CLKCTRL) &
           CM_WKUP_GPIO0_CLKCTRL_OPTFCLKEN_GPIO0_GDBCLK));

    /* Verifying if the other bits are set to required settings. */

    /*
    ** Waiting for IDLEST field in CM_WKUP_CONTROL_CLKCTRL register to attain
    ** desired value.
    */
    while((CM_WKUP_CONTROL_CLKCTRL_IDLEST_FUNC <<
           CM_WKUP_CONTROL_CLKCTRL_IDLEST_SHIFT) !=
          (HWREG(SOC_CM_WKUP_REGS + CM_WKUP_CONTROL_CLKCTRL) &
           CM_WKUP_CONTROL_CLKCTRL_IDLEST));

    /*
    ** Waiting for CLKACTIVITY_L3_AON_GCLK field in CM_L3_AON_CLKSTCTRL
    ** register to attain desired value.
    */
    while(CM_WKUP_CM_L3_AON_CLKSTCTRL_CLKACTIVITY_L3_AON_GCLK !=
          (HWREG(SOC_CM_WKUP_REGS + CM_WKUP_CM_L3_AON_CLKSTCTRL) &
           CM_WKUP_CM_L3_AON_CLKSTCTRL_CLKACTIVITY_L3_AON_GCLK));

    /*
    ** Waiting for IDLEST field in CM_WKUP_L4WKUP_CLKCTRL register to attain
    ** desired value.
    */
    while((CM_WKUP_L4WKUP_CLKCTRL_IDLEST_FUNC <<
           CM_WKUP_L4WKUP_CLKCTRL_IDLEST_SHIFT) !=
          (HWREG(SOC_CM_WKUP_REGS + CM_WKUP_L4WKUP_CLKCTRL) &
           CM_WKUP_L4WKUP_CLKCTRL_IDLEST));

    /*
    ** Waiting for CLKACTIVITY_L4_WKUP_GCLK field in CM_WKUP_CLKSTCTRL register
    ** to attain desired value.
    */
    while(CM_WKUP_CLKSTCTRL_CLKACTIVITY_L4_WKUP_GCLK !=
          (HWREG(SOC_CM_WKUP_REGS + CM_WKUP_CLKSTCTRL) &
           CM_WKUP_CLKSTCTRL_CLKACTIVITY_L4_WKUP_GCLK));

    /*
    ** Waiting for CLKACTIVITY_L4_WKUP_AON_GCLK field in CM_L4_WKUP_AON_CLKSTCTRL
    ** register to attain desired value.
    */
    while(CM_WKUP_CM_L4_WKUP_AON_CLKSTCTRL_CLKACTIVITY_L4_WKUP_AON_GCLK !=
          (HWREG(SOC_CM_WKUP_REGS + CM_WKUP_CM_L4_WKUP_AON_CLKSTCTRL) &
           CM_WKUP_CM_L4_WKUP_AON_CLKSTCTRL_CLKACTIVITY_L4_WKUP_AON_GCLK));


    /* Writing to IDLEST field in CM_WKUP_GPIO0_CLKCTRL register. */
    while((CM_WKUP_GPIO0_CLKCTRL_IDLEST_FUNC <<
           CM_WKUP_GPIO0_CLKCTRL_IDLEST_SHIFT) !=
          (HWREG(SOC_CM_WKUP_REGS + CM_WKUP_GPIO0_CLKCTRL) &
           CM_WKUP_GPIO0_CLKCTRL_IDLEST));

    /*
    ** Waiting for CLKACTIVITY_GPIO0_GDBCLK field in CM_WKUP_GPIO0_CLKCTRL
    ** register to attain desired value.
    */
    while(CM_WKUP_CLKSTCTRL_CLKACTIVITY_GPIO0_GDBCLK !=
          (HWREG(SOC_CM_WKUP_REGS + CM_WKUP_CLKSTCTRL) &
           CM_WKUP_CLKSTCTRL_CLKACTIVITY_GPIO0_GDBCLK));
}

/*
** This function enables the L3 and L4_PER interface clocks.
** This also enables functional clocks of GPIO1 instance.
*/

void GPIO1ModuleClkConfig(void)
{

    /* Writing to MODULEMODE field of CM_PER_GPIO1_CLKCTRL register. */
    HWREG(SOC_CM_PER_REGS + CM_PER_GPIO1_CLKCTRL) |=
          CM_PER_GPIO1_CLKCTRL_MODULEMODE_ENABLE;

    /* Waiting for MODULEMODE field to reflect the written value. */
    while(CM_PER_GPIO1_CLKCTRL_MODULEMODE_ENABLE !=
          (HWREG(SOC_CM_PER_REGS + CM_PER_GPIO1_CLKCTRL) &
           CM_PER_GPIO1_CLKCTRL_MODULEMODE));
    /*
    ** Writing to OPTFCLKEN_GPIO_1_GDBCLK bit in CM_PER_GPIO1_CLKCTRL
    ** register.
    */
    HWREG(SOC_CM_PER_REGS + CM_PER_GPIO1_CLKCTRL) |=
          CM_PER_GPIO1_CLKCTRL_OPTFCLKEN_GPIO_1_GDBCLK;

    /*
    ** Waiting for OPTFCLKEN_GPIO_1_GDBCLK bit to reflect the desired
    ** value.
    */
    while(CM_PER_GPIO1_CLKCTRL_OPTFCLKEN_GPIO_1_GDBCLK !=
          (HWREG(SOC_CM_PER_REGS + CM_PER_GPIO1_CLKCTRL) &
           CM_PER_GPIO1_CLKCTRL_OPTFCLKEN_GPIO_1_GDBCLK));

    /*
    ** Waiting for IDLEST field in CM_PER_GPIO1_CLKCTRL register to attain the
    ** desired value.
    */
    while((CM_PER_GPIO1_CLKCTRL_IDLEST_FUNC <<
           CM_PER_GPIO1_CLKCTRL_IDLEST_SHIFT) !=
           (HWREG(SOC_CM_PER_REGS + CM_PER_GPIO1_CLKCTRL) &
            CM_PER_GPIO1_CLKCTRL_IDLEST));

    /*
    ** Waiting for CLKACTIVITY_GPIO_1_GDBCLK bit in CM_PER_L4LS_CLKSTCTRL
    ** register to attain desired value.
    */
    while(CM_PER_L4LS_CLKSTCTRL_CLKACTIVITY_GPIO_1_GDBCLK !=
          (HWREG(SOC_CM_PER_REGS + CM_PER_L4LS_CLKSTCTRL) &
           CM_PER_L4LS_CLKSTCTRL_CLKACTIVITY_GPIO_1_GDBCLK));
}




/*
** This function enables GPIO1 pins
*/
void GPIO1PinMuxSetup(unsigned int pinNo)
{
	HWREG(SOC_CONTROL_REGS + CONTROL_CONF_GPMC_AD(pinNo)) =
		(CONTROL_CONF_GPMC_AD_CONF_GPMC_AD_SLEWCTRL | 	/* Slew rate slow */
		CONTROL_CONF_GPMC_AD_CONF_GPMC_AD_RXACTIVE |	/* Receiver enabled */
		(CONTROL_CONF_GPMC_AD_CONF_GPMC_AD_PUDEN & (~CONTROL_CONF_GPMC_AD_CONF_GPMC_AD_PUDEN)) | /* PU_PD enabled */
		(CONTROL_CONF_GPMC_AD_CONF_GPMC_AD_PUTYPESEL & (~CONTROL_CONF_GPMC_AD_CONF_GPMC_AD_PUTYPESEL)) | /* PD */
		(CONTROL_CONF_MUXMODE(7))	/* Select mode 7 */
		);
}

/**
 * \brief  This function does the pin multiplexing for any GPIO Pin.
 *
 * \param  offsetAddr   This is the offset address of the Pad Control Register
 *                      corresponding to the GPIO pin. These addresses are
 *                      offsets with respect to the base address of the
 *                      Control Module.
 * \param  padConfValue This is the value to be written to the Pad Control
 *                      register whose offset address is given by 'offsetAddr'.
 *
 * The 'offsetAddr' and 'padConfValue' can be obtained from macros defined
 * in the file 'include/armv7a/am335x/pin_mux.h'.\n
 *
 * \return  None.
 */
void GpioPinMuxSetup(unsigned int offsetAddr, unsigned int padConfValue)
{
    HWREG(SOC_CONTROL_REGS + offsetAddr) = (padConfValue);
}

/****************************** End of file *********************************/
