/**
 * \file   usb.c
 *
 * \brief  This file contains the board specific code for enabling the use of
 *         usb driver.
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
#include "hw_control_AM335x.h"
#include "hw_usbOtg_AM335x.h"
#include "hw_types.h"
#include "beaglebone.h"
#include "hw_cm_per.h"
#include "hw_cm_wkup.h"

/**  
 * \brief  This API returns a unique number which identifies itself  
 *         with the USB IP in AM335x SoC.  
 * \param  None  
 * \return This returns a number '2' which is unique to USB IP in AM335x.
 */
unsigned int USBVersionGet(void)
{
    return 2;
}

/**
 * \brief   This function enables USB clocks
 *          
 * \param   None
 *
 * \return  None.
 *
 */
void USB0ModuleClkConfig(void)
{
	HWREG(SOC_CM_WKUP_REGS + CM_WKUP_CM_CLKDCOLDO_DPLL_PER) |= 
		CM_WKUP_CM_CLKDCOLDO_DPLL_PER_DPLL_CLKDCOLDO_GATE_CTRL;

	
	HWREG(SOC_PRCM_REGS + CM_PER_USB0_CLKCTRL) |=
                             CM_PER_USB0_CLKCTRL_MODULEMODE_ENABLE;

    while((HWREG(SOC_PRCM_REGS + CM_PER_USB0_CLKCTRL) &
      CM_PER_USB0_CLKCTRL_MODULEMODE) != CM_PER_USB0_CLKCTRL_MODULEMODE_ENABLE);


    /*
    ** Waiting for IDLEST field in CM_PER_USB0_CLKCTRL register to attain the
    ** desired value.
    */
    while((CM_PER_USB0_CLKCTRL_IDLEST_FUNC <<
           CM_PER_USB0_CLKCTRL_IDLEST_SHIFT)!=
          (HWREG(SOC_CM_PER_REGS + CM_PER_USB0_CLKCTRL) &
           CM_PER_USB0_CLKCTRL_IDLEST));

}

/**  
 * \brief  This API  enables the USB Interrupts through subsystem specific wrapper 
 *	       registers
 * \param  Base address 
 * \return None
 */
void USBEnableInt(unsigned int ulBase)
{
	HWREG(ulBase + USB_0_IRQ_ENABLE_SET_0) = 0xFFFFFFFF;
	HWREG(ulBase + USB_0_IRQ_ENABLE_SET_1) = 0xFFFFFFFF;
#ifdef DMA_MODE
	HWREG(USBSS_BASE + USBSS_IRQ_ENABLE_SET) = 0xFFFFFFFF;
#endif
}

/**  
 * \brief  This API  Clear  the USB Interrupts through subsystem specific wrapper 
 *	       registers
 * \param  Base address 
 * \return None
 */
void USBClearInt(unsigned int ulBase)
{

}
/**  
 * \brief  This API  enables the USB Module clock
 *	       registers
 * \param  Base address 
 * \return None
 */
void USBModuleClkEnable(unsigned int ulIndex, unsigned int ulBase)
{
	//
	//Call the clock enabel API
	//
	USB0ModuleClkConfig();	
}

/**  
 * \brief  This API Disables the module clock
 *	       registers
 * \param  Base address 
 * \return None
 */
void USBModuleClkDisable(unsigned int ulIndex, unsigned int ulBase)
{
	//
	//Disables the module clock
	//
	HWREG(SOC_PRCM_REGS + CM_PER_USB0_CLKCTRL) |=
                             CM_PER_USB0_CLKCTRL_MODULEMODE_DISABLE;
}

/****************************** End Of File *********************************/
