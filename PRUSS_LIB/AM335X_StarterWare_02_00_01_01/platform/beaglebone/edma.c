/**
 * \file  edma.c
 *
 * \brief Platform related APIs for EDMA
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
#include "hw_cm_per.h"
#include "hw_types.h"


/**
 *  \brief    This function maps the crossbar events. 
 *
 *  \param    baseAdd         It is the Control Module Address.
 *
 *  \param    crossBarEvent   It is the crossBar event number.
 *
 *  \param    Channel         It is the channel number to which cross bar
 *                            event needs to be mapped.
 */

unsigned int EDMA3CrossBarChannelMap(unsigned int baseAdd, unsigned int crossBarEvent,
                                     unsigned int Channel)
{

    unsigned int offset;
    unsigned int select;
    unsigned int n = 0;

    /* offset of the TPCC_MUX to be configured */
    offset = Channel / 4;

    /* 
    ** Each TPCC_MUX register has four event mux which can be used for
    ** cross bar mapping.Thus "select" variable is used to select,
    ** which of the event mux out of four,for a given TPCC_MUX register 
    ** to be used.
    */
    select = Channel - offset * 4;

    switch(select)
    {
         case 0:
              n = 0;
         break;

         case 1:
              n = 8;
         break;

         case 2:
              n = 16;
         break;

         case 3:
              n = 24;
         break;

         default:
         break;
    }
    
    /* 'n' specifies the offset of the event mux */
    HWREG(baseAdd + TPCC_MUX(offset)) &= ~(crossBarEvent << n);

    HWREG(baseAdd + TPCC_MUX(offset)) |= crossBarEvent << n;

    return 0;
}


 /**  
 * \brief  This API returns a unique number which identifies itself  
 *         with the EDMA IP in AM335x SoC.  
 * \param  None  
 * \return This returns a number '2' which is unique to EDMA IP in AM335x.
 */
unsigned int EDMAVersionGet(void)
{
    return 2;
}

/*
** This function enables the system L3 clocks.
** This also enables the clocks for EDMA instance.
*/

void EDMAModuleClkConfig(void)
{
    /* Configuring clocks for EDMA3 TPCC and TPTCs. */

    /* Writing to MODULEMODE field of CM_PER_TPCC_CLKCTRL register. */
    HWREG(SOC_CM_PER_REGS + CM_PER_TPCC_CLKCTRL) |=
          CM_PER_TPCC_CLKCTRL_MODULEMODE_ENABLE;

    /* Waiting for MODULEMODE field to reflect the written value. */
    while(CM_PER_TPCC_CLKCTRL_MODULEMODE_ENABLE !=
          (HWREG(SOC_CM_PER_REGS + CM_PER_TPCC_CLKCTRL) &
          CM_PER_TPCC_CLKCTRL_MODULEMODE));

    /* Writing to MODULEMODE field of CM_PER_TPTC0_CLKCTRL register. */
    HWREG(SOC_CM_PER_REGS + CM_PER_TPTC0_CLKCTRL) |=
          CM_PER_TPTC0_CLKCTRL_MODULEMODE_ENABLE;

    /* Waiting for MODULEMODE field to reflect the written value. */
    while(CM_PER_TPTC0_CLKCTRL_MODULEMODE_ENABLE !=
          (HWREG(SOC_CM_PER_REGS + CM_PER_TPTC0_CLKCTRL) &
           CM_PER_TPTC0_CLKCTRL_MODULEMODE));

    /* Writing to MODULEMODE field of CM_PER_TPTC1_CLKCTRL register. */
    HWREG(SOC_CM_PER_REGS + CM_PER_TPTC1_CLKCTRL) |=
          CM_PER_TPTC1_CLKCTRL_MODULEMODE_ENABLE;

    /* Waiting for MODULEMODE field to reflect the written value. */
    while(CM_PER_TPTC1_CLKCTRL_MODULEMODE_ENABLE !=
          (HWREG(SOC_CM_PER_REGS + CM_PER_TPTC1_CLKCTRL) &
           CM_PER_TPTC1_CLKCTRL_MODULEMODE));

    /* Writing to MODULEMODE field of CM_PER_TPTC2_CLKCTRL register. */
    HWREG(SOC_CM_PER_REGS + CM_PER_TPTC2_CLKCTRL) |=
          CM_PER_TPTC2_CLKCTRL_MODULEMODE_ENABLE;

    /* Waiting for MODULEMODE field to reflect the written value. */
    while(CM_PER_TPTC2_CLKCTRL_MODULEMODE_ENABLE !=
          (HWREG(SOC_CM_PER_REGS + CM_PER_TPTC2_CLKCTRL) &
           CM_PER_TPTC2_CLKCTRL_MODULEMODE));


	/*	DMA in non-idle mode	*/
	HWREG(0x49800010) = 0x00000028;
	HWREG(0x49900010) = 0x00000028;
	HWREG(0x49a00010) = 0x00000028;


    /*
    ** Waiting for IDLEST field in CM_PER_TPCC_CLKCTRL register to attain the
    ** desired value.
    */
    while((CM_PER_TPCC_CLKCTRL_IDLEST_FUNC <<
           CM_PER_TPCC_CLKCTRL_IDLEST_SHIFT) !=
           (HWREG(SOC_CM_PER_REGS + CM_PER_TPCC_CLKCTRL) &
            CM_PER_TPCC_CLKCTRL_IDLEST));

    /*
    ** Waiting for IDLEST field in CM_PER_TPTC0_CLKCTRL register to attain the
    ** desired value.
    */
    while((CM_PER_TPTC0_CLKCTRL_IDLEST_FUNC <<
           CM_PER_TPTC0_CLKCTRL_IDLEST_SHIFT) !=
           (HWREG(SOC_CM_PER_REGS + CM_PER_TPTC0_CLKCTRL) &
            CM_PER_TPTC0_CLKCTRL_IDLEST));

    /*
    ** Waiting for STBYST field in CM_PER_TPTC0_CLKCTRL register to attain the
    ** desired value.
    */
    while((CM_PER_TPTC0_CLKCTRL_STBYST_FUNC <<
           CM_PER_TPTC0_CLKCTRL_STBYST_SHIFT) !=
           (HWREG(SOC_CM_PER_REGS + CM_PER_TPTC0_CLKCTRL) &
            CM_PER_TPTC0_CLKCTRL_STBYST));

    /*
    ** Waiting for IDLEST field in CM_PER_TPTC1_CLKCTRL register to attain the
    ** desired value.
    */
    while((CM_PER_TPTC1_CLKCTRL_IDLEST_FUNC <<
           CM_PER_TPTC1_CLKCTRL_IDLEST_SHIFT) !=
           (HWREG(SOC_CM_PER_REGS + CM_PER_TPTC1_CLKCTRL) &
            CM_PER_TPTC1_CLKCTRL_IDLEST));

    /*
    ** Waiting for STBYST field in CM_PER_TPTC1_CLKCTRL register to attain the
    ** desired value.
    */
    while((CM_PER_TPTC1_CLKCTRL_STBYST_FUNC <<
           CM_PER_TPTC1_CLKCTRL_STBYST_SHIFT) !=
           (HWREG(SOC_CM_PER_REGS + CM_PER_TPTC1_CLKCTRL) &
            CM_PER_TPTC1_CLKCTRL_STBYST));

    /*
    ** Waiting for IDLEST field in CM_PER_TPTC2_CLKCTRL register to attain the
    ** desired value.
    */
    while((CM_PER_TPTC2_CLKCTRL_IDLEST_FUNC <<
           CM_PER_TPTC2_CLKCTRL_IDLEST_SHIFT) !=
           (HWREG(SOC_CM_PER_REGS + CM_PER_TPTC2_CLKCTRL) &
            CM_PER_TPTC2_CLKCTRL_IDLEST));

    /*
    ** Waiting for STBYST field in CM_PER_TPTC2_CLKCTRL register to attain the
    ** desired value.
    */
    while((CM_PER_TPTC2_CLKCTRL_STBYST_FUNC <<
           CM_PER_TPTC2_CLKCTRL_STBYST_SHIFT) !=
           (HWREG(SOC_CM_PER_REGS + CM_PER_TPTC2_CLKCTRL) &
            CM_PER_TPTC2_CLKCTRL_STBYST));
}

