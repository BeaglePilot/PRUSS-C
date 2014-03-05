/**
 * \file   ctlregcontext.c
 *
 * \brief  This file contains functions for saving and restoring platform
 *         related registers
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
#include "hw_types.h"
#include "beaglebone.h"

/******************************************************************************
**                       INTERNAL MACRO DEFINITIONS
******************************************************************************/
#define CTRL_IOPAD_OFFSET                 (0x800)
#define CTRL_IOPAD_REG_BASE               (SOC_CONTROL_REGS + CTRL_IOPAD_OFFSET)

/******************************************************************************
**                          FUNCTION DEFINITIONS
******************************************************************************/
/**
 * \brief   This function saves the platform related context
 *
 * \param   Pointer to the structure where context needs to be saved
 *
 * \return  None
 *
 */
void ControlRegContextSave(CTRLREGCONTEXT *contextPtr)
{
    unsigned int idx;

    contextPtr->pwmssctrl = HWREG(SOC_CONTROL_REGS + CONTROL_PWMSS_CTRL);
    contextPtr->gmiisel = HWREG(SOC_CONTROL_REGS + CONTROL_GMII_SEL);

    for(idx = 0; idx < CTRL_NUM_IOPAD_REGS; idx++)
    {
        contextPtr->ioPadMap[idx].ioPad =
                                         HWREG(CTRL_IOPAD_REG_BASE + (idx * 4));
    }
}

/**
 * \brief   This function restores the platform related context
 *
 * \param   Pointer to the structure where context needs to be restored from
 *
 * \return  None
 *
 */
void ControlRegContextRestore(CTRLREGCONTEXT *contextPtr)
{
    unsigned int idx;

    HWREG(SOC_CONTROL_REGS + CONTROL_PWMSS_CTRL) = contextPtr->pwmssctrl ;
    HWREG(SOC_CONTROL_REGS + CONTROL_GMII_SEL) = contextPtr->gmiisel;

    for(idx = 0; idx < CTRL_NUM_IOPAD_REGS; idx++)
    {
        HWREG(CTRL_IOPAD_REG_BASE + (idx * 4)) =
                                                contextPtr->ioPadMap[idx].ioPad;
    }
}

/*
** This function saves the selected IO Pad context
*/
void IOPadContextSave(CTRLREGCONTEXT *contextPtr, unsigned int ioPadOff)
{
    unsigned int ioPadIdx = (ioPadOff - CTRL_IOPAD_OFFSET)/4;

    contextPtr->ioPadMap[ioPadIdx].ioPad =
                                    HWREG(CTRL_IOPAD_REG_BASE + (ioPadIdx * 4));
}

/*
** This function restores the selected IO Pad context
*/
void IOPadContextRestore(CTRLREGCONTEXT *contextPtr, unsigned int ioPadOff)
{
    unsigned int ioPadIdx = (ioPadOff - CTRL_IOPAD_OFFSET)/4;

    HWREG(CTRL_IOPAD_REG_BASE + (ioPadIdx * 4)) =
                                           contextPtr->ioPadMap[ioPadIdx].ioPad;
}

/*
** This function configures given IO Pad
*/
void IOPadConfigure(unsigned int ioPadOff, unsigned int ioPadValue)
{
    unsigned int idx = (ioPadOff - CTRL_IOPAD_OFFSET)/4;

    HWREG(CTRL_IOPAD_REG_BASE + (idx * 4)) = ioPadValue;
}

/*
** This function configures IO Pad selection
*/
void IOPadSel(CTRLREGCONTEXT *contextPtr, unsigned int ioPadOff,
              unsigned int ioPadSel)
{
    unsigned int idx = (ioPadOff - CTRL_IOPAD_OFFSET)/4;

    if(!ioPadSel)
    {
        contextPtr->ioPadMap[idx].sel = false;
    }
    else
    {
        contextPtr->ioPadMap[idx].sel = true;
    }
}

/*
** This function configures selected IO Pads.
**
** Note: Use IOPadSel to configure selection.
*/
void IOPadSelConfigure(CTRLREGCONTEXT *contextPtr, unsigned int ioPadValue)
{
    unsigned int idx = 0;

    for(idx = 0; idx < CTRL_NUM_IOPAD_REGS; idx++)
    {
        if(false == contextPtr->ioPadMap[idx].sel)
        {
            HWREG(CTRL_IOPAD_REG_BASE + (idx * 4)) = ioPadValue;
        }
    }
}
