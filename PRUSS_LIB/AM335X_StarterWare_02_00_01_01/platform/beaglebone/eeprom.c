/**
 * \file   eeprom.c
 *
 * \brief  This file contains functions for accessing eeprom
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
#include "hsi2c.h"
#include "board.h"

/******************************************************************************
	**              INTERNAL FUNCTION PROTOTYPES
******************************************************************************/
static void StatusClear(void);
static void I2C0PinMux(void);
/******************************************************************************
**                       INTERNAL MACRO DEFINITIONS
******************************************************************************/
#define I2C_BASE_ADDR                       (SOC_I2C_0_REGS)
#define I2C_SLAVE_ADDR                      (0x50)
#define EEPROM_OFFSET                       (0x00)

/******************************************************************************
**                          FUNCTION DEFINITIONS
******************************************************************************/
/**
 * \brief   Sets up the EEPROM I2C interface
 *
 * \param   slaveAddr   Slave Address of the EEPROM
 *
 * \return  None.
 */
void EEPROMI2CSetUp(unsigned int slaveAddr)
{
    /* Configuring system clocks for I2C0 instance. */
    I2C0ModuleClkConfig();

    /* Performing Pin Multiplexing for I2C0. */
    I2C0PinMux();

    /* Put i2c in reset/disabled state */
    I2CMasterDisable(I2C_BASE_ADDR);

    I2CSoftReset(I2C_BASE_ADDR);

    /* Configure i2c bus speed to 100khz */
    I2CMasterInitExpClk(I2C_BASE_ADDR, 48000000, 24000000, 100000);

    /* Set i2c slave address */
    I2CMasterSlaveAddrSet(I2C_BASE_ADDR, slaveAddr);

    /* Disable all I2C interrupts */
    I2CMasterIntDisableEx(I2C_BASE_ADDR, 0xFFFFFFFF);

    /* Bring I2C module out of reset */
    I2CMasterEnable(I2C_BASE_ADDR);

    while(!I2CSystemStatusGet(I2C_BASE_ADDR));
}

/**
 * \brief   This function reads data from EEPROM.
 *
 * \param   data    Address where data is to be read.
 * \param   length  Length of data to be read
 * \param   offset  Address of the byte from which data to be read.
 *
 * \return  None.
 *
 * \note    This muxing depends on the profile in which the EVM is configured.
 *          EEPROMI2CSetUp Shall be called Before this API is used
 */
void EEPROMI2CRead(unsigned char *data, unsigned int length,
                   unsigned short offset)
{
    unsigned int idx = 0;

    /* First send the register offset - TX operation */
    I2CSetDataCount(I2C_BASE_ADDR, 2);

    StatusClear();

    I2CMasterControl(I2C_BASE_ADDR, I2C_CFG_MST_TX);

    I2CMasterStart(I2C_BASE_ADDR);

    /* Wait for the START to actually occir on the bus */
    while (0 == I2CMasterBusBusy(I2C_BASE_ADDR));

    I2CMasterDataPut(I2C_BASE_ADDR, (unsigned char)((offset >> 8) & 0xFF));

    /* Wait for the Tx register to be empty */
    while (0 == I2CMasterIntRawStatusEx(I2C_BASE_ADDR,
                                        I2C_INT_TRANSMIT_READY));

    /* Push offset out and tell CPLD from where we intend to read the data */
    I2CMasterDataPut(I2C_BASE_ADDR, (unsigned char)(offset & 0xFF));

    I2CMasterIntClearEx(I2C_BASE_ADDR, I2C_INT_TRANSMIT_READY);

    while(0 == (I2CMasterIntRawStatus(I2C_BASE_ADDR) & I2C_INT_ADRR_READY_ACESS));

    StatusClear();

    I2CSetDataCount(I2C_BASE_ADDR, length);

    /* Now that we have sent the register offset, start a RX operation*/
    I2CMasterControl(I2C_BASE_ADDR, I2C_CFG_MST_RX);

    /* Repeated start condition */
    I2CMasterStart(I2C_BASE_ADDR);

    while (length--)
    {
        while (0 == I2CMasterIntRawStatusEx(I2C_BASE_ADDR,
                                            I2C_INT_RECV_READY));
        data[idx++] = (unsigned char)I2CMasterDataGet(I2C_BASE_ADDR);
        I2CMasterIntClearEx(I2C_BASE_ADDR, I2C_INT_RECV_READY);
    }

    I2CMasterStop(I2C_BASE_ADDR);

    while(0 == (I2CMasterIntRawStatus(I2C_BASE_ADDR) & I2C_INT_STOP_CONDITION));

    I2CMasterIntClearEx(I2C_BASE_ADDR, I2C_INT_STOP_CONDITION);
}


/* Clear the status of all interrupts */
static void StatusClear(void)
{
    I2CMasterIntClearEx(I2C_BASE_ADDR,  0x7FF);
}

/* Pin Multiplexing for I2C0. */
static void I2C0PinMux(void)
{
    HWREG(SOC_CONTROL_REGS + CONTROL_CONF_I2C0_SDA)  =
         (CONTROL_CONF_I2C0_SDA_CONF_I2C0_SDA_RXACTIVE  |
          CONTROL_CONF_I2C0_SDA_CONF_I2C0_SDA_SLEWCTRL  |
          CONTROL_CONF_I2C0_SDA_CONF_I2C0_SDA_PUTYPESEL   );

    HWREG(SOC_CONTROL_REGS + CONTROL_CONF_I2C0_SCL)  =
         (CONTROL_CONF_I2C0_SCL_CONF_I2C0_SCL_RXACTIVE  |
          CONTROL_CONF_I2C0_SCL_CONF_I2C0_SCL_SLEWCTRL  |
          CONTROL_CONF_I2C0_SCL_CONF_I2C0_SCL_PUTYPESEL );
}

/* Read Data from EEPROM */
void BoardInfoRead(unsigned char *data)
{
    EEPROMI2CSetUp(I2C_SLAVE_ADDR);
    EEPROMI2CRead(data, MAX_DATA, EEPROM_OFFSET);
}
/****************************** End Of File *********************************/
