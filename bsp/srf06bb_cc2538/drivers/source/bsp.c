//*****************************************************************************
//! @file       bsp.c
//! @brief      Board support package for CC2538 Cortex devices on SmartRF06BB.
//!
//! Revised     $Date: 2013-04-11 20:13:31 +0200 (Thu, 11 Apr 2013) $
//! Revision    $Revision: 9715 $
//
//  Copyright (C) 2013 Texas Instruments Incorporated - http://www.ti.com/
//
//
//  Redistribution and use in source and binary forms, with or without
//  modification, are permitted provided that the following conditions
//  are met:
//
//    Redistributions of source code must retain the above copyright
//    notice, this list of conditions and the following disclaimer.
//
//    Redistributions in binary form must reproduce the above copyright
//    notice, this list of conditions and the following disclaimer in the
//    documentation and/or other materials provided with the distribution.
//
//    Neither the name of Texas Instruments Incorporated nor the names of
//    its contributors may be used to endorse or promote products derived
//    from this software without specific prior written permission.
//
//  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
//  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
//  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
//  A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
//  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
//  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
//  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
//  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
//  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
//  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
//  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//****************************************************************************/


/**************************************************************************//**
* @addtogroup bsp_api
* @{
******************************************************************************/


/******************************************************************************
* INCLUDES
*/
#include "bsp.h"
#include "hw_ioc.h"             // Access to IOC register defines
#include "hw_ssi.h"             // Access to SSI register defines
#include "ioc.h"                // Access to driverlib ioc fns
#include "gpio.h"               // Access to driverlib gpio fns
#include "sys_ctrl.h"           // Access to driverlib SysCtrl fns
#include "interrupt.h"          // Access to driverlib interrupt fns
#include "ssi.h"                // Access to driverlib ssi fns


/******************************************************************************
* LOCAL VARIABLES
*/


/******************************************************************************
* FUNCTIONS
*/

/**************************************************************************//**
* @brief    Function initializes the CC2538 clocks and I/O for use on
*           SmartRF06EB.
*
*           The function assumes an external crystal oscillator to be available
*           to the CC2538. The CC2538 system clock is set to the frequency given
*           by input argument \c ui32SysClockSpeed. The clock speed of other
*           internal clocks are set to the maximum value allowed based on the
*           system clock speed given by \c ui32SysClockSpeed.
*
*           If the value of \c ui32SysClockSpeed is invalid, the system clock
*           will be set to the highest allowed value.
*
* @param    ui32SysClockSpeed   The system clock speed in Hz. Must be one of
*                               the following:
*           \li \c SYS_CTRL_32MHZ
*           \li \c SYS_CTRL_16MHZ
*           \li \c SYS_CTRL_8MHZ
*           \li \c SYS_CTRL_4MHZ
*           \li \c SYS_CTRL_2MHZ
*           \li \c SYS_CTRL_1MHZ
*           \li \c SYS_CTRL_500KHZ
*           \li \c SYS_CTRL_250KHZ
*
* @return   None
******************************************************************************/
void
bspInit(uint32_t ui32SysClockSpeed)
{
    uint32_t ui32SysDiv;

    //
    // Disable global interrupts
    //
    bool bIntDisabled = IntMasterDisable();

    //
    // Determine sys clock divider and realtime clock
    //
    switch(ui32SysClockSpeed)
    {
    case SYS_CTRL_250KHZ:
        ui32SysDiv = SYS_CTRL_SYSDIV_250KHZ;
        break;
    case SYS_CTRL_500KHZ:
        ui32SysDiv = SYS_CTRL_SYSDIV_500KHZ;
        break;
    case SYS_CTRL_1MHZ:
        ui32SysDiv = SYS_CTRL_SYSDIV_1MHZ;
        break;
    case SYS_CTRL_2MHZ:
        ui32SysDiv = SYS_CTRL_SYSDIV_2MHZ;
        break;
    case SYS_CTRL_4MHZ:
        ui32SysDiv = SYS_CTRL_SYSDIV_4MHZ;
        break;
    case SYS_CTRL_8MHZ:
        ui32SysDiv = SYS_CTRL_SYSDIV_8MHZ;
        break;
    case SYS_CTRL_16MHZ:
        ui32SysDiv = SYS_CTRL_SYSDIV_16MHZ;
        break;
    case SYS_CTRL_32MHZ:
    default:
        ui32SysDiv = SYS_CTRL_SYSDIV_32MHZ;
        break;
    }

    //
    // Set system clock and realtime clock
    //
    SysCtrlClockSet(false, false, ui32SysDiv);

    //
    // Set IO clock to the same as system clock
    //
    SysCtrlIOClockSet(ui32SysDiv);

    //
    // LEDs (turn off)
    //
    GPIOPinTypeGPIOOutput(BSP_LED_BASE, BSP_LED_ALL);
    GPIOPinWrite(BSP_LED_BASE, BSP_LED_ALL, 0);

    //
    // Keys (input pullup)
    //
    GPIOPinTypeGPIOInput(BSP_KEY_DIR_BASE, BSP_KEY_DIR_ALL);
    IOCPadConfigSet(BSP_KEY_DIR_BASE, BSP_KEY_DIR_ALL, IOC_OVERRIDE_PUE);
    GPIOPinTypeGPIOInput(BSP_KEY_SEL_BASE, BSP_KEY_SELECT);
    IOCPadConfigSet(BSP_KEY_SEL_BASE, BSP_KEY_SELECT, IOC_OVERRIDE_PUE);

    //
    // Re-enable interrupt if initially enabled.
    //
    if(!bIntDisabled)
    {
        IntMasterEnable();
    }
}


/**************************************************************************//**
* @brief    Function initializes SPI interface. The SPI is configured to
*           Motorola mode with clock idle high and data valid on the second
*           (rising) edge. The SSI module uses the IO clock as clock source
*           (IO clock frequency set in bspInit()).
*
*           Input argument \c ui32SpiClockSpeed must obey the following criteria:
*           \li ui32SpiClockSpeed = srcClk / 2
*           where \c n is integer, \c n >= 2 and
*           \c srcClk is the clock frequency set in bspInit().
*
* @param    ui32SpiClockSpeed     The SPI clock speed in Hz.
*
* @return   None
******************************************************************************/
void
bspSpiInit(uint32_t ui32SpiClockSpeed)
{
    uint32_t ui32Dummy;

    //
    // Enable SSI peripheral module
    //
    SysCtrlPeripheralEnable(BSP_SPI_SSI_ENABLE_BM);

    //
    // Disable SSI function
    //
    SSIDisable(BSP_SPI_SSI_BASE);

    //
    // Set IO clock as SSI clock source
    //
    SSIClockSourceSet(BSP_SPI_SSI_BASE, SSI_CLOCK_PIOSC);

    //
    // Map SSI signals to the correct GPIO pins and configure them as HW ctrl'd
    //
    IOCPinConfigPeriphOutput(BSP_SPI_BUS_BASE, BSP_SPI_SCK,
                             IOC_MUX_OUT_SEL_SSI0_CLKOUT);
    IOCPinConfigPeriphOutput(BSP_SPI_BUS_BASE, BSP_SPI_MOSI,
                             IOC_MUX_OUT_SEL_SSI0_TXD);
    IOCPinConfigPeriphInput(BSP_SPI_BUS_BASE, BSP_SPI_MISO,
                            IOC_SSIRXD_SSI0);
    GPIOPinTypeSSI(BSP_SPI_BUS_BASE, (BSP_SPI_MOSI | BSP_SPI_MISO |           \
                                      BSP_SPI_SCK));

    //
    // Set SPI mode and speed
    //
    bspSpiClockSpeedSet(ui32SpiClockSpeed);

    //
    // Enable the SSI function
    //
    SSIEnable(BSP_SPI_SSI_BASE);

    //
    // Flush the RX FIFO
    //
    while(SSIDataGetNonBlocking(BSP_SPI_SSI_BASE, &ui32Dummy))
    {
    }
}


/**************************************************************************//**
* @brief    Function returns the clock speed of the BSP SPI interface.
*           It is assumed that the BSP SPI SSI module runs off the IO clock.
*
* @return   Returns clock speed in Hz.
******************************************************************************/
uint32_t
bspSpiClockSpeedGet(void)
{
    //
    // Fetch current SPI clock source and divider settings.
    //
    uint32_t ui32PreDiv, ui32Scr;
    ui32PreDiv = HWREG(BSP_SPI_SSI_BASE + SSI_O_CPSR);
    ui32Scr = ((HWREG(BSP_SPI_SSI_BASE + SSI_O_CR0) & 0x0000FF00) >> 8);

    return (SysCtrlIOClockGet() / (ui32PreDiv * (1 + ui32Scr)));
}


/**************************************************************************//**
* @brief    Function configures the SPI interface to the given clock speed,
*           Motorola mode with clock idle high and data valid on the second
*           (rising) edge. For proper SPI function, the SPI interface must
*           first be initialized using bspSpiInit().
*
* @warning  Limitations apply to the allowed values of \c ui32ClockSpeed. See
*           chip's Driverlib documentation for details.
*
* @param    ui32ClockSpeed      is the clock speed in Hz.
*
* @return   None
******************************************************************************/
void bspSpiClockSpeedSet(uint32_t ui32ClockSpeed)
{
    //
    // Disable SSI function
    //
    SSIDisable(BSP_SPI_SSI_BASE);

    //
    // Configure SSI module to Motorola/Freescale SPI mode 3:
    // Polarity  = 1, SCK steady state is high
    // Phase     = 1, Data changed on first and captured on second clock edge
    // Word size = 8 bits
    //
    SSIConfigSetExpClk(BSP_SPI_SSI_BASE, SysCtrlIOClockGet(), SSI_FRF_MOTO_MODE_3,
                       SSI_MODE_MASTER, ui32ClockSpeed, 8);

    //
    // Enable the SSI function
    //
    SSIEnable(BSP_SPI_SSI_BASE);
}


/**************************************************************************//**
* @brief    Assert function. Eternal loop that blinks all LEDs quickly.
*           Function assumes LEDs to be initializedby e.g. bspInit().
*
* @return   None
******************************************************************************/
void bspAssert(void)
{
    uint32_t ui32Delay = 1000000UL;
    uint8_t ui8SetLeds = 1;

    while(1)
    {
        if(ui8SetLeds)
        {
            GPIOPinWrite(BSP_LED_BASE, BSP_LED_ALL, BSP_LED_ALL);
        }
        else
        {
            GPIOPinWrite(BSP_LED_BASE, BSP_LED_ALL, 0);
        }
        SysCtrlDelay(ui32Delay);
        ui8SetLeds ^= 1;
    }
}


/**************************************************************************//**
* Close the Doxygen group.
* @}
******************************************************************************/
