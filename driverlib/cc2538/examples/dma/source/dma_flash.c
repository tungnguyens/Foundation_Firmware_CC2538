/******************************************************************************
*  Filename:       dma_flash.c
*  Revised:        $Date: 2013-04-10 10:00:34 +0200 (on, 10 apr 2013) $
*  Revision:       $Revision: 9694 $
*
*  Description:    Show howto setup DMA and perform a SRAM to Flash transaction. 
*
*  Copyright (C) 2013 Texas Instruments Incorporated - http://www.ti.com/
*
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*    Redistributions of source code must retain the above copyright
*    notice, this list of conditions and the following disclaimer.
*
*    Redistributions in binary form must reproduce the above copyright
*    notice, this list of conditions and the following disclaimer in the
*    documentation and/or other materials provided with the distribution.
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
******************************************************************************/

#include <stdbool.h>
#include <stdint.h>
#include "hw_memmap.h"
#include "hw_ints.h"
#include "hw_flash_ctrl.h"
#include "flash.h"
#include "gpio.h"
#include "hw_ioc.h"
#include "ioc.h"
#include "sys_ctrl.h"
#include "udma.h"
#include "cpu.h"
#include "uartstdio.h"
#include "string.h"
#include "uart.h"
#include "debug.h"


//*****************************************************************************
// NOTE:
// This example has been run on a SmartRF06 evaluation board.  
//
//*****************************************************************************
//
//! \addtogroup dma_examples_list
//! <h1>uDMA Flash Transaction (dma_flash)</h1>
//!
//! This example shows how to set up the uDMA controller to perform a 
//! peripheral-initiated SRAM-to-flash transfer.
//!
//! This example uses the following peripherals and I/O signals.  You must
//! review these and change as needed for your own board:
//! - NONE
//!
//! The following UART signals are configured only for displaying console
//! messages for this example.  These are not required for operation of
//! uDMA or Flash.
//! - UART0 peripheral
//! - GPIO Port A peripheral (for UART0 pins)
//! - UART0RX - PA0
//! - UART0TX - PA1
//!
//! This example uses the following interrupt handlers.  To use this example
//! in your own application you may have to add these interrupt handlers to your
//! vector table.
//! - NONE
//!
//*****************************************************************************
#define EXAMPLE_PIN_UART_RXD            GPIO_PIN_0 
#define EXAMPLE_PIN_UART_TXD            GPIO_PIN_1 
#define EXAMPLE_GPIO_UART_BASE          GPIO_A_BASE

#define PAGE_SIZE                2048
#define PAGE_TO_ERASE            14
#define PAGE_TO_ERASE_START_ADDR (FLASH_BASE + (PAGE_TO_ERASE * PAGE_SIZE))

//*****************************************************************************
//
// The application must allocate the channel control table.
// This one is a full table for all modes and channels.
// NOTE: This table must be 1024-byte aligned.
//
//*****************************************************************************
#if defined(__IAR_SYSTEMS_ICC__)
#pragma data_alignment=1024
unsigned char ucDMAControlTable[1024];
#else
unsigned char ucDMAControlTable[1024] __attribute__ ((aligned(1024)));
#endif

//*****************************************************************************
//
// Source buffer used for the DMA transfer.
//
//*****************************************************************************
static uint32_t ucSourceBuffer[256];

//*****************************************************************************
//
// This function sets up UART0 to be used for a console to display information
// as the example is running.
//
//*****************************************************************************
void
InitConsole(void)
{
    //
    // Map UART signals to the correct GPIO pins and configure them as
    // hardware controlled.
    //
    IOCPinConfigPeriphOutput(EXAMPLE_GPIO_UART_BASE, EXAMPLE_PIN_UART_TXD, 
                             IOC_MUX_OUT_SEL_UART0_TXD);
    GPIOPinTypeUARTOutput(EXAMPLE_GPIO_UART_BASE, EXAMPLE_PIN_UART_TXD);
    
    IOCPinConfigPeriphInput(EXAMPLE_GPIO_UART_BASE, EXAMPLE_PIN_UART_RXD, 
                            IOC_UARTRXD_UART0);
    GPIOPinTypeUARTInput(EXAMPLE_GPIO_UART_BASE, EXAMPLE_PIN_UART_RXD);
     
    //
    // Initialize the UART (UART0) for console I/O.
    //
    UARTStdioInit(0);
}

//*****************************************************************************
//
// Main function, setup DMA and perform flash write. Verify the transaction.
//
//*****************************************************************************
int
main(void)
{
    uint16_t i;
    int32_t i32Res;
  
    //
    // Set the clocking to run directly from the external crystal/oscillator.
    // (no ext 32k osc, no internal osc)
    //
    SysCtrlClockSet(false, false, SYS_CTRL_SYSDIV_32MHZ);

    //
    // Set IO clock to the same as system clock
    //
    SysCtrlIOClockSet(SYS_CTRL_SYSDIV_32MHZ);
    
    
    //
    // Set up the serial console to use for displaying messages.  This is
    // just for this example program and is not needed for Systick operation.
    //
    InitConsole();

    //
    // Display the setup on the console.
    //
    UARTprintf("Example - Write to Flash using DMA.\n");
    
    //
    // Erase Flash page that will hold our transferred data
    //
    i32Res = FlashMainPageErase(PAGE_TO_ERASE_START_ADDR);
    ASSERT(i32Res==0);
    
    //
    // Fill Source buffer (to be copied to flash) with some data
    //
    for(i=0; i<256; i++)
    {
        ucSourceBuffer[i] = '0' + (i % 10);
    }
    
    //
    // Enable the uDMA controller.
    //
    uDMAEnable();
    
    //
    // Disable the uDMA channel to be used, before modifications are done.
    //
    uDMAChannelDisable(UDMA_CH2_FLASH);

    //
    // Set the base for the channel control table.
    //
    uDMAControlBaseSet(&ucDMAControlTable[0]);

    //
    // Assign the DMA channel
    //
    uDMAChannelAssign(UDMA_CH2_FLASH);
    
    //
    // Set attributes for the channel.
    //
    uDMAChannelAttributeDisable(UDMA_CH2_FLASH, UDMA_ATTR_HIGH_PRIORITY);

    //
    // Now set up the characteristics of the transfer.
    // 32-bit data size, with source increments in words (32 bits), 
    // no destination increment.
    // A bus arbitration size of 1 must be used.
    //
    uDMAChannelControlSet(UDMA_CH2_FLASH, UDMA_SIZE_32 | UDMA_SRC_INC_32 |
                          UDMA_DST_INC_NONE | UDMA_ARB_1);

    //
    // Set transfer parameters. 
    // Source address is the location of the data to write
    // and destination address is the FLASH_CTRL_FWDATA register.
    //
    uDMAChannelTransferSet(UDMA_CH2_FLASH, UDMA_MODE_BASIC, ucSourceBuffer, 
                           (void *) FLASH_CTRL_FWDATA, sizeof(ucSourceBuffer));

    //
    // Asure that the flash controller is not busy.
    //
    while(HWREG(FLASH_CTRL_FCTL) & FLASH_CTRL_FCTL_BUSY)
    {
    }
    
    //
    // Initialize Flash control register without changing the cache mode.
    //
    HWREG(FLASH_CTRL_FCTL) &= FLASH_CTRL_FCTL_CM_M;
    
    //
    // Setup Flash Address register to address of first data word (32-bit)
    //
    HWREG(FLASH_CTRL_FADDR) = PAGE_TO_ERASE_START_ADDR;
    
    //
    // Finally, the DMA channel must be enabled.
    //
    uDMAChannelEnable(UDMA_CH2_FLASH);
    
    //
    // Set FCTL.WRITE, to trigger flash write
    //
    HWREG(FLASH_CTRL_FCTL) |= FLASH_CTRL_FCTL_WRITE;
    
    //
    // Wait until all words has been programmed.
    //
    while( HWREG(FLASH_CTRL_FCTL) & FLASH_CTRL_FCTL_FULL )
    {
    }
    
    // 
    // Check if flash write was successfull
    //
    if (HWREG(FLASH_CTRL_FCTL) & FLASH_CTRL_FCTL_ABORT)
    {
        UARTprintf("Write not successful!\n");
    }
    else
    {
        UARTprintf("Write success!\n");
    }
    
    //
    // Set control register back to reset value without changing the cache mode.
    //
    HWREG(FLASH_CTRL_FCTL) &= FLASH_CTRL_FCTL_CM_M;
    

    //
    // Compare source buffer and destination flash page
    //
    if(memcmp(ucSourceBuffer, (void*) PAGE_TO_ERASE_START_ADDR, 256)==0)
    {
        UARTprintf("Buffer compares to flash page!\n");
    }
    else
    {
        UARTprintf("Buffer does not compare to flash page!\n");
    }
    
    //
    // We are done, loop forever
    //
    while(1)
    {
    }
}
