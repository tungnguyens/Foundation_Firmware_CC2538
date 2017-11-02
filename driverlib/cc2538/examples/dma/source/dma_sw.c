/******************************************************************************
*  Filename:       edge_trigger_int.c
*  Revised:        $Date: 2013-05-02 13:57:37 +0200 (Thu, 02 May 2013) $
*  Revision:       $Revision: 9957 $
*
*  Description:    Setup a GPIO pin to interrupt on a falling edge.
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
#include "gpio.h"
#include "hw_ioc.h"
#include "ioc.h"
#include "interrupt.h"
#include "sys_ctrl.h"
#include "udma.h"
#include "cpu.h"
#include "uartstdio.h"
#include "string.h"


//*****************************************************************************
// NOTE:
// This example has been run on a SmartRF06 evaluation board.  
//
//*****************************************************************************
//
//! \addtogroup dma_examples_list
//! <h1>uDMA Interrupt (dma_sw)</h1>
//!
//! This example shows how to set up the uDMA controller to perform a 
//! software-initiated memory-to-memory transfer. On completion of the
//! transfer a DMA interrupt is triggered.
//!
//! This example uses the following peripherals and I/O signals.  You must
//! review these and change as needed for your own board:
//! - NONE
//!
//! The following UART signals are configured only for displaying console
//! messages for this example.  These are not required for operation of
//! uDMA.
//! - UART0 peripheral
//! - GPIO Port A peripheral (for UART0 pins)
//! - UART0RX - PA0
//! - UART0TX - PA1
//!
//! This example uses the following interrupt handlers.  To use this example
//! in your own application you may have to add these interrupt handlers to your
//! vector table.
//! - uDMAIntHandler
//!
//*****************************************************************************
#define EXAMPLE_PIN_UART_RXD            GPIO_PIN_0 
#define EXAMPLE_PIN_UART_TXD            GPIO_PIN_1 
#define EXAMPLE_GPIO_UART_BASE          GPIO_A_BASE

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
// Source and destination buffers used for the DMA transfer.
//
//*****************************************************************************
unsigned char ucSourceBuffer[256];
unsigned char ucDestBuffer[256];

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
// ISR called when uDMA software transfer is done.
// The function will compare the two buffers and report if move was done.
//
//*****************************************************************************
void 
uDMAIntHandler(void)
{
    if(memcmp(ucSourceBuffer, ucDestBuffer, 256)==0)
    {
        UARTprintf("Buffers compare!\n");
    }
    else
    {
        UARTprintf("Buffers do not compare!\n");
    }
}

//*****************************************************************************
//
// Configure the SysTick and SysTick interrupt with a period of 1 second.
//
//*****************************************************************************
int
main(void)
{
    uint16_t i;
  
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
    UARTprintf("DMA SW example mem to mem!\n");
    
    //
    // Fill Source buffer with some data
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
    // Set the base for the channel control table.
    //
    uDMAControlBaseSet(&ucDMAControlTable[0]);

    //
    // No attributes must be set for a software-based transfer.
    // The attributes are cleared by default, but are explicitly cleared
    // here, in case they were set elsewhere.
    //
    uDMAChannelAttributeDisable(UDMA_CH30_SW, UDMA_ATTR_ALL);
    
    //
    // Now set up the characteristics of the transfer for
    // 8-bit data size, with source and destination increments
    // in bytes, and a byte-wise buffer copy.  A bus arbitration
    // size of 8 is used.
    //
    uDMAChannelControlSet(UDMA_CH30_SW | UDMA_PRI_SELECT,
                          UDMA_SIZE_8 | UDMA_SRC_INC_8 |
                          UDMA_DST_INC_8 | UDMA_ARB_8);

    //
    // The transfer buffers and transfer size are now configured.
    // The transfer uses AUTO mode, which means that the
    // transfer automatically runs to completion after the first
    // request.
    //
    uDMAChannelTransferSet(UDMA_CH30_SW | UDMA_PRI_SELECT,
                           UDMA_MODE_AUTO, ucSourceBuffer, ucDestBuffer,
                           sizeof(ucDestBuffer));

    //
    // Enable Interrupt for DMA
    //
    IntEnable(INT_UDMA);
    
    //
    // Finally, the channel must be enabled.  Because this is a
    // software-initiated transfer, a request must also be made.
    // The request starts the transfer.
    //
    uDMAChannelEnable(UDMA_CH30_SW);
    uDMAChannelRequest(UDMA_CH30_SW);    
    
    //
    // Put cpu to sleep and wait for interrupt (when DMA transfer is done)
    //
    SysCtrlSleep();
    
    //
    // Loop forever while the SysTick runs.
    //
    while(1)
    {
    }
}
