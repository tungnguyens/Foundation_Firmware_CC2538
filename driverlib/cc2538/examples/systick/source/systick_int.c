/******************************************************************************
*  Filename:       systick_int.c
*  Revised:        $Date: 2013-04-17 10:40:35 +0200 (Wed, 17 Apr 2013) $
*  Revision:       $Revision: 9797 $
*
*  Description:    Example demonstrating how to configure the systick 
*                  interrupt.
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
#include "gpio.h"
#include "hw_ioc.h"
#include "ioc.h"
#include "interrupt.h"
#include "sys_ctrl.h"
#include "systick.h"
#include "uartstdio.h"

//
// NOTE:
// This example has been run on a SmartRF06 evaluation board.  
// Using a UART window, watch the interrupt counter displayed on the consle.  
// The SysTick should interrupt every half second and increment a counter.
// The SysTick interrupt will run forever.
//
//*****************************************************************************
//
//! \addtogroup systick_examples_list
//! <h1>Systick Interrupt (systick_int)</h1>
//!
//! This example shows how to configure the SysTick and the SysTick interrupt.
//!
//! This example uses the following peripherals and I/O signals.  You must
//! review these and change as needed for your own board:
//! - NONE
//!
//! The following UART signals are configured only for displaying console
//! messages for this example.  These are not required for operation of
//! Systick.
//! - UART0 peripheral
//! - GPIO Port A peripheral (for UART0 pins)
//! - UART0RX - PA0
//! - UART0TX - PA1
//!
//! This example uses the following interrupt handlers.  To use this example
//! in your own application you can add these interrupt handlers to your
//! vector table.
//! - SysTickIntHandler
//!
//! \note The example registers the handler in dynamic fashion. This consumes
//!       RAM space as the vector table is allocated and copied to RAM.
//*****************************************************************************

#define EXAMPLE_PIN_UART_RXD            GPIO_PIN_0 
#define EXAMPLE_PIN_UART_TXD            GPIO_PIN_1 
#define EXAMPLE_GPIO_UART_BASE          GPIO_A_BASE

//*****************************************************************************
//
// Counter to count the number of interrupts that have been called.
//
//*****************************************************************************
volatile uint32_t g_ui32Counter = 0;

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
// The interrupt handler for the for Systick interrupt.
//
//*****************************************************************************
void
SysTickIntHandler(void)
{
    //
    // Update the Systick interrupt counter.
    //
    g_ui32Counter++;
}

//*****************************************************************************
//
// Configure the SysTick and SysTick interrupt with a period of 1 second.
//
//*****************************************************************************
int
main(void)
{
    uint32_t ui32PrevCount = 0;
    
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
    UARTprintf("SysTick Firing Interrupt ->");
    UARTprintf("\n   Rate = 0.5 sec\n\n");

    //
    // Initialize the interrupt counter.
    //
    g_ui32Counter = 0;

    //
    // Set up the period for the SysTick timer.  The SysTick timer period will
    // be equal to half the system clock, resulting in a period of 0.5 seconds.
    //
    SysTickPeriodSet(SysCtrlClockGet()/2);

    
    SysTickIntDisable();
    
    //
    // Use small interrupt map
    //
    IntAltMapEnable();
    
    //
    // The following call will result in a dynamic interrupt table being used.
    // The table resides in RAM.
    // Alternatively SysTickIntHandler can be statically registred in your
    // application.
    // 
    SysTickIntRegister(SysTickIntHandler);
    
    //
    // Enable interrupts to the processor.
    //
    IntMasterEnable();

    //
    // Enable the SysTick Interrupt.
    //
    SysTickIntEnable();

    //
    // Enable SysTick.
    //
    SysTickEnable();

    //
    // Loop forever while the SysTick runs.
    //
    while(1)
    {
        //
        // Check to see if systick interrupt count changed, and if so then
        // print a message with the count.
        //
        if(ui32PrevCount != g_ui32Counter)
        {
            //
            // Print the interrupt counter.
            //
            UARTprintf("Number of interrupts: %d\r", g_ui32Counter);
            ui32PrevCount = g_ui32Counter;
        }
    }
}
