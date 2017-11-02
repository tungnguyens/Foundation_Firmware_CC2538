/******************************************************************************
*  Filename:       edge_trigger_int.c
*  Revised:        $Date: 2013-04-10 10:04:18 +0200 (Wed, 10 Apr 2013) $
*  Revision:       $Revision: 9695 $
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
#include "hw_ioc.h"
#include "hw_ints.h"
#include "interrupt.h"
#include "gpio.h"
#include "ioc.h"
#include "uart.h"
#include "sys_ctrl.h"
#include "uartstdio.h"

//
// NOTE: Test this program on a SmartRF06 EM.  
// The interrupt is triggered by pressing select button. Load the program and 
// open up a console window.  Check that the interrupt count on the console 
// increases every time you press the on-board push button (it may increase more 
// than once due to the signal bouncing, this is OK).
//
//*****************************************************************************
//
//! \addtogroup gpio_examples_list
//! <h1>GPIO Edge Trigger Interrupt (edge_trigger_int)</h1>
//!
//! This example shows how to configure a GPIO pin to trigger an interrupt
//! when receiving a falling edge signal.
//!
//! This example uses the following peripherals and I/O signals.  You must
//! review these and change as needed for your own board:
//! - GPIO Port A peripheral (for GPIO Port A - Pin 3)
//!
//! The following UART signals are configured only for displaying console
//! messages for this example.  These are not required for operation of
//! GPIO edge trigger.
//! - UART0 peripheral
//! - GPIO Port A peripheral (for UART0 pins)
//! - UART0_RX - PA0
//! - UART0_TX - PA1
//!
//! This example uses the following interrupt handlers.
//! - INT_GPIOA- GPIOAIntHandler
//
//*****************************************************************************
#define EXAMPLE_PIN_UART_RXD            GPIO_PIN_0 
#define EXAMPLE_PIN_UART_TXD            GPIO_PIN_1 
#define EXAMPLE_GPIO_UART_BASE          GPIO_A_BASE

// Mapped to "SELECT" button on board
#define EXAMPLE_GPIO_BASE               GPIO_A_BASE
#define EXAMPLE_GPIO_PIN                GPIO_PIN_3
#define EXAMPLE_INT_GPIO                INT_GPIOA

//*****************************************************************************
//
// Variable to hold the interrupt count.
//
//*****************************************************************************
static uint32_t g_ui32GPIOJIntCount = 0;

//*****************************************************************************
//
// This is a flag that gets set in the interrupt handler to indicate that an
// interrupt occurred.
//
//*****************************************************************************
static volatile bool g_bGPIOIntFlag = false;

//*****************************************************************************
//
// The interrupt handler for the for GPIO J interrupt.
//
//*****************************************************************************
void
GPIOAIntHandler(void)
{
    uint32_t ui32GPIOIntStatus;

    //
    // Get the masked interrupt status.
    //
    ui32GPIOIntStatus = GPIOPinIntStatus(EXAMPLE_GPIO_BASE, true);

    //
    // Simple debounce function wait for button de-press
    //
    while(!GPIOPinRead(EXAMPLE_GPIO_BASE, EXAMPLE_GPIO_PIN))
    {
    }
    
    //
    // Acknowledge the GPIO  - Pin n interrupt by clearing the interrupt flag.
    //
    GPIOPinIntClear(EXAMPLE_GPIO_BASE, ui32GPIOIntStatus);

    //
    // Set an interrupt flag to indicate an interrupt has occurred.
    //
    g_bGPIOIntFlag = true;
}

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
// Configure Port A Pin 3 to interrupt on a falling edge.  The console will
// display a counter which will increment every time an interrupt occurs.
//
//*****************************************************************************
int
main(void)
{
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
    // just for this example program and is not needed for GPIO edge trigger
    // operation.
    //
    InitConsole();

    //
    // Display the example setup information on the console.
    //
    UARTprintf("GPIO edge interrupt ->");
    UARTprintf("\n   GPIO = Port A");
    UARTprintf("\n   Pin  = 3");
    UARTprintf("\n   Type = Falling edge.\n\n");

    //
    // Set Port A Pin 3 as an input.  This configures the pin as an input using
    // the standard push-pull mode.
    //
    GPIOPinTypeGPIOInput(EXAMPLE_GPIO_BASE, EXAMPLE_GPIO_PIN);
    IOCPadConfigSet(EXAMPLE_GPIO_BASE, EXAMPLE_GPIO_PIN, IOC_OVERRIDE_PUE);

    //
    // Set the type of interrupt for Port A - Pin 3.  In this example we will
    // use a falling edge.  You have the option to do any of the following:
    // rising edge, falling edge, both edges, low level, or high level.
    //
    GPIOIntTypeSet(EXAMPLE_GPIO_BASE, EXAMPLE_GPIO_PIN, GPIO_FALLING_EDGE);
   
    //
    // Enable the GPIO interrupt.
    //
    GPIOPinIntEnable(EXAMPLE_GPIO_BASE, EXAMPLE_GPIO_PIN);

    //
    // Enable processor interrupts.
    //
    IntMasterEnable();

    //
    // Enable the GPIOA interrupt on the processor (NVIC).
    //
    IntEnable(EXAMPLE_INT_GPIO);

    //
    // Wait for GPIO A interrupts.  They will only occur once a falling edge is
    // applied to GPIO A - Pin 3.
    //
    while(1)
    {
         UARTprintf("\r   Press again!");
        //
        // Wait for an interrupt to occur.
        //
        while(g_bGPIOIntFlag == false)
        {
        }
        
        //
        // Clear the interrupt flag.
        //
        g_bGPIOIntFlag = false;

        //
        // Update the interrupt count.
        //
        g_ui32GPIOJIntCount++;        
        
        //
        // Print the interrupt count to the terminal.
        //
        UARTprintf("\r   Interrupt Count = %d", g_ui32GPIOJIntCount);
    }
}
