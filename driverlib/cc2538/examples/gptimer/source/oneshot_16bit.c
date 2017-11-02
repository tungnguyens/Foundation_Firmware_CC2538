/******************************************************************************
*  Filename:       oneshot_16bit.c
*  Revised:        $Date: 2013-04-10 10:59:30 +0200 (Wed, 10 Apr 2013) $
*  Revision:       $Revision: 9698 $
*
*  Description:    Example demonstrating a one shot 16-bit timer.
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
#include "hw_ints.h"
#include "hw_memmap.h"
#include "gpio.h"
#include "interrupt.h"
#include "ioc.h"
#include "hw_ioc.h"
#include "sys_ctrl.h"
#include "gptimer.h"
#include "uartstdio.h"


//
// NOTES:
// This example has been run on a SmartRF06 evaluation board. Using a UART 
// window, after 1ms there should be an indicator on the console when the 
// oneshot interrupt is received.  Once the oneshot interrupt is received, the
// interrupt should not get called again.
//*****************************************************************************
//
//! \addtogroup timer_examples_list
//! <h1>16-Bit One-Shot Timer (oneshot_16bit)</h1>
//!
//! This example shows how to configure Timer0B as a one-shot timer with a
//! single interrupt triggering after 1ms.
//!
//! This example uses the following peripherals and I/O signals.  You must
//! review these and change as needed for your own board:
//! - TIMER0 peripheral
//!
//! The following UART signals are configured only for displaying console
//! messages for this example.  These are not required for operation of
//! Timer0.
//! - UART0 peripheral
//! - GPIO Port A peripheral (for UART0 pins)
//! - UART0RX - PA0
//! - UART0TX - PA1
//!
//! This example uses the following interrupt handlers.  
//! - INT_TIMER0B - Timer0BIntHandler
//
//*****************************************************************************

#define EXAMPLE_PIN_UART_RXD            GPIO_PIN_0 
#define EXAMPLE_PIN_UART_TXD            GPIO_PIN_1 
#define EXAMPLE_GPIO_UART_BASE          GPIO_A_BASE

//*****************************************************************************
//
// This is a flag that gets set in the interrupt handler to indicate that an
// interrupt occurred.
//
//*****************************************************************************
static volatile bool g_bIntFlag = false;

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
// The interrupt handler for the for Timer0B interrupt.
//
//*****************************************************************************
void
Timer0BIntHandler(void)
{
    //
    // Clear the timer interrupt flag.
    //
    TimerIntClear(GPTIMER0_BASE, GPTIMER_TIMB_TIMEOUT);

    //
    // Set a flag to indicate that the interrupt occurred.
    //
    g_bIntFlag = true;
}

//*****************************************************************************
//
// Configure Timer0B as a 16-bit one-shot counter with a single interrupt
// after 1ms.
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
    // The Timer0 peripheral must be enabled for use.
    //
    SysCtrlPeripheralEnable(SYS_CTRL_PERIPH_GPT0);

    //
    // Set up the serial console to use for displaying messages.  This is
    // just for this example program and is not needed for Timer operation.
    //
    InitConsole();

    //
    // Display the example setup on the console.
    //
    UARTprintf("16-Bit Timer Interrupt ->");
    UARTprintf("\n   Timer = Timer0B");
    UARTprintf("\n   Mode = One Shot");
    UARTprintf("\n   Rate = 1ms");

    //
    // The Timer0 peripheral must be enabled for use.
    //
    TimerConfigure(GPTIMER0_BASE, GPTIMER_CFG_SPLIT_PAIR | 
                   GPTIMER_CFG_A_ONE_SHOT | GPTIMER_CFG_B_ONE_SHOT);

    //
    // Set the Timer0B load value to 1ms.
    //
    TimerLoadSet(GPTIMER0_BASE, GPTIMER_B, SysCtrlClockGet() / 1000);
    
    //
    // The following call will result in a dynamic interrupt table being used.
    // The table resides in RAM.
    // Alternatively Timer0BIntHandler can be statically registred in your
    // application.
    // 
    TimerIntRegister(GPTIMER0_BASE, GPTIMER_B, Timer0BIntHandler);    
    
    //
    // Enable processor interrupts.
    //
    IntMasterEnable();

    //
    // Configure the Timer0B interrupt for timer timeout.
    //
    TimerIntEnable(GPTIMER0_BASE, GPTIMER_TIMB_TIMEOUT);

    //
    // Enable the Timer0B interrupt on the processor (NVIC).
    //
    IntEnable(INT_TIMER0B);

    //
    // Enable Timer0B.
    //
    TimerEnable(GPTIMER0_BASE, GPTIMER_B);

    //
    // Wait for interrupt to occur
    //
    while(!g_bIntFlag)
    {
    }

    //
    // Display a message indicating that the one shot interrupt was received.
    //
    UARTprintf("\n\nOne shot timer interrupt received.");

    //
    // Loop forever.
    //
    while(1)
    {
    }
}
