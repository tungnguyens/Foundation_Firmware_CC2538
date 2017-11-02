/******************************************************************************
*  Filename:       periodic_prescaler_16bit.c
*  Revised:        $Date: 2013-04-10 11:03:42 +0200 (Wed, 10 Apr 2013) $
*  Revision:       $Revision: 9699 $
*
*  Description:    Example demonstrating a periodic 16-bit timer with a clock 
*                  prescaler.
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
// NOTE:
// This example has been run on a SmartRF06 evaluation board.  Using a UART
// window, watch the interrupt counter displayed on the consle.  This counter
// displays how many periodic interrupts have been received.  Once this counter
// is equal to the predefined NUMBER_OF_INTS, the counter should stop and the
// TIMER0B interrupt should be disabled.
//
//*****************************************************************************
//
//! \addtogroup timer_examples_list
//! <h1>16-Bit Periodic Timer using Prescaler (periodic_prescaler_16bit)</h1>
//!
//! This example shows how to configure Timer0B as a periodic timer with 
//! a clock prescaller of X.  This will allow us to configure the interrupt to
//! trigger every 1 second.  After NUMBER_OF_INTS interrupts, the Timer0B
//! interrupt will be disabled.
//!
//! This example uses the following peripherals and I/O signals.  You must
//! review these and change as needed for your own board:
//! - TIMER0 peripheral
//!
//! The following UART signals are configured only for displaying console
//! messages for this example.  These are not required for operation of
//! Timer0B.
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
// Number of interrupts before the timer gets turned off.
//
//*****************************************************************************
#define NUMBER_OF_INTS          10

//*****************************************************************************
//
// Counter to count the number of interrupts that have been called.
//
//*****************************************************************************
static volatile uint32_t g_ui32Counter;

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
    // Update the periodic interrupt counter.
    //
    g_ui32Counter++;

    //
    // Print the periodic interrupt counter.
    //
    UARTprintf("Number of interrupts: %d\r", g_ui32Counter);

    //
    // Once NUMBER_OF_INTS interrupts have been received, turn off the
    // GPTimer0B interrupt.
    //
    if(g_ui32Counter == NUMBER_OF_INTS)
    {
        //
        // Disable the GPTimer0B interrupt.
        //
        IntDisable(INT_TIMER0B);

        //
        // Turn off GPTimer0B interrupt.
        //    
        TimerIntDisable(GPTIMER0_BASE, GPTIMER_TIMB_TIMEOUT);       

        //
        // Clear any pending interrupt flag.
        //
        TimerIntClear(GPTIMER0_BASE, GPTIMER_TIMB_TIMEOUT);         

        //
        // Indicate that NUMBER_OF_INTS have been received.
        //
        UARTprintf("\n\n%d interrupts received!", NUMBER_OF_INTS);
        UARTprintf("\nTimer0B is now disabled.");
    }	 
}

//*****************************************************************************
//
// Configure GPTimer0B as a 16-bit periodic counter with a clcok prescaller. 
// GPTimer0B will be set to interrupt every second.
//
//*****************************************************************************
int
main(void)
{
    //
    // Initialize the interrupt counter.
    //
    g_ui32Counter = 0;
  
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
    UARTprintf("\n   Mode = Periodic a with timer clock prescaller of 255");
    UARTprintf("\n   Number of interrupts = %d", NUMBER_OF_INTS);
    UARTprintf("\n   Rate = 1sec\n\n");

    //
    // Configure GPTimer0B as a 16-bit periodic timer.
    //
    TimerConfigure(GPTIMER0_BASE, GPTIMER_CFG_SPLIT_PAIR | 
                   GPTIMER_CFG_A_PERIODIC |GPTIMER_CFG_B_PERIODIC);

    //
    // Configure GPTimer0B clock prescaler for 255. This will make the 
    // effective timer clock be SYSCLOCK / 255.
    //
    TimerPrescaleSet(GPTIMER0_BASE, GPTIMER_B, 255);

    //
    // Set the GPTimer0B load value to 1sec by setting the timer load value 
    // to SYSCLOCK / 255. This is determined by:
    //      Prescaled clock = 16Mhz / 255
    //      Cycles to wait = 1sec * Prescaled clock
    //
    TimerLoadSet(GPTIMER0_BASE, GPTIMER_B, SysCtrlClockGet() / 255);		

    //
    // The following call will result in a dynamic interrupt table being used.
    // The table resides in RAM.
    // Alternatively SysTickIntHandler can be statically registred in your
    // application.
    // 
    TimerIntRegister(GPTIMER0_BASE, GPTIMER_B, Timer0BIntHandler);    
    
    //
    // Enable processor interrupts.
    //
    IntMasterEnable();     

    //
    // Configure the GPTimer0B interrupt for timer timeout.
    //    
    TimerIntEnable(GPTIMER0_BASE, GPTIMER_TIMB_TIMEOUT);    

    //
    // Enable the GPTimer0B interrupt on the processor (NVIC).
    //
    IntEnable(INT_TIMER0B);

    //
    // Enable GPTimer0B.
    //
    TimerEnable(GPTIMER0_BASE, GPTIMER_B);    

    //
    // Loop forever while the Timer0B runs.
    //
    while(1)
    {
    }
}
