/******************************************************************************
*  Filename:       spi_master.c
*  Revised:        $Date: 2013-04-10 11:13:39 +0200 (Wed, 10 Apr 2013) $
*  Revision:       $Revision: 9701 $
*
*  Description:    Example demonstrating how to configure SSI0 in SPI master
*                  mode.
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
#include "gpio.h"
#include "ioc.h"
#include "ssi.h"
#include "uart.h"
#include "sys_ctrl.h"
#include "uartstdio.h"


//
// NOTE:
// This example has been run on a SmartRF06 evaluation board.  
// Short the Tx (PA5) and Rx (PA4) pins together.  
// The code sends data on the Tx line and with the Tx and Rx pins shorted 
// you are creating a loopback.  The code then verifies that the data you 
// received matches the code that you sent out.  Use the terminal window
// to see the results of the data sent and received.
//
//*****************************************************************************
//
//! \addtogroup ssi_examples_list
//! <h1>SPI Master (spi_master)</h1>
//!
//! This example shows how to configure the SSI0 as SPI Master.  The code will
//! send three characters on the master Tx then polls the receive FIFO until
//! 3 characters are received on the master Rx.
//!
//! This example uses the following peripherals and I/O signals.  You must
//! review these and change as needed for your own board:
//! - SSI0 peripheral
//! - GPIO Port A peripheral (for SSI0 pins)
//! - SSI0Clk - PA2
//! - SSI0Fss - PA3
//! - SSI0Rx  - PA4
//! - SSI0Tx  - PA5
//!
//! The following UART signals are configured only for displaying console
//! messages for this example.  These are not required for operation of SSI0.
//! - UART0 peripheral
//! - GPIO Port A peripheral (for UART0 pins)
//! - UART0RX - PA0
//! - UART0TX - PA1
//!
//! This example uses the following interrupt handlers.  To use this example
//! in your own application you must add these interrupt handlers to your
//! vector table.
//! - None.
//
//*****************************************************************************
#define EXAMPLE_PIN_SSI_CLK             GPIO_PIN_2 
#define EXAMPLE_PIN_SSI_FSS             GPIO_PIN_3
#define EXAMPLE_PIN_SSI_RX              GPIO_PIN_4
#define EXAMPLE_PIN_SSI_TX              GPIO_PIN_5
#define EXAMPLE_GPIO_SSI_BASE           GPIO_A_BASE
    
#define EXAMPLE_PIN_UART_RXD            GPIO_PIN_0 
#define EXAMPLE_PIN_UART_TXD            GPIO_PIN_1 
#define EXAMPLE_GPIO_UART_BASE          GPIO_A_BASE


//*****************************************************************************
//
// Number of bytes to send and receive.
//
//*****************************************************************************
#define NUM_SSI_DATA            3

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
// Configure SSI0 in master Freescale (SPI) mode.  This example will send out
// 3 bytes of data, then wait for 3 bytes of data to come in.  This will all be
// done using the polling method.
//
//*****************************************************************************
int
main(void)
{
    uint32_t pui32DataTx[NUM_SSI_DATA];
    uint32_t pui32DataRx[NUM_SSI_DATA];
    uint32_t ui32Index;

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
    // just for this example program and is not needed for SSI operation.
    //
    InitConsole();
    
    //
    // Display the setup on the console.
    //
    UARTprintf("SSI ->\n");
    UARTprintf("  Mode: SPI\n");
    UARTprintf("  Data: 8-bit\n\n");

    //
    // The SSI0 peripheral must be enabled for use.
    //
    SysCtrlPeripheralEnable(SYS_CTRL_PERIPH_SSI0);

    //
    // Disable SSI function before configuring module
    //
    SSIDisable(SSI0_BASE);

    //
    // Set IO clock as SSI clock source
    //
    SSIClockSourceSet(SSI0_BASE, SSI_CLOCK_PIOSC);

    //
    // For this example SSI0 is used with PA2-PA5.  The actual port and pins
    // used may be different on your part, consult the data sheet for more
    // information.  GPIO port A needs to be enabled so these pins can be used.
    // TODO: change this to whichever GPIO port you are using.
    //
    // Configure the pin muxing for SSI0 functions on port A2, A3, A4, and A5.
    // This step is not necessary if your part does not support pin muxing.
    // TODO: change this to select the port/pin you are using.
    //

    IOCPinConfigPeriphOutput(EXAMPLE_GPIO_SSI_BASE, EXAMPLE_PIN_SSI_CLK, 
                             IOC_MUX_OUT_SEL_SSI0_CLKOUT);    

    IOCPinConfigPeriphOutput(EXAMPLE_GPIO_SSI_BASE, EXAMPLE_PIN_SSI_FSS, 
                             IOC_MUX_OUT_SEL_SSI0_FSSOUT);
    
    IOCPinConfigPeriphOutput(EXAMPLE_GPIO_SSI_BASE, EXAMPLE_PIN_SSI_TX, 
                             IOC_MUX_OUT_SEL_SSI0_TXD);    
    
    IOCPinConfigPeriphInput(EXAMPLE_GPIO_SSI_BASE, EXAMPLE_PIN_SSI_RX, 
                            IOC_SSIRXD_SSI0);    
    //
    // Configure the GPIO settings for the SSI pins.  This function also gives
    // control of these pins to the SSI hardware.  Consult the data sheet to
    // see which functions are allocated per pin.
    // The pins are assigned as follows:
    //      PA5 - SSI0Tx
    //      PA4 - SSI0Rx
    //      PA3 - SSI0Fss
    //      PA2 - SSI0CLK
    // TODO: change this to select the port/pin you are using.
    //
    GPIOPinTypeSSI(EXAMPLE_GPIO_SSI_BASE, EXAMPLE_PIN_SSI_CLK | 
                   EXAMPLE_PIN_SSI_FSS | EXAMPLE_PIN_SSI_RX | 
                   EXAMPLE_PIN_SSI_TX);
    GPIOPinTypeSSI(EXAMPLE_GPIO_SSI_BASE, EXAMPLE_PIN_SSI_CLK | 
                   EXAMPLE_PIN_SSI_RX | EXAMPLE_PIN_SSI_TX);

    //
    // Configure SSI module to Motorola/Freescale SPI mode 3:
    // Polarity  = 1, SCK steady state is high
    // Phase     = 1, Data changed on first and captured on second clock edge
    // Word size = 8 bits
    //
    SSIConfigSetExpClk(SSI0_BASE, SysCtrlIOClockGet(), SSI_FRF_MOTO_MODE_3,
                       SSI_MODE_MASTER, SysCtrlClockGet()/2, 8);
    
    //
    // Enable the SSI0 module.
    //
    SSIEnable(SSI0_BASE);

    //
    // Read any residual data from the SSI port.  This makes sure the receive
    // FIFOs are empty, so we don't read any unwanted junk.  This is done here
    // because the SPI SSI mode is full-duplex, which allows you to send and
    // receive at the same time.  The SSIDataGetNonBlocking function returns
    // "true" when data was returned, and "false" when no data was returned.
    // The "non-blocking" function checks if there is any data in the receive
    // FIFO and does not "hang" if there isn't.
    //
    while(SSIDataGetNonBlocking(SSI0_BASE, &pui32DataRx[0]))
    {
    }

    //
    // Initialize the data to send.
    //
    pui32DataTx[0] = 's';
    pui32DataTx[1] = 'p';
    pui32DataTx[2] = 'i';

    //
    // Display indication that the SSI is transmitting data.
    //
    UARTprintf("Sent:\n  ");

    //
    // Send 3 bytes of data.
    //
    for(ui32Index = 0; ui32Index < NUM_SSI_DATA; ui32Index++)
    {
        //
        // Display the data that SSI is transferring.
        //
        UARTprintf("'%c' ", pui32DataTx[ui32Index]);

        //
        // Send the data using the "blocking" put function.  This function
        // will wait until there is room in the send FIFO before returning.
        // This allows you to assure that all the data you send makes it into
        // the send FIFO.
        //
        SSIDataPut(SSI0_BASE, pui32DataTx[ui32Index]);
    }

    //
    // Wait until SSI0 is done transferring all the data in the transmit FIFO.
    //
    while(SSIBusy(SSI0_BASE))
    {
    }

    //
    // Display indication that the SSI is receiving data.
    //
    UARTprintf("\nReceived:\n  ");

    //
    // Receive 3 bytes of data.
    //
    for(ui32Index = 0; ui32Index < NUM_SSI_DATA; ui32Index++)
    {
        //
        // Receive the data using the "blocking" Get function. This function
        // will wait until there is data in the receive FIFO before returning.
        //
        SSIDataGet(SSI0_BASE, &pui32DataRx[ui32Index]);

        //
        // Since we are using 8-bit data, mask off the MSB.
        //
        pui32DataRx[ui32Index] &= 0x00FF;

        //
        // Display the data that SSI0 received.
        //
        UARTprintf("'%c' ", pui32DataRx[ui32Index]);
    }

    //##### SELF CHECK BEGIN #####
    //
    // Check that the data you sent was the same as the data that you received.
    //
    for(ui32Index = 0; ui32Index < NUM_SSI_DATA; ui32Index++)
    {
        if(pui32DataTx[ui32Index] != pui32DataRx[ui32Index])
        {
            //
            // Tell the user that the test failed.
            //
            UARTprintf("\n\nError: Data does not exactly match.\n");
            UARTprintf("Check that Tx & Rx are shorted together.\n\n");

            //
            // Wait in infinite loop for debugging.
            //
            while(1)
            {
            }
        }
    }

    //
    // Tell the user that the test passed.
    //
    UARTprintf("\n\nTest Passed.\n\n");
    //##### SELF CHECK END #####
    
    //
    // Done - enter an infinite loop.
    //
    while(1)
    {
    }
}
