/******************************************************************************
*  Filename:       master_slave_loopback.c
*  Revised:        $Date: 2013-04-10 11:06:40 +0200 (Wed, 10 Apr 2013) $
*  Revision:       $Revision: 9700 $
*
*  Description:    Example demonstrating a simple I2C message transmission 
*                  and reception.
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
#include "hw_i2cm.h"
#include "hw_i2cs.h"
#include "i2c.h"
#include "uartstdio.h"


//
// NOTE:
// This example has been run on a SmartRF06 evaluation board.  Using a UART 
// window, you will see printouts of the data being sent and the data being 
// received.
// If the data sent is not the same as the data received, there will be an
// error message printed to the screen.  If the data is correct the program
// will say it's done and return 0.
//
//*****************************************************************************
//
//! \addtogroup i2c_examples_list
//! <h1>I2C Master Loopback (i2c_master_slave_loopback)</h1>
//!
//! This example shows how to configure the I2C module for loopback mode.
//! This includes setting up the master and slave module.  Loopback mode
//! internally connects the master and slave data and clock lines together.
//! The address of the slave module is set in order to read data from the
//! master.  Then the data is checked to make sure the received data matches
//! the data that was transmitted.  This example uses a polling method for
//! sending and receiving data.
//!
//! This example uses the following peripherals and I/O signals.  You must
//! review these and change as needed for your own board:
//! - I2C peripheral
//! - GPIO Port B peripheral (for I2C pins)
//! - I2C_SCL - PB2
//! - I2C_SDA - PB3
//!
//! The following UART signals are configured only for displaying console
//! messages for this example.  These are not required for operation of I2C.
//! - UART0 peripheral
//! - GPIO Port A peripheral (for UART0 pins)
//! - UART0_RX - PA0
//! - UART0_TX - PA1
//!
//! This example uses the following interrupt handlers.  To use this example
//! in your own application you must add these interrupt handlers to your
//! vector table.
//! - None.
//
//*****************************************************************************
#define EXAMPLE_PIN_UART_RXD            GPIO_PIN_0 
#define EXAMPLE_PIN_UART_TXD            GPIO_PIN_1 
#define EXAMPLE_GPIO_UART_BASE          GPIO_A_BASE


#define EXAMPLE_PIN_I2C_SCL             GPIO_PIN_2
#define EXAMPLE_PIN_I2C_SDA             GPIO_PIN_3
#define EXAMPLE_GPIO_I2C_BASE           GPIO_B_BASE

//*****************************************************************************
//
// Number of I2C data packets to send.
//
//*****************************************************************************
#define NUM_I2C_DATA 3

//*****************************************************************************
//
// Set the address for slave module. This is a 7-bit address sent in the
// following format:
//                      [A6:A5:A4:A3:A2:A1:A0:RS]
//
// A zero in the "RS" position of the first byte means that the master
// transmits (sends) data to the selected slave, and a one in this position
// means that the master receives data from the slave.
//
//*****************************************************************************
#define SLAVE_ADDRESS 0x3C

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
// Configure the I2C0 master and slave and connect them using loopback mode.
//
//*****************************************************************************
int
main(void)
{
    uint32_t pui32DataTx[NUM_I2C_DATA];
    uint32_t pui32DataRx[NUM_I2C_DATA];
    uint32_t ui32Index;

    //
    // Set the clocking to run directly from the external crystal/oscillator.
    // (no ext 32k osc, no internal osc)
    //
    SysCtrlClockSet(false, false, SYS_CTRL_SYSDIV_16MHZ);

    //
    // Set IO clock to the same as system clock
    //
    SysCtrlIOClockSet(SYS_CTRL_SYSDIV_16MHZ);    
    
    //
    //  The I2C peripheral must be enabled before use.
    //
    SysCtrlPeripheralEnable(SYS_CTRL_PERIPH_I2C);
    
    //
    // Do reset of I2C module
    //
    SysCtrlPeripheralReset(SYS_CTRL_PERIPH_I2C);

    //
    // Configure I2C pins
    //
    GPIOPinTypeI2C(EXAMPLE_GPIO_I2C_BASE, EXAMPLE_PIN_I2C_SCL);
    GPIOPinTypeI2C(EXAMPLE_GPIO_I2C_BASE, EXAMPLE_PIN_I2C_SDA);

    //
    // Configure pins as peripheral input and output
    //
    IOCPinConfigPeriphInput(EXAMPLE_GPIO_I2C_BASE, EXAMPLE_PIN_I2C_SCL, 
                            IOC_I2CMSSCL);
    IOCPinConfigPeriphInput(EXAMPLE_GPIO_I2C_BASE, EXAMPLE_PIN_I2C_SDA,
                            IOC_I2CMSSDA);    
    IOCPinConfigPeriphOutput(EXAMPLE_GPIO_I2C_BASE, EXAMPLE_PIN_I2C_SCL,
                             IOC_MUX_OUT_SEL_I2C_CMSSCL);
    IOCPinConfigPeriphOutput(EXAMPLE_GPIO_I2C_BASE, EXAMPLE_PIN_I2C_SDA,
                             IOC_MUX_OUT_SEL_I2C_CMSSDA);          
        
    //
    // Enable loopback mode.  Loopback mode is a built in feature that is
    // useful for debugging I2C operations.  It internally connects the I2C
    // master and slave terminals, which effectively let's you send data as
    // a master and receive data as a slave.
    // NOTE: For external I2C operation you will need to use external pullups
    // that are stronger than the internal pullups.  Refer to the datasheet for
    // more information.
    //
    HWREG(I2CM_CR) |= I2CM_CR_LPBK;

    //
    // Enable and initialize the I2C master module.  Use the system clock for
    // the I2C module.  The last parameter sets the I2C data transfer rate.
    // If false the data rate is set to 100kbps and if true the data rate will
    // be set to 400kbps.  For this example we will use a data rate of 100kbps.
    //
    I2CMasterInitExpClk(SysCtrlClockGet(), false);

    //
    // Enable the I2C slave module. This module is enabled only for testing
    // purposes.  It does not need to be enabled for proper operation of the
    // I2Cx master module.
    //
    I2CSlaveEnable(); // TBD is this needed or done by I2CSlaveInit???

    //
    // Set the slave address to SLAVE_ADDRESS.  In loopback mode, it's an
    // arbitrary 7-bit number (set in a macro above) that is sent to the
    // I2CMasterSlaveAddrSet function.
    //
    I2CSlaveInit(SLAVE_ADDRESS);

    //
    // Tell the master module what address it will place on the bus when
    // communicating with the slave.  Set the address to SLAVE_ADDRESS
    // (as set in the slave module).  The receive parameter is set to false
    // which indicates the I2C Master is initiating a writes to the slave.  If
    // true, that would indicate that the I2C Master is initiating reads from
    // the slave.
    //
    I2CMasterSlaveAddrSet(SLAVE_ADDRESS, false);

        
    //
    // Set up the serial console to use for displaying messages.  This is
    // just for this example program and is not needed for I2C operation.
    //
    InitConsole();

    //
    // Display the example setup on the console.
    //
    UARTprintf("I2C Loopback Example ->");
    UARTprintf("\n   Module = I2C");
    UARTprintf("\n   Mode = Single Send/Receive");
    UARTprintf("\n   Rate = 100kbps\n\n");

    //
    // Initalize the data to send.
    //
    pui32DataTx[0] = 'I';
    pui32DataTx[1] = '2';
    pui32DataTx[2] = 'C';

    //
    // Initalize the receive buffer.
    //
    for(ui32Index = 0; ui32Index < NUM_I2C_DATA; ui32Index++)
    {
        pui32DataRx[ui32Index] = 0;
    }

    //
    // Indicate the direction of the data.
    //
    UARTprintf("Tranferring from: Master -> Slave\n");

    //
    // Send 3 peices of I2C data from the master to the slave.
    //
    for(ui32Index = 0; ui32Index < NUM_I2C_DATA; ui32Index++)
    {
        //
        // Display the data that the I2C master is transferring.
        //
        UARTprintf("  Sending: '%c'  . . .  ", pui32DataTx[ui32Index]);

        //
        // Place the data to be sent in the data register
        //
        I2CMasterDataPut(pui32DataTx[ui32Index]);

        //
        // Initiate send of data from the master.  Since the loopback
        // mode is enabled, the master and slave units are connected
        // allowing us to receive the same data that we sent out.
        //
        I2CMasterControl(I2C_MASTER_CMD_SINGLE_SEND);

        //
        // Wait until the slave has received and acknowledged the data.
        //
        while(!(I2CSlaveStatus() & I2C_SLAVE_ACT_RREQ))
        {
        }

        
        //
        // Read the data from the slave.
        //
        pui32DataRx[ui32Index] = I2CSlaveDataGet();

        //
        // Wait until master module is done transferring.
        //
        while(I2CMasterBusy())
        {
        }

        //
        // Display the data that the slave has received.
        //
        UARTprintf("Received: '%c'\n", pui32DataRx[ui32Index]);
    }

    
    //##### SELF TEST BEGIN #####
    
    //
    // Check that the data you sent was the same as the data that you received.
    //
    for(ui32Index = 0; ui32Index < NUM_I2C_DATA; ui32Index++)
    {
        if(pui32DataTx[ui32Index] != pui32DataRx[ui32Index])
        {
            //
            // Tell the user that the test failed.
            //
            UARTprintf("\n\nError: Data does not exactly match.\n");

            //
            // Wait in infinite loop for debugging.
            //
            while(1)
            {
            }
        }
    }
    
    //##### SELF TEST END #####
    
    
    //
    // Reset receive buffer.
    //
    for(ui32Index = 0; ui32Index < NUM_I2C_DATA; ui32Index++)
    {
        pui32DataRx[ui32Index] = 0;
    }

    //
    // Indicate the direction of the data.
    //
    UARTprintf("\n\nTranferring from: Slave -> Master\n");

    //
    // Modifiy the data direction to true, so that seeing the address will
    // indicate that the I2C Master is initiating a read from the slave.
    //
    I2CMasterSlaveAddrSet(SLAVE_ADDRESS, true);

    //
    // Do a dummy receive to make sure you don't get junk on the first receive.
    //
    I2CMasterControl(I2C_MASTER_CMD_SINGLE_RECEIVE);

    //
    // Dummy acknowledge and wait for the receive request from the master.
    // This is done to clear any flags that should not be set.
    //
    while(!(I2CSlaveStatus() & I2C_SLAVE_ACT_TREQ))
    {
    }

    for(ui32Index = 0; ui32Index < NUM_I2C_DATA; ui32Index++)
    {
        //
        // Display the data that I2C slave module is transferring.
        //
        UARTprintf("  Sending: '%c'  . . .  ", pui32DataTx[ui32Index]);

        //
        // Place the data to be sent in the data register
        //
        I2CSlaveDataPut(pui32DataTx[ui32Index]);

        //
        // Tell the master to read data.
        //
        I2CMasterControl(I2C_MASTER_CMD_SINGLE_RECEIVE);

        //
        // Wait until the slave is done sending data.
        //
        while(!(I2CSlaveStatus() & I2C_SLAVE_ACT_TREQ))
        {
        }

        //
        // Read the data from the master.
        //
        pui32DataRx[ui32Index] = I2CMasterDataGet();

        //
        // Display the data that the slave has received.
        //
        UARTprintf("Received: '%c'\n", pui32DataRx[ui32Index]);
    }

    //##### SELF TEST BEGIN #####
    
    //
    // Check that the data you sent was the same as the data that you received.
    //
    for(ui32Index = 0; ui32Index < NUM_I2C_DATA; ui32Index++)
    {
        if(pui32DataTx[ui32Index] != pui32DataRx[ui32Index])
        {
            //
            // Tell the user that the test failed.
            //
            UARTprintf("\n\nError: Data does not exactly match.\n");

            //
            // Wait in infinite loop for debugging.
            //
            while(1)
            {
            }
        }
    }
    
    //##### SELF TEST BEGIN #####
    
    //
    // Tell the user that the test is done.
    //
    UARTprintf("\nDone.\n\n");

    //
    // Wait in infinite loop.
    //
    while(1)
    {
    }
}
