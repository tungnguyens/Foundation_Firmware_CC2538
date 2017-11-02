/******************************************************************************
*  Filename:       adc_tempsens.c
*  Revised:        $Date: 2013-05-02 13:54:48 +0200 (Thu, 02 May 2013) $
*  Revision:       $Revision: 9956 $
*
*  Description:    Example demonstrating how to use ADC with temp sense.
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
#include <stdio.h>
#include "hw_memmap.h"
#include "gpio.h"
#include "hw_ioc.h"
#include "ioc.h"
#include "interrupt.h"
#include "adc.h"
#include "sys_ctrl.h"
#include "hw_sys_ctrl.h"
#include "systick.h"
#include "uartstdio.h"
#include "hw_cctest.h"
#include "hw_rfcore_xreg.h"

// NOTE:
// This example has been run on a SmartRF06 evaluation board.  
// Using a UART window, watch the adc readout displayed on the console.  
//
//*****************************************************************************
//
//! \addtogroup adc_examples_list
//! <h1>ADC Temperature Sensor (adc_tempsens)</h1>
//!
//! This example shows how to configure the ADC for using the internal  
//! temperature sensor. In this example no calibration has been performed, but  
//! using typical values from data sheet. 
//! In a real application calibration is needed to get accurate readings.
//!
//! This example uses the following peripherals and I/O signals.  You must
//! review these and change as needed for your own board:
//! - NONE
//!
//! The following UART signals are configured only for displaying console
//! messages for this example.  These are not required for operation of
//! ADC and temperature sensor.
//! - UART0 peripheral
//! - GPIO Port A peripheral (for UART0 pins)
//! - UART0RX - PA0
//! - UART0TX - PA1
//!
//! This example uses the following interrupt handlers. To use this example
//! in your own application you may have to add these interrupt handlers to your
//! vector table.
//! - None
//!
//*****************************************************************************
#define EXAMPLE_PIN_UART_RXD            GPIO_PIN_0 
#define EXAMPLE_PIN_UART_TXD            GPIO_PIN_1 
#define EXAMPLE_GPIO_UART_BASE          GPIO_A_BASE

//*****************************************************************************
//
// Constants needed for conversion of ADC readout to a temperature value.
//
//*****************************************************************************
#define CONST 0.58134 //(VREF / 2047) = (1190 / 2047), VREF from Datasheet
#define OFFSET_DATASHEET_25C 827 // 1422*CONST, from Datasheet 
#define TEMP_COEFF (CONST * 4.2) // From Datasheet
#define OFFSET_0C (OFFSET_DATASHEET_25C - (25 * TEMP_COEFF))

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
// Main function. Sets up the ADC to use the temperature sensor as input. Note 
// that you must enable to RF Core in order to use the ADC.
// The function runs a while forever loop converting the readout from ADC
// to temperature values and print it on the console.
//
//*****************************************************************************
int
main(void)
{
    uint16_t ui16Dummy;
    double dOutputVoltage;
    char pcTemp[20];

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
    // Enable RF Core (needed to enable temp sensor)
    //
    SysCtrlPeripheralEnable(SYS_CTRL_PERIPH_RFC);
    
    //
    // Set up the serial console to use for displaying messages.  This is
    // just for this example program and is not needed for Systick operation.
    //
    InitConsole();

    //
    // Display the setup on the console.
    //
    UARTprintf("ADC temp sens\n");
    
    //
    // Connect temp sensor to ADC
    //
    HWREG(CCTEST_TR0) |= CCTEST_TR0_ADCTM;

    //
    // Enable the temperature sensor 
    //
    HWREG(RFCORE_XREG_ATEST) = 0x01;
    
    //
    // Configure ADC, Internal reference, 512 decimation rate (12bit)
    //
    SOCADCSingleConfigure(SOCADC_12_BIT, SOCADC_REF_INTERNAL);
     
    //
    // Loop forever.
    //
    while(1)
    {
        //
        // Trigger single conversion on internal temp sensor
        //
        SOCADCSingleStart(SOCADC_TEMP_SENS);
        
        //
        // Wait until conversion is completed
        //
        while(!SOCADCEndOfCOnversionGet())
        {
        }

        //
        // Get data and shift down based on decimation rate
        //
        ui16Dummy = SOCADCDataGet() >> SOCADC_12_BIT_RSHIFT;
        
        //
        // Convert to temperature
        //
        dOutputVoltage = ui16Dummy * CONST;       
        dOutputVoltage = ((dOutputVoltage - OFFSET_0C) / TEMP_COEFF);
        
        //
        // Convert float to string
        //
        sprintf(pcTemp, "%.1f", dOutputVoltage);
                
        //
        // Print the result on UART
        //
        UARTprintf("ADC raw readout: %d\n", ui16Dummy);
        UARTprintf("Temperature: %s", pcTemp);
        UARTprintf(" C\n");
        
        //
        // Simple delay
        //
        for(int i=0;i<1000000;i++)
        {
        }
    }
}
