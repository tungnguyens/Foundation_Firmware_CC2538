//*****************************************************************************
//! @file       acc_example.c
//! @brief      Accelerometer access example for CC2538 on SmartRF06EB. The
//!             accelerometer on SmartRF06EB is a Bosch BMA250 with SPI
//!             interface.
//!
//!             In this example, the accelerometer is polled every ~200 ms and
//!             the values of the axes are displayed on the LCD.
//!
//!             If \c ACC_EXCLUDE_INTERRUPT_EXAMPLE is _not_ defined, the
//!             accelerometer is configured to toggle its interrupt pin whenever
//!             a double tap is detected. LEDs are toggled when an interrupt is
//!             triggered.
//!
//! Revised     $Date: 2013-04-11 19:50:45 +0200 (Thu, 11 Apr 2013) $
//! Revision    $Revision: 9709 $
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

//
// Uncomment to exclude double tap interrupt from this example example.
//
//#define ACC_EXCLUDE_INTERRUPT_EXAMPLE


/******************************************************************************
* INCLUDES
*/
#include "bsp.h"
#include "acc_bma250.h"
#include "lcd_dogm128_6.h"

#ifndef ACC_EXCLUDE_INTERRUPT_EXAMPLE
#include "bsp_led.h"
#include "interrupt.h"          // Access to driverlib interrupt fns
#include "io_pin_int.h"
#endif // ACC_EXCLUDE_INTERRUPT_EXAMPLE


/******************************************************************************
* DEFINES
*/


/******************************************************************************
* LOCAL VARIABLES AND FUNCTIONS
*/
#ifndef ACC_EXCLUDE_INTERRUPT_EXAMPLE
static void accIsr(void);
#endif


/******************************************************************************
* FUNCTIONS
*/

/**************************************************************************//**
* @brief    Main function of example.
******************************************************************************/
void main(void)
{
    volatile uint32_t ui32Loop;
    int16_t i16X, i16Y, i16Z;
#ifndef ACC_EXCLUDE_INTERRUPT_EXAMPLE
    uint8_t ui8RegVal;
#endif

    //
    // Initialize board
    //
    bspInit(BSP_SYS_CLK_SPD);

    //
    // Initialize SPI interface
    //
    bspSpiInit(BSP_SPI_CLK_SPD);

    //
    // Initialize LCD
    //
    lcdInit();

    //
    // Initialize accelerometer
    //
    accInit();

#ifndef ACC_EXCLUDE_INTERRUPT_EXAMPLE
    //
    // Configure accelerometer to generate interrupt on double tap.
    //

    //
    // Set tap threshold = 250mg (in 2g mode)
    //
    ui8RegVal = 0x04;
    accWriteReg(ACC_TAP_INT_SAMP_TH, &ui8RegVal, 1);

    //
    // Configure tap detection timing (second tap within 150 ms of the first)
    //
    ui8RegVal = 0x02;
    accWriteReg(ACC_TAP_INT_TIMING, &ui8RegVal, 1);

    //
    // Map double tap interrupt to acc's INT1 gpio
    //
    ui8RegVal = ACC_INT_MAP_D_TAP;
    accWriteReg(ACC_INT1_MAP, &ui8RegVal, 1);

    //
    // Set acc's INT1 as push-pull, active high
    //
    ui8RegVal = ACC_INT_CFG_INT1_ACTIVE_HI;
    accWriteReg(ACC_INT_PIN_CFG, &ui8RegVal, 1);

    //
    // Enable double tap interrupt
    //
    ui8RegVal = ACC_INT_EN0_D_TAP_EN;
    accWriteReg(ACC_INT_EN0, &ui8RegVal, 1);

    //
    // Configure CC2538 to accept interrupts on ACC interrupt pin1 (rising
    // edge), map interrupt handler and enable interrupts.
    //
    accIntTypeSet(BSP_ACC_INT1, GPIO_RISING_EDGE);
    accIntRegister(BSP_ACC_INT1, &accIsr);
    accIntEnable(BSP_ACC_INT1);
#endif // ACC_EXCLUDE_INTERRUPT_EXAMPLE

    //
    // Fill local LCD buffer with static text and send to display
    //
    lcdBufferPrintStringAligned(0, "Accelerometer Ex.", eLcdAlignCenter, eLcdPage0);
    lcdBufferInvertPage(0, 0, 127, eLcdPage0);
#ifndef ACC_EXCLUDE_INTERRUPT_EXAMPLE
    lcdBufferPrintStringAligned(0, "Double tap on board", eLcdAlignCenter, eLcdPage1);
#endif
    lcdBufferPrintString(0, "X-axis:", 0, eLcdPage3);
    lcdBufferPrintString(0, "Y-axis:", 0, eLcdPage4);
    lcdBufferPrintString(0, "Z-axis:", 0, eLcdPage5);
    lcdSendBuffer(0);

#ifndef ACC_EXCLUDE_INTERRUPT_EXAMPLE
    //
    // Enable global interrutps
    //
    IntMasterEnable();
#endif

    while(1)
    {
        //
        // Poll accelerometer for data
        //
        accReadData(&i16X, &i16Y, &i16Z);

        //
        // Clear part of buffer with axis values and update with new ones
        //
        lcdBufferClearPart(0, 10 * LCD_CHAR_WIDTH, 127, eLcdPage3, eLcdPage5);
        lcdBufferPrintInt(0, i16X, 10 * LCD_CHAR_WIDTH, eLcdPage3);
        lcdBufferPrintInt(0, i16Y, 10 * LCD_CHAR_WIDTH, eLcdPage4);
        lcdBufferPrintInt(0, i16Z, 10 * LCD_CHAR_WIDTH, eLcdPage5);

        //
        // Send updated parts of LCD buffer to display
        //
        lcdSendBufferPart(0, 10*LCD_CHAR_WIDTH, 127, eLcdPage3, eLcdPage5);

        //
        // Simple wait
        //
        for(ui32Loop = 0; ui32Loop < 250000; ui32Loop++)
        {
        }
    }
}


#ifndef ACC_EXCLUDE_INTERRUPT_EXAMPLE
/**************************************************************************//**
* @brief    This function is the interrupt service routine for accelerometer
*           interrupts.
******************************************************************************/
static void accIsr(void)
{
    //
    // Toggle LEDs
    //
    bspLedToggle(BSP_LED_ALL);
}
#endif // ACC_EXCLUDE_INTERRUPT_EXAMPLE
