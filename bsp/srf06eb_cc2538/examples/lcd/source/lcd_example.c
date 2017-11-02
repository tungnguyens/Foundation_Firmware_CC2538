//*****************************************************************************
//! @file       lcd_example.c
//! @brief      LCD access example for CC2538 on SmartRF06EB. The SmartRF06EB
//!             LCD display is a DOGM128W-6 128x64 dot matrix.
//!
//!             In this example, use the directional keys (LEFT, RIGHT) on
//!             SmartRF06EB to switch between two different display buffers.
//!             Use the UP key to invert the bits of the displayed buffer.
//!
//!             @Warning This example will not work if \c LCD_NO_DEFAULT_BUFFER
//!             is defined!
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


/******************************************************************************
* INCLUDES
*/
#include "bsp.h"
#include "bsp_key.h"
#include "lcd_dogm128_6.h"
#include "sys_ctrl.h"           // Access to driverlib SysCtrl fns
#include "interrupt.h"          // Access to driverlib interrupt fns


/******************************************************************************
* DEFINES
*/


/******************************************************************************
* LOCAL VARIABLES AND FUNCTIONS
*/
char secondLcdBuf[LCD_BYTES];


/******************************************************************************
* FUNCTIONS
*/

/**************************************************************************//**
* @brief    Main function of example.
******************************************************************************/
void main(void)
{
    uint8_t ui8BtnPressed = 0;
    bool bShowingDefault = true;

    //
    // Initialize board
    //
    bspInit(BSP_SYS_CLK_SPD);

    //
    // Initialize SPI interface
    //
    bspSpiInit(BSP_SPI_CLK_SPD);

    //
    // Initialize keys driver
    //
    bspKeyInit(BSP_KEY_MODE_ISR);

    //
    // Initialize LCD
    //
    lcdInit();

    //
    // Clear local LCD buffers
    //
    lcdBufferClear(0);
    lcdBufferClear(secondLcdBuf);

    //
    // Write default buffer
    //
    lcdBufferPrintString(0, "Default Buffer", 1, eLcdPage0);
    lcdBufferInvertPage(0, 0, 127, eLcdPage0);
    lcdBufferPrintStringAligned(0, "Left aligned", eLcdAlignLeft, eLcdPage2);
    lcdBufferPrintStringAligned(0, "Center aligned", eLcdAlignCenter,
                                eLcdPage3);
    lcdBufferPrintStringAligned(0, "Right aligned", eLcdAlignRight, eLcdPage4);
    lcdBufferPrintStringAligned(0, "Press LEFT", eLcdAlignCenter, eLcdPage7);

    //
    // Write second buffer
    //
    lcdBufferPrintString(secondLcdBuf, "Second buffer", 1, eLcdPage0);
    lcdBufferInvertPage(secondLcdBuf, 0, 127, eLcdPage0);
    lcdBufferHArrow(secondLcdBuf, 7,120, 12);
    lcdBufferHArrow(secondLcdBuf, 120,7, 20);
    lcdBufferPrintStringAligned(secondLcdBuf, "SmartRF06EB", eLcdAlignCenter,
                                eLcdPage4);
    lcdBufferSetHLine(secondLcdBuf, 15, 112, 8*eLcdPage4-2);
    lcdBufferSetHLine(secondLcdBuf, 15, 112, 8*eLcdPage5);
    lcdBufferSetVLine(secondLcdBuf, 15, 8*eLcdPage4-2, 8*eLcdPage5);
    lcdBufferSetVLine(secondLcdBuf, 112, 8*eLcdPage4-2, 8*eLcdPage5);
    lcdBufferPrintStringAligned(secondLcdBuf, "Press RIGHT", eLcdAlignCenter,
                                eLcdPage7);

    //
    // Send the default buffer to LCD
    //
    lcdSendBuffer(0);

    //
    // Enable interrupts on needed keys
    //
    bspKeyIntEnable(BSP_KEY_LEFT|BSP_KEY_RIGHT|BSP_KEY_UP);

    //
    // Enable global interrupts
    //
    IntMasterEnable();

    while(1)
    {
        //
        // Check if button is pressed
        //
        ui8BtnPressed = bspKeyGetDir();
        if(ui8BtnPressed == BSP_KEY_EVT_LEFT && bShowingDefault)
        {
            //
            // Smooth leftward transition
            //
            lcdSendBufferAnimated(secondLcdBuf, 0, eLcdSlideLeft);
            bShowingDefault = false;
        }
        else if (ui8BtnPressed == BSP_KEY_EVT_RIGHT && !bShowingDefault)
        {
            //
            // Smooth rightward transition
            //
            lcdSendBufferAnimated(0, secondLcdBuf, eLcdSlideRight);
            bShowingDefault = true;
        }
        else if (ui8BtnPressed == BSP_KEY_EVT_UP)
        {
            //
            // Invert the currently active buffer and send to display
            //
            if(bShowingDefault)
            {
                lcdBufferInvert(0, 0,0 ,127,63);
                lcdSendBuffer(0);
            }
            else
            {
                lcdBufferInvert(secondLcdBuf, 0,0, 127,63);
                lcdSendBuffer(secondLcdBuf);
            }
        }
    }
}
