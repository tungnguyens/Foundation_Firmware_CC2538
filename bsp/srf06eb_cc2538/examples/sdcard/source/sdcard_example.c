//*****************************************************************************
//! @file       sdcard_example.c
//! @brief      Example of board support package SD card functionality.
//!
//!             @warning This example overwrites the content of the SD card.
//!             If the SD card is formatted as e.g. a FAT32 file system, the
//!             partition will be corrupted and data may be lost!
//!
//!             In this example, use the directional keys (UP, DOWN) on
//!             SmartRF06EB to read or write blocks of data to an SC card.
//!
//!             Use the UP key to write a local LCD buffer to the SD card. The
//!             local buffer ,\c lcdSrcBuffer, is altered prior to every write.
//!             Use the DOWN key to read data from the SD card to local buffer
//!             \c lcdDstBuffer and display \c lcdDstBuffer on the display.
//!
//!             LED status indication:
//!             If SD card initialization fails, all LEDs will blink.
//!             If CD card write fails: LED1 (red) will be lit and program
//!             enters an eternal loop.
//!             If SD card read fails: LED4 (red) will be lit and program
//!             enters an eternal loop.
//!             LED3 (green) will be toogled when either UP or DOWN keys are
//!             pushed.
//!
//! Revised     $Date: 2013-03-14 11:55:47 +0100 (on, 14 mar 2013) $
//! Revision    $Revision: 7035 $
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
#include "sdcard.h"
#include "bsp_led.h"
#include "bsp_key.h"
#include "lcd_dogm128_6.h"
#include "interrupt.h"          // Access to IntMasterEnable function


/******************************************************************************
* DEFINES
*/


/******************************************************************************
* LOCAL VARIABLES AND FUNCTIONS
*/
static char lcdSrcBuffer[1024]; // Buffer written to SD card
static char lcdDstBuffer[1024]; // Buffer to hold data from SD card


/******************************************************************************
* FUNCTIONS
*/

/**************************************************************************//**
* @brief    Main function of example.
******************************************************************************/
int main(void)
{
    uint32_t ui32CardSize, ui32Loop;
    uint_fast8_t cardStatus;
    uint_fast8_t keysPressed = 0;
    uint_fast8_t writeCounter = 0;

    //
    // Initialize board
    //
    bspInit(BSP_SYS_CLK_SPD);

    //
    // Initialize SPI interface
    //
    bspSpiInit(BSP_SPI_CLK_SPD);

    //
    // Initialize keys and enable interrupts for the UP/DOWN keys
    //
    bspKeyInit(BSP_KEY_MODE_ISR);
    bspKeyIntEnable(BSP_KEY_UP|BSP_KEY_DOWN);

    //
    // Enable global interrupts
    //
    IntMasterEnable();

    //
    // Init LCD, print header and send to display
    //
    lcdInit();
    lcdBufferPrintStringAligned(0, "SD card example", eLcdAlignCenter, eLcdPage0);
    lcdBufferInvertPage(0, 0,127, eLcdPage0);
    lcdSendBuffer(0);

    //
    // Initialize SD card
    //
    cardStatus = sdCardInit();
    if(cardStatus != SDCARD_SUCCESS)
    {
        //
        // Init failed!
        //
        lcdBufferPrintString(0, "Card init failed!", 0, eLcdPage3);
        lcdSendBufferPart(0, 0,127, eLcdPage3, eLcdPage3);
        bspAssert();
    }

    //
    // Read card size and print to LCD (converting from KiB to MiB)
    //
    ui32CardSize = sdCardGetSize();
    lcdBufferPrintStringAligned(0, "Card size (MiB):", eLcdAlignLeft,
                                eLcdPage1);
    lcdBufferPrintIntAligned(0, (ui32CardSize>>10), eLcdAlignRight, eLcdPage1);

    //
    // Inform user on what to do
    //
    lcdBufferPrintStringAligned(0, "UP: Write buf to SD", eLcdAlignLeft,
                                eLcdPage3);
    lcdBufferPrintStringAligned(0, "DOWN: Read SD to buf", eLcdAlignLeft,
                                eLcdPage4);
    lcdSendBufferPart(0, 0,127, eLcdPage1, eLcdPage4);

    //
    // Fill a second buffer (buffer to send to SD card)
    //
    lcdBufferPrintStringAligned(lcdSrcBuffer, "This data", eLcdAlignCenter,
                                eLcdPage2);
    lcdBufferPrintStringAligned(lcdSrcBuffer, "is from", eLcdAlignCenter,
                                eLcdPage3);
    lcdBufferPrintStringAligned(lcdSrcBuffer, "the memory card!",
                                eLcdAlignCenter, eLcdPage4);

    //
    // Infinite loop.
    //
    while(1)
    {
        //
        // Any keys pressed?
        //
        keysPressed = bspKeyGetDir();

        if(keysPressed == BSP_KEY_EVT_UP)
        {
            //
            // Toogle LED to indicate response
            //
            bspLedToggle(BSP_LED_3);

            //
            // Write a counter value to buffer
            //
            writeCounter++;
            lcdBufferClearPart(lcdSrcBuffer, 0,127, eLcdPage6,eLcdPage6);
            lcdBufferPrintIntAligned(lcdSrcBuffer, writeCounter,
                                     eLcdAlignCenter, eLcdPage6);

            //
            // Send data in lcdSrcBuffer to SD card (SD card block 0 and 1)
            //
            for(ui32Loop = 0; ui32Loop < 2; ui32Loop++)
            {
                cardStatus = sdCardBlockWrite(ui32Loop, (uint8_t *)           \
                                              &lcdSrcBuffer[512*ui32Loop]);
                if(cardStatus != SDCARD_SUCCESS)
                {
                    //
                    // Indicate error and entar an infinite loop
                    //
                    bspLedSet(BSP_LED_1);
                    while(1)
                    {
                    }
                }
            }
        }
        else if(keysPressed == BSP_KEY_EVT_DOWN)
        {

            //
            // Toggle LED to indicate response
            //
            bspLedToggle(BSP_LED_3);

            //
            // Read SD card to buffer lcdDstBuffer
            //
            for(ui32Loop = 0; ui32Loop < 2; ui32Loop++)
            {
                cardStatus = sdCardBlockRead(ui32Loop, (uint8_t *)            \
                                             &lcdDstBuffer[512*ui32Loop]);
                if(cardStatus != SDCARD_SUCCESS)
                {
                    //
                    // Indicate error and entar an infinite loop
                    //
                    bspLedSet(BSP_LED_4);
                    while(1)
                    {
                    }
                }
            }

            //
            // Send buffer to LCD
            //
            lcdSendBuffer(lcdDstBuffer);
        }

        //
        // Simple wait
        //
        for(ui32Loop = 0; ui32Loop < 100000; ui32Loop++)
        {
        }
    }
}
