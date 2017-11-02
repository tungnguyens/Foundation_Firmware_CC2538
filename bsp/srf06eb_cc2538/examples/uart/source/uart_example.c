//*****************************************************************************
//! @file       uart_example.c
//! @brief      Example of SmartRF06xB BSP UART functionality.
//!
//!             The example implements a simple transmitter and a UART
//!             repeater running at baud rate 115200. Use the SELECT key to
//!             switch between the two modes. On startup, LED1 (red) is lit.
//!             The example starts up in transmit mode. In repeater mode, LED3
//!             (green) is lit.
//!
//!             In transmitter mode (LED3 off), a single byte (0x34, ASCI "4")
//!             is sent when the UP key is pressed.
//!
//!             In repeater mode (LED3 on), data received on the \b BSP_UART_RXD
//!             line is transmitted back on the \b BSP_UART_TXD line.
//!
//!             By default, the BSP UART driver does not allocate interrupt
//!             service routines, so this must be done by the application. A
//!             basic UART ISR is provided in this example. UART ISR is needed
//!             if \b BSP_UART_ISR is defined. If \b BSP_UART_ALLOCATE_ISR is
//!             defined, the ISRs needed by the BSP UART driver will be
//!             allocated by the driver.
//!
//!             See usb_uart.c for available configuration options of the
//!             BSP UART driver.
//!
//! Revised     $Date: 2013-11-14 15:36:40 +0100 (Wed, 14 Nov 2013) $
//! Revision    $Revision: 8757 $
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
#include "bsp_uart.h"
#include "bsp_led.h"
#include "bsp_key.h"
#include "lcd_dogm128_6.h"

#include <interrupt.h>          // Access to driverlib interrupt fns
#include <uart.h>               // Access to driverlib uart fns


/******************************************************************************
* DEFINES
*/
//
// Byte sent by example in transmit mode
//
#define APP_TX_BYTE         0x34

/******************************************************************************
* LOCAL VARIABLES AND FUNCTIONS
*/
static uint8_t pui8TxBuf[16];
static uint8_t pui8RxBuf[16];
static uint8_t pui8AppBuf[16];

//
// Example mode
//
static bool bRepeaterMode = false;

//
// Function prototype (BSP_UART_ISR)
//
static void appUartIsr(void);


/******************************************************************************
* FUNCTIONS
*/

/**************************************************************************//**
* @brief    Main function of example.
******************************************************************************/
void main(void)
{
    uint8_t ui8KeyBm = 0;
    uint_fast16_t ui16Cnt = 0;
    uint8_t ui8Byte = APP_TX_BYTE;

    //
    // Initialize clocks and board I/O
    //
    bspInit(BSP_SYS_CLK_SPD);

    //
    // Set LED1 to indicate life
    //
    bspLedSet(BSP_LED_1);

    //
    // Initialize key driver
    //
    bspKeyInit(BSP_KEY_MODE_ISR);
    bspKeyIntEnable(BSP_KEY_SELECT|BSP_KEY_UP);

    //
    // Initialize UART to USB MCU
    //
    bspUartBufInit(pui8TxBuf, sizeof(pui8TxBuf), pui8RxBuf, sizeof(pui8RxBuf));

    //
    // Application must register the UART interrupt handler
    //
    UARTIntRegister(BSP_UART_BASE, &appUartIsr);

    //
    // Open UART connection
    //
    if(bspUartOpen(eBaudRate115200) != BSP_UART_SUCCESS)
    {
        //
        // Failed to initialize UART handler
        //
        bspAssert();
    }

    //
    // Initialize SPI interface to LCD, configure LCD, and display information.
    //
    bspSpiInit(BSP_SPI_CLK_SPD);
    lcdInit();
    lcdBufferPrintStringAligned(0, "UART example", eLcdAlignCenter, eLcdPage0);
    lcdBufferInvertPage(0, 0,127, eLcdPage0);
    lcdBufferPrintString(0, "Baud rate   :", 6, eLcdPage2);
    lcdBufferPrintIntAligned(0, bspUartBaudRateGet(), eLcdAlignRight, eLcdPage2);
    lcdBufferPrintString(0, "Format      :", 6, eLcdPage3);
    lcdBufferPrintStringAligned(0, "8-N-1", eLcdAlignRight, eLcdPage3);
    lcdBufferPrintString(0, "Flow control:", 6, eLcdPage4);
    lcdBufferPrintStringAligned(0, "No", eLcdAlignRight, eLcdPage4);
    lcdBufferPrintStringAligned(0, "Transmit: UP key", eLcdAlignRight, eLcdPage6);
    lcdBufferPrintStringAligned(0, "SELECT to toggle mode", eLcdAlignCenter, eLcdPage7);
    lcdBufferInvertPage(0, 0,127, eLcdPage7);
    lcdSendBuffer(0);

    //
    // Enable global interrupts
    //
    IntMasterEnable();

    while(1)
    {
        ui8KeyBm = bspKeyPushed(BSP_KEY_ALL);

        if(BSP_KEY_SELECT & ui8KeyBm)
        {
            //
            // Change mode
            //
            bRepeaterMode ^= 1;
            bspLedToggle(BSP_LED_3);

            //
            // Update LCD for the new mode
            //
            lcdBufferClearPart(0, 0,127, eLcdPage6, eLcdPage6);
            if(bRepeaterMode)
            {
                lcdBufferPrintStringAligned(0, "Repeater mode",
                                            eLcdAlignCenter, eLcdPage6);
            }
            else
            {
                lcdBufferPrintStringAligned(0, "Transmit: UP key",
                                            eLcdAlignCenter, eLcdPage6);
            }
            lcdSendBufferPart(0, 0,127, eLcdPage6, eLcdPage6);
        }

        //
        // Read data from UART RX buffer to application buffer
        //
        ui16Cnt = bspUartDataGet(pui8AppBuf, bspUartRxCharsAvail());

        if(bRepeaterMode)
        {
            //
            // Repeater mode
            //

            if(ui16Cnt)
            {
                //
                // Send data from application buffer to UART TX buffer
                //
                bspUartDataPut(pui8AppBuf, ui16Cnt);
            }
        }
        else
        {
            //
            // Transmit mode
            //

            if(BSP_KEY_UP & ui8KeyBm)
            {
                //
                // Transmit a single character
                //
                bspUartDataPut(&ui8Byte, 1);
            }
        }
    }
}


/**************************************************************************//**
* @brief    BSP UART interrupt service routine.
*           By default, the BSP UART driver does not allocate ISRs,
*           the application must provide the ISR for the UART RX/TX interrupts.
*           The BSP UART ISR handler must be called in the application's ISR
*           for the processing to be correct.
*
*           If \c BSP_UART_ALLOCATE_ISR and \c BSP_UART_ISR are defined,
*           bspUartOpen() will map bspUartIsrHandler() to the correct UART
*           interrupt vector.
*
*           See bsp.cfg in the bsp source directory for more info.
*
* @return   None.
******************************************************************************/
static void appUartIsr(void)
{
    uint32_t ulIntBm = UARTIntStatus(BSP_UART_BASE, 1);

    //
    // Serve interrupts handled by BSP UART interrupt handler
    //
    if(ulIntBm & (BSP_UART_INT_BM))
    {
        bspUartIsrHandler();
    }
}
