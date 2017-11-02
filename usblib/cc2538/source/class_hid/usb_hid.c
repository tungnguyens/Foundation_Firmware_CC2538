//*****************************************************************************
//! @file       usb_hid.c
//! @brief      Application support for the HID class.
//!
//! Revised     $Date: 2013-04-08 13:14:23 +0200 (Mon, 08 Apr 2013) $
//! Revision    $Revision: 9658 $
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

#include "usb_firmware_library_headers.h"
#include "usb_hid_reports.h"
#include "usb_class_requests.h"
#include "usb_hid.h"

/******************************************************************************
* GLOBAL DATA
*/
KEYBOARD_IN_REPORT keyboard;
MOUSE_IN_REPORT mouse;


/** \brief Initializes the \ref module_usb_firmware_library_config module
 *
 * This function should be called first.
 */
void usbHidInit(void)
{
    //
    // Initialize the USB library
    //
    usbfwInit();

    //
    // Initialize the USB interrupt handler with bit mask containing all processed USBIRQ events
    //
    usbirqInit(USBIRQ_EVENT_RESET | USBIRQ_EVENT_SETUP | USBIRQ_EVENT_SUSPEND | USBIRQ_EVENT_RESUME);

    //
    // Activate the USB D+ pull-up resistor
    //
    UsbDplusPullUpEnable();

}


/** \brief Processes USB HID events.
 *
 * This function processes USB events. Calls application callback function
 * usbHidAppPoll().
 */
void usbHidProcessEvents(void)
{
    //
    // Handle USB resume
    //
    if(USBIRQ_GET_EVENT_MASK() & USBIRQ_EVENT_RESUME)
    {
        USBIRQ_CLEAR_EVENTS(USBIRQ_EVENT_RESUME);
    }

    //
    // Handle USB reset
    //
    if(USBIRQ_GET_EVENT_MASK() & USBIRQ_EVENT_RESET)
    {
        USBIRQ_CLEAR_EVENTS(USBIRQ_EVENT_RESET);
        usbfwResetHandler();
    }

    //
    // Handle USB suspend
    //
    if(USBIRQ_GET_EVENT_MASK() & USBIRQ_EVENT_SUSPEND)
    {
        USBIRQ_CLEAR_EVENTS(USBIRQ_EVENT_SUSPEND);
        usbsuspEnter();
    }

    //
    // Handle USB packets on EP0
    //
    if(USBIRQ_GET_EVENT_MASK() & USBIRQ_EVENT_SETUP)
    {
        USBIRQ_CLEAR_EVENTS(USBIRQ_EVENT_SETUP);
        usbfwSetupHandler();
    }

    //
    // Call application polling callback
    //
    usbHidAppPoll();
}


/** \brief Processes USB keyboard events.
 *
 */
void usbHidProcessKeyboard(uint8_t *pRfData)
{
    extern void halMcuWaitMs(uint16_t msec);
    uint8_t i;

    //
    // Copy data from RX buffer
    //
    keyboard.modifiers = pRfData[1];
    keyboard.reserved = pRfData[2];
    for(i = 0; i < sizeof(keyboard.pKeyCodes); i++)
    {
        keyboard.pKeyCodes[i] = pRfData[i + 3];
    }

    //
    // Update and send the received HID keyboard report if USB endpoint is ready
    //
    hidUpdateKeyboardInReport(&keyboard);

    if(hidSendKeyboardInReport())
    {
        //
        // For each successfully sent keyboard report, also clear the HID
        // keyboard report and send a blank report
        //
        keyboard.modifiers = 0;
        keyboard.reserved = 0;
        for(i = 0; i < sizeof(keyboard.pKeyCodes); i++)
        {
            keyboard.pKeyCodes[i] = 0;
        }

        hidUpdateKeyboardInReport(&keyboard);
        hidSendKeyboardInReport();
    }
}


/** \brief Processes USB mouse events.
 *
 */
void usbHidProcessMouse(uint8_t *pRfData)
{
    //
    // Copy data from RX buffer.
    //
    mouse.buttons = pRfData[1];
    if(mouse.buttons == 0)
    {
        mouse.dX = pRfData[2];
        mouse.dY = pRfData[3];
        mouse.dZ = pRfData[4];
    }

    //
    // Update and send the received HID mouse report if USB endpoint is ready
    //
    hidUpdateMouseInReport(&mouse);
    hidSendMouseInReport();
}
